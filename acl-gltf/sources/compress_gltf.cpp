////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include "command_line_options.h"
#include "common_utils.h"
#include "compress_gltf.h"
#include "tinygltf_impl.h"

#include <acl/algorithm/uniformly_sampled/decoder.h>
#include <acl/algorithm/uniformly_sampled/encoder.h>
#include <acl/compression/animation_clip.h>
#include <acl/compression/compression_settings.h>
#include <acl/compression/skeleton.h>
#include <acl/compression/skeleton_error_metric.h>
#include <acl/compression/track.h>
#include "acl/compression/utils.h"
#include <acl/core/ansi_allocator.h>
#include <acl/core/utils.h>

#include <rtm/scalarf.h>

#include <cmath>
#include <limits>
#include <vector>

static bool validate_input(const tinygltf::Model& model)
{
	if (model.asset.version != "2.0")
	{
		printf("Expected a glTF 2.0 input\n");
		return false;
	}

	if (model.nodes.empty())
	{
		printf("glTF input does not contain any nodes, nothing to compress\n");
		return false;
	}

	if (model.nodes.size() >= std::numeric_limits<uint16_t>::max())
	{
		printf("glTF input contains more nodes than ACL can handle\n");
		return false;
	}

	if (model.animations.empty())
	{
		printf("glTF input does not contain any animations, nothing to compress\n");
		return false;
	}

	if (model.animations.size() >= std::numeric_limits<uint16_t>::max())
	{
		printf("glTF input contains more animations than ACL can handle\n");
		return false;
	}

	if (std::find(model.extensionsUsed.begin(), model.extensionsUsed.end(), k_acl_gltf_extension_str) != model.extensionsUsed.end())
	{
		printf("Cannot compress a glTF input that already uses ACL\n");
		return false;
	}

	return true;
}

bool compress_gltf(const command_line_options& options)
{
	if (options.input_filename0 == options.output_filename)
	{
		printf("Input and output cannot be the same file\n");
		return false;
	}

	tinygltf::Model model;
	tinygltf::TinyGLTF loader;

	std::string err;
	std::string warn;

	const bool success = loader.LoadASCIIFromFile(&model, &err, &warn, options.input_filename0);

	if (!warn.empty())
		printf("Warn: %s\n", warn.c_str());

	if (!err.empty())
		printf("Err: %s\n", err.c_str());

	if (!success)
	{
		printf("Failed to parse input glTF\n");
		return false;
	}

	if (!validate_input(model))
		return false;

	bool any_failed = false;
	bool any_compressed = false;
	std::vector<int> animation_buffers;

	acl::ANSIAllocator allocator;

	const std::vector<uint16_t> node_parent_indices = build_node_parent_indices(model);
	const acl::RigidSkeleton skeleton = build_skeleton(model, node_parent_indices, allocator);

	for (tinygltf::Animation& animation : model.animations)
	{
		printf("Processing animation '%s' ...\n", animation.name.empty() ? "<unnamed>" : animation.name.c_str());

		const acl::AnimationClip clip = build_clip(model, animation, skeleton, allocator);

		printf("    %.4f seconds (%u samples @ %.2f FPS)\n", clip.get_duration(), clip.get_num_samples(), clip.get_sample_rate());

		acl::CompressionSettings settings = acl::get_default_compression_settings();

		// glTF units are in meters but ACL default thresholds are in centimeters
		settings.constant_translation_threshold /= 100.0F;
		settings.error_threshold /= 100.0F;

		acl::qvvf_transform_error_metric error_metric;
		settings.error_metric = &error_metric;

		// Compress our animation
		acl::OutputStats stats;
		acl::CompressedClip* compressed_clip = nullptr;
		const acl::ErrorResult result = acl::uniformly_sampled::compress_clip(allocator, clip, settings, compressed_clip, stats);

		if (result.any())
		{
			printf("Failed to compress animation: %s\n", result.c_str());
			any_failed = true;
			continue;
		}

		const uint32_t raw_animation_size = get_raw_animation_size(model, animation);
		const uint32_t compressed_animation_size = compressed_clip->get_size();
		const float compression_ratio = float(raw_animation_size) / float(compressed_animation_size);

		printf("    Compressed %u glTF bytes into %u ACL bytes (ratio: %.2f : 1)\n", raw_animation_size, compressed_animation_size, compression_ratio);

		// Measure the compression error
		acl::uniformly_sampled::DecompressionContext<acl::uniformly_sampled::DebugDecompressionSettings> context;
		context.initialize(*compressed_clip);
		const acl::BoneError bone_error = calculate_compressed_clip_error(allocator, clip, error_metric, context);

		printf("    Largest error of %.2f mm on transform %u ('%s') @ %.2f sec\n", bone_error.error * 100.0F * 100.0F, bone_error.index, skeleton.get_bone(bone_error.index).name.c_str(), bone_error.sample_time);

		// Copy the compressed data into a new buffer
		tinygltf::Buffer compressed_buffer;
		compressed_buffer.name = animation.name + "_acl";
		compressed_buffer.data.resize(compressed_animation_size);
		std::memcpy(compressed_buffer.data.data(), compressed_clip, compressed_animation_size);
		model.buffers.push_back(std::move(compressed_buffer));

		// Free the compressed data, we don't need it anymore
		allocator.deallocate(compressed_clip, compressed_animation_size);

		// Add a buffer view
		tinygltf::BufferView compressed_buffer_view;
		compressed_buffer_view.name = animation.name + "_acl";
		compressed_buffer_view.buffer = static_cast<int>(model.buffers.size() - 1);
		compressed_buffer_view.byteOffset = 0;
		compressed_buffer_view.byteLength = compressed_animation_size;
		compressed_buffer_view.byteStride = 0;
		compressed_buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

		model.bufferViews.push_back(std::move(compressed_buffer_view));

		// Mark our old buffer views for removal and update to new buffer view
		for (tinygltf::AnimationChannel& channel : animation.channels)
		{
			tinygltf::AnimationSampler& sampler = animation.samplers[channel.sampler];
			tinygltf::Accessor& sample_time_accessor = model.accessors[sampler.input];
			tinygltf::Accessor& sample_value_accessor = model.accessors[sampler.output];

			// Add the buffers we use so we can clear them later
			animation_buffers.push_back(model.bufferViews[sample_time_accessor.bufferView].buffer);
			animation_buffers.push_back(model.bufferViews[sample_value_accessor.bufferView].buffer);

			// Clear our old buffer views
			reset_buffer_view(model.bufferViews[sample_time_accessor.bufferView]);
			reset_buffer_view(model.bufferViews[sample_value_accessor.bufferView]);

			// Set our new buffer view and related settings
			sample_time_accessor.bufferView = static_cast<int>(model.bufferViews.size() - 1);
			sample_value_accessor.bufferView = static_cast<int>(model.bufferViews.size() - 1);
			sample_time_accessor.byteOffset = 0;
			sample_value_accessor.byteOffset = 0;
			sample_time_accessor.componentType = TINYGLTF_COMPONENT_TYPE_BYTE;
			sample_value_accessor.componentType = TINYGLTF_COMPONENT_TYPE_BYTE;
			sample_time_accessor.type = TINYGLTF_TYPE_SCALAR;
			sample_value_accessor.type = TINYGLTF_TYPE_SCALAR;
		}

		any_compressed = true;
	}

	if (any_compressed)
	{
		model.extensionsRequired.push_back(k_acl_gltf_extension_str);
		model.extensionsUsed.push_back(k_acl_gltf_extension_str);

		// Clear any buffers the animation data used if we managed to compress everything.
		// This assumes that buffers referenced by raw animation data contain only raw animation data
		// and that it is no longer needed once compressed.
		// If a single animation fails to compress, we do not remove any raw data.
		if (!any_failed)
		{
			for (int buffer_index : animation_buffers)
				reset_buffer(model.buffers[buffer_index]);
		}

		// We cannot remove the stale buffer and buffer view entries because we cannot safely remap old indices.
		// An extension we don't know about might have its own buffer view and reference it in a way that
		// isn't visible to us. As such, we can't remove any entries as this would change the indices.
		// We simply clear the entries for now.

		// If we have no output file, we'll just end up printing the stats which is fine
		if (!options.output_filename.empty())
		{
			const size_t gltf_ext_start_pos = options.output_filename.rfind(".gltf");
			const bool is_binary = gltf_ext_start_pos == std::string::npos;
			const bool embed_images = true;
			const bool embed_buffers = true;
			const bool pretty_print = true;
			const bool is_write_okay = loader.WriteGltfSceneToFile(&model, options.output_filename, embed_images, embed_buffers, pretty_print, is_binary);
			if (!is_write_okay)
			{
				printf("Failed to write output glTF file: %s\n", options.output_filename.c_str());
				return false;
			}
		}
	}

	return !any_failed;
}
