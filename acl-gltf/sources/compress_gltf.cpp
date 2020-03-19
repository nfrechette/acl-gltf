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
#include <acl/compression/compress.h>
#include <acl/compression/compression_settings.h>
#include <acl/compression/skeleton.h>
#include <acl/compression/skeleton_error_metric.h>
#include <acl/compression/track.h>
#include <acl/compression/track_error.h>
#include <acl/compression/utils.h>
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

struct gltf_channel_weight_mapping
{
	const tinygltf::AnimationChannel* channel;
	int buffer_view;
};

struct gltf_animation_buffer_view_mapping
{
	// The acl::CompressedClip
	int animation_clip_buffer_view_index;

	// The acl::compressed_track_list, one for each glTF animation channel that contains weight tracks
	std::vector<gltf_channel_weight_mapping> weight_track_list_buffer_view_indices;
};

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

	bool success;
	if (is_binary_gltf_filename(options.input_filename0))
		success = loader.LoadBinaryFromFile(&model, &err, &warn, options.input_filename0);
	else
		success = loader.LoadASCIIFromFile(&model, &err, &warn, options.input_filename0);

	if (!warn.empty())
		printf("Warn: %s\n", warn.c_str());

	if (!err.empty())
		printf("Err: %s\n", err.c_str());

	if (!success)
	{
		printf("Failed to parse glTF input: %s\n", options.input_filename0.c_str());
		return false;
	}

	if (!validate_input(model))
		return false;

	if (model.animations.empty())
	{
		printf("glTF input does not contain any animations, nothing to compress\n");
		return true;
	}

	//////////////////////////////////////////////////////////////////////////
	// ACL works with sample tracks. Each track has a type such as float1f, float3f, quatf, etc.
	// Transform type tracks can be hierarchical when they are stored in local space
	// of each other. This is generally the case for skeletal based animation.
	// ACL also supports multiple root transforms and can thus handle rigid body
	// simulations.
	//
	// A glTF file contains a set of nodes with each node having one rotation/translation/scale
	// track. Each node can also contain multiple scalar float tracks for blend weights or
	// other purposes (e.g. IK blend weight).
	//
	// This complicates things a bit. It means that we can have a single track list
	// for all our transform tracks that contains every node (our AnimationClip) per glTF animation.
	// However, we need one track list per node that contains scalar tracks.
	// As such, each glTF animation contains one AnimationClip (our transform tracks) and
	// zero or more scalar track lists for our weight tracks.
	//
	// Because glTF animations to not point to any particular buffer (the individual channels do),
	// we re-purpose the two buffer views within each channel. Each glTF animation channel points to
	// two buffer views: the list of time markers for every sample and the list of sample values for
	// the given channel (track). The convention the ACL extension uses is that the time marker
	// buffer view will always point to the animation clip buffer (every channel will point to this)
	// and the sample value buffer view will point to the weight track list buffer. This means that
	// when importing happens, we only need to read the first time buffer view metadata to figure out
	// if it contains ACL data and where to find the animation clip for every node. Because a channel
	// might not contain weight values and its sample buffer view cannot be empty/invalid, if it points
	// to the same buffer view as the time sampler then it does not contain weight scalar tracks.
	//
	// For now, even if an animation contains no transform tracks, an empty compressed clip will be generated
	// and stored. It is assumed to be always present.
	// Weight track lists are not stored when not present.
	//////////////////////////////////////////////////////////////////////////

	bool any_failed = false;
	bool any_compressed = false;

	acl::ANSIAllocator allocator;

	const hierarchy_description hierarchy = build_hierarchy(model);
	const acl::RigidSkeleton skeleton = build_skeleton(model, hierarchy, allocator);

	std::vector<gltf_animation_buffer_view_mapping> animation_buffer_view_mappings;
	std::vector<int> acl_buffer_view_indices;

	for (const tinygltf::Animation& animation : model.animations)
	{
		printf("Processing animation '%s' ...\n", animation.name.empty() ? "<unnamed>" : animation.name.c_str());

		const acl::AnimationClip clip = build_clip(model, animation, hierarchy, skeleton, allocator);
		const std::vector<channel_weight_animation> weight_animations = build_weight_tracks(model, animation, allocator);

		printf("    %.4f seconds (%u samples @ %.2f FPS)\n", clip.get_duration(), clip.get_num_samples(), clip.get_sample_rate());

		{
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
			const acl::BoneError bone_error = acl::calculate_error_between_clips(allocator, error_metric, clip, context);

			printf("    Largest error of %.2f mm on transform %u ('%s') @ %.2f sec\n", bone_error.error * 1000.0F, bone_error.index, skeleton.get_bone(bone_error.index).name.c_str(), bone_error.sample_time);

			{
				// Copy the compressed data into a new buffer
				tinygltf::Buffer compressed_buffer;
				compressed_buffer.name = animation.name + "_acl";
				compressed_buffer.data.resize(compressed_animation_size);
				std::memcpy(compressed_buffer.data.data(), compressed_clip, compressed_animation_size);
				model.buffers.push_back(std::move(compressed_buffer));
			}

			// Free the compressed data, we don't need it anymore
			allocator.deallocate(compressed_clip, compressed_animation_size);
			compressed_clip = nullptr;

			{
				// Add a buffer view
				tinygltf::BufferView compressed_buffer_view;
				compressed_buffer_view.name = animation.name + "_acl";
				compressed_buffer_view.buffer = static_cast<int>(model.buffers.size() - 1);
				compressed_buffer_view.byteOffset = 0;
				compressed_buffer_view.byteLength = compressed_animation_size;
				compressed_buffer_view.byteStride = 0;
				compressed_buffer_view.target = 0;	// Optional

				model.bufferViews.push_back(std::move(compressed_buffer_view));

				acl_buffer_view_indices.push_back(static_cast<int>(model.bufferViews.size() - 1));
			}
		}

		// Start building our mapping
		gltf_animation_buffer_view_mapping animation_mapping;
		animation_mapping.animation_clip_buffer_view_index = acl_buffer_view_indices.back();

		for (const channel_weight_animation& weight_animation : weight_animations)
		{
			acl::compression_settings weight_track_settings;

			acl::OutputStats stats;
			acl::compressed_tracks* tracks = nullptr;
			const acl::ErrorResult result = acl::compress_track_list(allocator, weight_animation.tracks, weight_track_settings, tracks, stats);
			if (result.any())
			{
				printf("Failed to compress animation weight tracks: %s\n", result.c_str());
				any_failed = true;
				break;
			}

			const uint32_t raw_tracks_size = weight_animation.tracks.get_raw_size();
			const uint32_t compressed_tracks_size = tracks->get_size();
			const float tracks_compression_ratio = float(raw_tracks_size) / float(compressed_tracks_size);

			printf("    Compressed weights %u glTF bytes into %u ACL bytes (ratio: %.2f : 1)\n", raw_tracks_size, compressed_tracks_size, tracks_compression_ratio);

			acl::decompression_context<acl::debug_decompression_settings> context;
			context.initialize(*tracks);

			const acl::track_error track_err = acl::calculate_compression_error(allocator, weight_animation.tracks, context);

			printf("    Largest error of %.4f on weight track %u @ %.2f sec\n", track_err.error, track_err.index, track_err.sample_time);

			{
				// Copy the compressed data into a new buffer
				tinygltf::Buffer compressed_buffer;
				compressed_buffer.name = animation.name + "_acl";
				compressed_buffer.data.resize(compressed_tracks_size);
				std::memcpy(compressed_buffer.data.data(), tracks, compressed_tracks_size);
				model.buffers.push_back(std::move(compressed_buffer));
			}

			// Free the compressed data, we don't need it anymore
			allocator.deallocate(tracks, compressed_tracks_size);
			tracks = nullptr;

			{
				// Add a buffer view
				tinygltf::BufferView compressed_buffer_view;
				compressed_buffer_view.name = animation.name + "_acl";
				compressed_buffer_view.buffer = static_cast<int>(model.buffers.size() - 1);
				compressed_buffer_view.byteOffset = 0;
				compressed_buffer_view.byteLength = compressed_tracks_size;
				compressed_buffer_view.byteStride = 0;
				compressed_buffer_view.target = 0;	// Optional

				model.bufferViews.push_back(std::move(compressed_buffer_view));

				acl_buffer_view_indices.push_back(static_cast<int>(model.bufferViews.size() - 1));
			}

			// Add channel mapping
			animation_mapping.weight_track_list_buffer_view_indices.push_back(gltf_channel_weight_mapping{ weight_animation.channel, acl_buffer_view_indices.back() });
		}

		if (any_failed)
			continue;

		animation_buffer_view_mappings.push_back(std::move(animation_mapping));

		any_compressed = true;
	}

	if (any_compressed)
	{
		const size_t num_animations = model.animations.size();

		// Keep track of the data we already reference, we'll invalidate it later when we are done
		std::vector<int> animation_buffer_views;
		std::vector<std::pair<size_t, int>> animation_samplers;
		std::vector<int> animation_accessors;
		for (size_t animation_index = 0; animation_index < num_animations; ++animation_index)
		{
			const tinygltf::Animation& animation = model.animations[animation_index];
			for (const tinygltf::AnimationChannel& channel : animation.channels)
			{
				const tinygltf::AnimationSampler& sampler = animation.samplers[channel.sampler];
				const tinygltf::Accessor& sample_time_accessor = model.accessors[sampler.input];
				const tinygltf::Accessor& sample_value_accessor = model.accessors[sampler.output];

				animation_buffer_views.push_back(sample_time_accessor.bufferView);
				animation_buffer_views.push_back(sample_value_accessor.bufferView);
				animation_samplers.push_back({ animation_index, channel.sampler });
				animation_accessors.push_back(sampler.input);
				animation_accessors.push_back(sampler.output);
			}
		}

		// Mark our old buffer views for removal and update to new buffer view
		for (size_t animation_index = 0; animation_index < num_animations; ++animation_index)
		{
			tinygltf::Animation& animation = model.animations[animation_index];
			const gltf_animation_buffer_view_mapping& mapping = animation_buffer_view_mappings[animation_index];

			// Every channel will share the same sample times accessor
			const int sample_time_accessor_index = static_cast<int>(model.accessors.size());

			{
				tinygltf::Accessor sample_time_accessor;
				sample_time_accessor.bufferView = mapping.animation_clip_buffer_view_index;
				sample_time_accessor.byteOffset = 0;
				sample_time_accessor.componentType = TINYGLTF_COMPONENT_TYPE_BYTE;
				sample_time_accessor.type = TINYGLTF_TYPE_SCALAR;
				sample_time_accessor.minValues.resize(1, -128.0);
				sample_time_accessor.maxValues.resize(1, 127.0);

				model.accessors.push_back(std::move(sample_time_accessor));
			}

			for (tinygltf::AnimationChannel& channel : animation.channels)
			{
				const int sample_value_accessor_index = static_cast<int>(model.accessors.size());

				{
					// Check if this channel has weight tracks
					auto channel_mapping = std::find_if(mapping.weight_track_list_buffer_view_indices.begin(), mapping.weight_track_list_buffer_view_indices.end(),
						[&](const gltf_channel_weight_mapping& entry)
						{
							return entry.channel == &channel;
						});

					tinygltf::Accessor sample_value_accessor;
					// Set our new buffer view and related settings
					// If we don't have scalar weights, use the same buffer view as our clip
					// otherwise use the scalar buffer view
					if (channel_mapping == mapping.weight_track_list_buffer_view_indices.end())
						sample_value_accessor.bufferView = mapping.animation_clip_buffer_view_index;
					else
						sample_value_accessor.bufferView = channel_mapping->buffer_view;

					sample_value_accessor.byteOffset = 0;
					sample_value_accessor.componentType = TINYGLTF_COMPONENT_TYPE_BYTE;
					sample_value_accessor.type = TINYGLTF_TYPE_SCALAR;
					sample_value_accessor.minValues.resize(1, -128.0);
					sample_value_accessor.maxValues.resize(1, 127.0);

					model.accessors.push_back(std::move(sample_value_accessor));
				}

				const int compressed_sampler_index = static_cast<int>(animation.samplers.size());

				{
					tinygltf::AnimationSampler sampler;
					sampler.input = sample_time_accessor_index;
					sampler.output = sample_value_accessor_index;
					sampler.interpolation = "LINEAR";	// TODO: Should we mark this with 'ACL' instead?

					animation.samplers.push_back(std::move(sampler));
				}

				channel.sampler = compressed_sampler_index;
			}
		}

		model.extensionsRequired.push_back(k_acl_gltf_extension_str);
		model.extensionsUsed.push_back(k_acl_gltf_extension_str);

		// Clear any metadata the animation used if we managed to compress everything.
		// Buffers can be shared by everything and as such we cannot remove or touch them.
		// If a single animation fails to compress, we do not remove any raw data.
		if (!any_failed)
		{
			for (int buffer_view_index : animation_buffer_views)
				reset_buffer_view(model.bufferViews[buffer_view_index]);

			for (const std::pair<size_t, int>& sampler_info : animation_samplers)
				reset_animation_sampler(model.animations[sampler_info.first].samplers[sampler_info.second]);

			for (int accessor_index : animation_accessors)
				reset_accessor(model.accessors[accessor_index]);
		}

		// We cannot remove the stale buffer and buffer view entries because we cannot safely remap old indices.
		// An extension we don't know about might have its own buffer view and reference it in a way that
		// isn't visible to us. As such, we can't remove any entries as this would change the indices.
		// We simply clear the entries for now.

		// If we have no output file, we'll just end up printing the stats which is fine
		if (!options.output_filename.empty())
		{
			const bool is_output_binary = is_binary_gltf_filename(options.output_filename);
			const bool embed_images = true;
			const bool embed_buffers = true;
			const bool pretty_print = true;
			const bool is_write_okay = loader.WriteGltfSceneToFile(&model, options.output_filename, embed_images, embed_buffers, pretty_print, is_output_binary);
			if (!is_write_okay)
			{
				printf("Failed to write output glTF file: %s\n", options.output_filename.c_str());
				return false;
			}
		}
	}

	return !any_failed;
}
