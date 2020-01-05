////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2020 Nicholas Frechette
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
#include "decompress_gltf.h"
#include "tinygltf_impl.h"

#include <acl/algorithm/uniformly_sampled/decoder.h>
#include <acl/compression/skeleton.h>
#include <acl/compression/skeleton_error_metric.h>
#include <acl/compression/track.h>
#include <acl/compression/track_array.h>
#include "acl/compression/utils.h"
#include "acl/decompression/default_output_writer.h"
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
		printf("glTF input does not contain any nodes, nothing to decompress\n");
		return false;
	}

	if (model.nodes.size() >= std::numeric_limits<uint16_t>::max())
	{
		printf("glTF input contains more nodes than ACL can handle\n");
		return false;
	}

	if (model.animations.empty())
	{
		printf("glTF input does not contain any animations, nothing to decompress\n");
		return false;
	}

	if (model.animations.size() >= std::numeric_limits<uint16_t>::max())
	{
		printf("glTF input contains more animations than ACL can handle\n");
		return false;
	}

	if (std::find(model.extensionsUsed.begin(), model.extensionsUsed.end(), k_acl_gltf_extension_str) == model.extensionsUsed.end())
	{
		printf("Cannot decompress a glTF input that doesn't use ACL\n");
		return false;
	}

	return true;
}

static void setup_channel(tinygltf::Animation& animation, const char* target_path, int transform_index, int channel_index)
{
	tinygltf::AnimationChannel& channel = animation.channels[channel_index];
	channel.target_node = transform_index;
	channel.target_path = target_path;
	channel.sampler = channel_index;
}

static void setup_sampler(tinygltf::Model& model, tinygltf::AnimationSampler& sampler, int sample_time_buffer_view_index, int animation_buffer_index, uint32_t num_samples, size_t& animation_buffer_byte_offset, int sample_type, size_t sample_size)
{
	sampler.input = static_cast<int>(model.accessors.size());
	sampler.output = sampler.input + 1;
	sampler.interpolation = "LINEAR";

	tinygltf::Accessor sample_time_accessor;
	sample_time_accessor.bufferView = sample_time_buffer_view_index;
	sample_time_accessor.byteOffset = 0;
	sample_time_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
	sample_time_accessor.count = num_samples;
	//sample_time_accessor.maxValues = ??
	//sample_time_accessor.minValues = ??
	sample_time_accessor.name = "";
	sample_time_accessor.normalized = false;
	sample_time_accessor.type = TINYGLTF_TYPE_SCALAR;
	model.accessors.push_back(std::move(sample_time_accessor));

	tinygltf::Accessor sample_value_accessor;
	sample_value_accessor.bufferView = static_cast<int>(model.bufferViews.size());
	sample_value_accessor.byteOffset = 0;
	sample_value_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
	sample_value_accessor.count = num_samples;
	//sample_value_accessor.maxValues = ??
	//sample_value_accessor.minValues = ??
	sample_value_accessor.name = "";
	sample_value_accessor.normalized = false;
	sample_value_accessor.type = sample_type;
	model.accessors.push_back(std::move(sample_value_accessor));

	tinygltf::BufferView sample_value_buffer_view;
	sample_value_buffer_view.buffer = animation_buffer_index;
	sample_value_buffer_view.byteLength = sample_size * num_samples;
	sample_value_buffer_view.byteOffset = animation_buffer_byte_offset;
	sample_value_buffer_view.byteStride = sample_size;
	sample_value_buffer_view.name = "";
	sample_value_buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;
	model.bufferViews.push_back(std::move(sample_value_buffer_view));

	animation_buffer_byte_offset += sample_size * num_samples;	// Skip samples
}

static void setup_quat_sampler(tinygltf::Model& model, tinygltf::AnimationSampler& sampler, int sample_time_buffer_view_index, int animation_buffer_index, uint32_t num_samples, size_t& animation_buffer_byte_offset)
{
	setup_sampler(model, sampler, sample_time_buffer_view_index, animation_buffer_index, num_samples, animation_buffer_byte_offset, TINYGLTF_TYPE_VEC4, sizeof(rtm::float4f));
}

static void setup_vec3_sampler(tinygltf::Model& model, tinygltf::AnimationSampler& sampler, int sample_time_buffer_view_index, int animation_buffer_index, uint32_t num_samples, size_t& animation_buffer_byte_offset)
{
	setup_sampler(model, sampler, sample_time_buffer_view_index, animation_buffer_index, num_samples, animation_buffer_byte_offset, TINYGLTF_TYPE_VEC3, sizeof(rtm::float3f));
}

static void write_sample_time(tinygltf::Model& model, const tinygltf::AnimationSampler& sampler, uint32_t sample_index, float sample_time)
{
	const tinygltf::Accessor& sample_time_accessor = model.accessors[sampler.input];

	ACL_ASSERT(sample_time_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
	ACL_ASSERT(sample_time_accessor.type == TINYGLTF_TYPE_SCALAR, "Unexpected accessor type");
	ACL_ASSERT(!sample_time_accessor.normalized, "Normalized sample time not supported");

	acl::track_float1f sample_times = make_track_ref<acl::track_float1f>(model, sample_time_accessor);
	sample_times[sample_index] = sample_time;
}

static void RTM_SIMD_CALL write_quat_sample(tinygltf::Model& model, const tinygltf::AnimationSampler& sampler, uint32_t sample_index, rtm::quatf_arg0 sample_value)
{
	const tinygltf::Accessor& sample_value_accessor = model.accessors[sampler.output];

	ACL_ASSERT(sample_value_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
	ACL_ASSERT(sample_value_accessor.type == TINYGLTF_TYPE_VEC4, "Unexpected accessor type");
	ACL_ASSERT(!sample_value_accessor.normalized, "Normalized sample value not supported");

	acl::track_float4f sample_values = make_track_ref<acl::track_float4f>(model, sample_value_accessor);
	rtm::quat_store(sample_value, &sample_values[sample_index]);
}

static void RTM_SIMD_CALL write_vec3_sample(tinygltf::Model& model, const tinygltf::AnimationSampler& sampler, uint32_t sample_index, rtm::vector4f_arg0 sample_value)
{
	const tinygltf::Accessor& sample_value_accessor = model.accessors[sampler.output];

	ACL_ASSERT(sample_value_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
	ACL_ASSERT(sample_value_accessor.type == TINYGLTF_TYPE_VEC3, "Unexpected accessor type");
	ACL_ASSERT(!sample_value_accessor.normalized, "Normalized sample value not supported");

	acl::track_float3f sample_values = make_track_ref<acl::track_float3f>(model, sample_value_accessor);
	rtm::vector_store3(sample_value, &sample_values[sample_index]);
}

bool decompress_gltf(const command_line_options& options)
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
	bool any_decompressed = false;

	acl::ANSIAllocator allocator;

	const std::vector<uint16_t> node_parent_indices = build_node_parent_indices(model);
	const acl::RigidSkeleton skeleton = build_skeleton(model, node_parent_indices, allocator);

	for (tinygltf::Animation& animation : model.animations)
	{
		printf("Processing animation '%s' ...\n", animation.name.empty() ? "<unnamed>" : animation.name.c_str());

		int acl_buffer_view_index;
		if (!is_animation_compressed_with_acl(model, animation, acl_buffer_view_index))
		{
			printf("    No ACL data found, skipping\n");
			continue;
		}

		const int acl_buffer_index = model.bufferViews[acl_buffer_view_index].buffer;

		acl::ClipHeader clip_header;
		float clip_duration;
		{
			const tinygltf::Buffer& acl_buffer = model.buffers[acl_buffer_index];

			const acl::CompressedClip* compressed_clip = reinterpret_cast<const acl::CompressedClip*>(acl_buffer.data.data());
			const acl::ErrorResult is_valid_result = compressed_clip->is_valid(true);
			if (is_valid_result.any())
			{
				printf("    ACL data is invalid: %s\n", is_valid_result.c_str());
				any_failed = true;
				continue;
			}

			// Copy our clip header, we'll add new buffers later and things might resize and move our clip memory
			clip_header = acl::get_clip_header(*compressed_clip);
			if (clip_header.num_bones != skeleton.get_num_bones())
			{
				printf("    Unexpected number of compressed bones\n");
				any_failed = true;
				continue;
			}

			clip_duration = acl::calculate_duration(clip_header.num_samples, clip_header.sample_rate);

			printf("    %.4f seconds (%u samples @ %.2f FPS)\n", clip_duration, clip_header.num_samples, clip_header.sample_rate);
		}

		// Reserve our raw memory buffers and their views
		{
			uint32_t raw_size = 0;
			// Sample times can be shared with every transform track
			raw_size += sizeof(float) * clip_header.num_samples;

			// If we have no scale, strip it
			const uint32_t transform_size = clip_header.has_scale ? (sizeof(float) * 10) : (sizeof(float) * 7);

			// TODO: Handle constant tracks with a single sample
			raw_size += transform_size * clip_header.num_samples * clip_header.num_bones;

			const int animation_buffer_index = static_cast<int>(model.buffers.size());

			tinygltf::Buffer animation_buffer;
			animation_buffer.name = animation.name + "_raw";
			animation_buffer.data.resize(raw_size);
			model.buffers.push_back(std::move(animation_buffer));

			// Buffer view for time samples
			const int sample_time_buffer_view_index = static_cast<int>(model.bufferViews.size());

			tinygltf::BufferView sample_time_buffer_view;
			sample_time_buffer_view.buffer = animation_buffer_index;
			sample_time_buffer_view.byteLength = sizeof(float) * clip_header.num_samples;
			sample_time_buffer_view.byteOffset = 0;
			sample_time_buffer_view.byteStride = sizeof(float);
			sample_time_buffer_view.name = "";
			sample_time_buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;
			model.bufferViews.push_back(std::move(sample_time_buffer_view));

			size_t animation_buffer_byte_offset = 0;
			animation_buffer_byte_offset += sizeof(float) * clip_header.num_samples;	// Skip sample time values

			const uint32_t num_channels_per_transform = clip_header.has_scale ? 3 : 2;
			animation.channels.resize(num_channels_per_transform * clip_header.num_bones);
			animation.samplers.resize(num_channels_per_transform * clip_header.num_bones);

			for (uint16_t transform_index = 0; transform_index < clip_header.num_bones; ++transform_index)
			{
				setup_channel(animation, "rotation", transform_index, (transform_index * num_channels_per_transform) + 0);
				setup_channel(animation, "translation", transform_index, (transform_index * num_channels_per_transform) + 1);

				if (clip_header.has_scale)
					setup_channel(animation, "scale", transform_index, (transform_index * num_channels_per_transform) + 2);

				tinygltf::AnimationSampler& rotation_sampler = animation.samplers[(transform_index * num_channels_per_transform) + 0];
				setup_quat_sampler(model, rotation_sampler, sample_time_buffer_view_index, animation_buffer_index, clip_header.num_samples, animation_buffer_byte_offset);

				tinygltf::AnimationSampler& translation_sampler = animation.samplers[(transform_index * num_channels_per_transform) + 1];
				setup_vec3_sampler(model, translation_sampler, sample_time_buffer_view_index, animation_buffer_index, clip_header.num_samples, animation_buffer_byte_offset);

				if (clip_header.has_scale)
				{
					tinygltf::AnimationSampler& scale_sampler = animation.samplers[(transform_index * num_channels_per_transform) + 2];
					setup_vec3_sampler(model, scale_sampler, sample_time_buffer_view_index, animation_buffer_index, clip_header.num_samples, animation_buffer_byte_offset);
				}
			}
		}

		// Decompress our animation
		{
			const tinygltf::Buffer& acl_buffer = model.buffers[acl_buffer_index];
			const acl::CompressedClip* compressed_clip = reinterpret_cast<const acl::CompressedClip*>(acl_buffer.data.data());

			acl::uniformly_sampled::DecompressionContext<acl::uniformly_sampled::DebugDecompressionSettings> context;
			context.initialize(*compressed_clip);

			std::vector<rtm::qvvf> lossy_local_transforms;
			lossy_local_transforms.resize(clip_header.num_bones);
			acl::DefaultOutputWriter pose_writer(lossy_local_transforms.data(), clip_header.num_bones);

			const uint32_t num_channels_per_transform = clip_header.has_scale ? 3 : 2;
			tinygltf::AnimationSampler& time_sampler = animation.samplers[0];	// Use the first sample to write the time values, they are shared with every channel

			for (uint32_t sample_index = 0; sample_index < clip_header.num_samples; ++sample_index)
			{
				const float sample_time = rtm::scalar_min(float(sample_index) / clip_header.sample_rate, clip_duration);

				context.seek(sample_time, acl::sample_rounding_policy::none);
				context.decompress_pose(pose_writer);

				write_sample_time(model, time_sampler, sample_index, sample_time);

				for (uint16_t transform_index = 0; transform_index < clip_header.num_bones; ++transform_index)
				{
					tinygltf::AnimationSampler& rotation_sampler = animation.samplers[(transform_index * num_channels_per_transform) + 0];
					write_quat_sample(model, rotation_sampler, sample_index, lossy_local_transforms[transform_index].rotation);

					tinygltf::AnimationSampler& translation_sampler = animation.samplers[(transform_index * num_channels_per_transform) + 1];
					write_vec3_sample(model, translation_sampler, sample_index, lossy_local_transforms[transform_index].translation);

					if (clip_header.has_scale)
					{
						tinygltf::AnimationSampler& scale_sampler = animation.samplers[(transform_index * num_channels_per_transform) + 2];
						write_vec3_sample(model, scale_sampler, sample_index, lossy_local_transforms[transform_index].scale);
					}
				}
			}

			const uint32_t raw_animation_size = get_raw_animation_size(model, animation);
			const uint32_t compressed_animation_size = compressed_clip->get_size();
			const float compression_ratio = float(raw_animation_size) / float(compressed_animation_size);

			printf("    Decompressed %u ACL bytes into %u glTF bytes (ratio: %.2f : 1)\n", compressed_animation_size, raw_animation_size, compression_ratio);
		}

		// Measure the compression error
		{
			const tinygltf::Buffer& acl_buffer = model.buffers[acl_buffer_index];
			const acl::CompressedClip* compressed_clip = reinterpret_cast<const acl::CompressedClip*>(acl_buffer.data.data());

			const acl::AnimationClip clip = build_clip(model, animation, skeleton, allocator);

			acl::uniformly_sampled::DecompressionContext<acl::uniformly_sampled::DebugDecompressionSettings> context;
			context.initialize(*compressed_clip);

			acl::qvvf_transform_error_metric error_metric;
			const acl::BoneError bone_error = calculate_compressed_clip_error(allocator, clip, error_metric, context);

			const char* worst_bone_name = bone_error.index == acl::k_invalid_bone_index ? "<Unknown>" : skeleton.get_bone(bone_error.index).name.c_str();
			printf("    Largest error of %.2f mm on transform %u ('%s') @ %.2f sec\n", bone_error.error * 100.0F * 100.0F, bone_error.index, worst_bone_name, bone_error.sample_time);
		}

		// Reset our buffer view
		reset_buffer_view(model.bufferViews[acl_buffer_view_index]);
		reset_buffer(model.buffers[acl_buffer_index]);

		any_decompressed = true;
	}

	if (any_failed)
		return false;

	if (any_decompressed)
	{
		model.extensionsRequired.erase(std::find(model.extensionsRequired.begin(), model.extensionsRequired.end(), k_acl_gltf_extension_str));
		model.extensionsUsed.erase(std::find(model.extensionsUsed.begin(), model.extensionsUsed.end(), k_acl_gltf_extension_str));

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

	return true;
}
