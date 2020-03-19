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
#include <acl/compression/track_error.h>
#include <acl/compression/utils.h>
#include <acl/core/impl/debug_track_writer.h>
#include <acl/core/ansi_allocator.h>
#include <acl/core/utils.h>
#include <acl/decompression/decompress.h>
#include <acl/decompression/default_output_writer.h>

#include <rtm/scalarf.h>
#include <rtm/vector4f.h>

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
	//sample_time_accessor.maxValues = ... this is set later
	//sample_time_accessor.minValues = ... this is set later
	sample_time_accessor.name = "";
	sample_time_accessor.normalized = false;
	sample_time_accessor.type = TINYGLTF_TYPE_SCALAR;
	model.accessors.push_back(std::move(sample_time_accessor));

	tinygltf::Accessor sample_value_accessor;
	sample_value_accessor.bufferView = static_cast<int>(model.bufferViews.size());
	sample_value_accessor.byteOffset = 0;
	sample_value_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
	sample_value_accessor.count = num_samples;
	//sample_value_accessor.maxValues = ... this is set later
	//sample_value_accessor.minValues = ... this is set later
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
	sample_value_buffer_view.target = 0;	// Optional
	model.bufferViews.push_back(std::move(sample_value_buffer_view));

	animation_buffer_byte_offset += sample_size * num_samples;	// Skip samples
}

static void setup_weights_sampler(tinygltf::Model& model, tinygltf::AnimationSampler& sampler, int sample_time_buffer_view_index, int animation_buffer_index, uint32_t num_samples, uint32_t num_weight_tracks)
{
	sampler.input = static_cast<int>(model.accessors.size());
	sampler.output = sampler.input + 1;
	sampler.interpolation = "LINEAR";

	tinygltf::Accessor sample_time_accessor;
	sample_time_accessor.bufferView = sample_time_buffer_view_index;
	sample_time_accessor.byteOffset = 0;
	sample_time_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
	sample_time_accessor.count = num_samples;
	//sample_time_accessor.maxValues = ... this is set later
	//sample_time_accessor.minValues = ... this is set later
	sample_time_accessor.name = "";
	sample_time_accessor.normalized = false;
	sample_time_accessor.type = TINYGLTF_TYPE_SCALAR;
	model.accessors.push_back(std::move(sample_time_accessor));

	tinygltf::Accessor sample_value_accessor;
	sample_value_accessor.bufferView = static_cast<int>(model.bufferViews.size());
	sample_value_accessor.byteOffset = 0;
	sample_value_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
	sample_value_accessor.count = num_samples * num_weight_tracks;
	//sample_value_accessor.maxValues = ... this is set later
	//sample_value_accessor.minValues = ... this is set later
	sample_value_accessor.name = "";
	sample_value_accessor.normalized = false;
	sample_value_accessor.type = TINYGLTF_TYPE_SCALAR;
	model.accessors.push_back(std::move(sample_value_accessor));

	tinygltf::BufferView sample_value_buffer_view;
	sample_value_buffer_view.buffer = animation_buffer_index;
	sample_value_buffer_view.byteLength = sizeof(float) * num_samples * num_weight_tracks;
	sample_value_buffer_view.byteOffset = 0;
	sample_value_buffer_view.byteStride = sizeof(float);
	sample_value_buffer_view.name = "";
	sample_value_buffer_view.target = 0;	// Optional
	model.bufferViews.push_back(std::move(sample_value_buffer_view));
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
	const tinygltf::Accessor& sample_accessor = model.accessors[sampler.input];

	ACL_ASSERT(sample_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
	ACL_ASSERT(sample_accessor.type == TINYGLTF_TYPE_SCALAR, "Unexpected accessor type");
	ACL_ASSERT(!sample_accessor.normalized, "Normalized sample value not supported");

	acl::track_float1f samples = make_track_ref<acl::track_float1f>(model, sample_accessor);
	samples[sample_index] = sample_time;
}

static void write_scalar_sample(tinygltf::Model& model, const tinygltf::AnimationSampler& sampler, uint32_t sample_index, float sample)
{
	const tinygltf::Accessor& sample_accessor = model.accessors[sampler.output];

	ACL_ASSERT(sample_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
	ACL_ASSERT(sample_accessor.type == TINYGLTF_TYPE_SCALAR, "Unexpected accessor type");
	ACL_ASSERT(!sample_accessor.normalized, "Normalized sample value not supported");

	acl::track_float1f samples = make_track_ref<acl::track_float1f>(model, sample_accessor);
	samples[sample_index] = sample;
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

static void update_accessor_min_max_vec4(tinygltf::Model& model, tinygltf::Accessor& accessor)
{
	ACL_ASSERT(accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
	ACL_ASSERT(accessor.type == TINYGLTF_TYPE_VEC4, "Unexpected accessor type");
	ACL_ASSERT(!accessor.normalized, "Normalized sample value not supported");

	const acl::track_float4f sample_values = make_track_ref<acl::track_float4f>(model, accessor);

	rtm::vector4f min = rtm::vector_set(1.0E38F);
	rtm::vector4f max = rtm::vector_set(-1.0E38F);

	const uint32_t num_samples = sample_values.get_num_samples();
	for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
	{
		const rtm::vector4f sample_ = rtm::vector_load(&sample_values[sample_index]);
		min = rtm::vector_min(min, sample_);
		max = rtm::vector_max(max, sample_);
	}

	accessor.minValues.resize(4);
	accessor.minValues[0] = rtm::vector_get_x(min);
	accessor.minValues[1] = rtm::vector_get_y(min);
	accessor.minValues[2] = rtm::vector_get_z(min);
	accessor.minValues[3] = rtm::vector_get_w(min);

	accessor.maxValues.resize(4);
	accessor.maxValues[0] = rtm::vector_get_x(max);
	accessor.maxValues[1] = rtm::vector_get_y(max);
	accessor.maxValues[2] = rtm::vector_get_z(max);
	accessor.maxValues[3] = rtm::vector_get_w(max);
}

static void update_accessor_min_max_vec3(tinygltf::Model& model, tinygltf::Accessor& accessor)
{
	ACL_ASSERT(accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
	ACL_ASSERT(accessor.type == TINYGLTF_TYPE_VEC3, "Unexpected accessor type");
	ACL_ASSERT(!accessor.normalized, "Normalized sample value not supported");

	const acl::track_float3f sample_values = make_track_ref<acl::track_float3f>(model, accessor);

	rtm::vector4f min = rtm::vector_set(1.0E38F);
	rtm::vector4f max = rtm::vector_set(-1.0E38F);

	const uint32_t num_samples = sample_values.get_num_samples();
	for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
	{
		const rtm::vector4f sample_ = rtm::vector_load3(&sample_values[sample_index]);
		min = rtm::vector_min(min, sample_);
		max = rtm::vector_max(max, sample_);
	}

	accessor.minValues.resize(3);
	accessor.minValues[0] = rtm::vector_get_x(min);
	accessor.minValues[1] = rtm::vector_get_y(min);
	accessor.minValues[2] = rtm::vector_get_z(min);

	accessor.maxValues.resize(3);
	accessor.maxValues[0] = rtm::vector_get_x(max);
	accessor.maxValues[1] = rtm::vector_get_y(max);
	accessor.maxValues[2] = rtm::vector_get_z(max);
}

static void update_accessor_min_max_scalar(tinygltf::Model& model, tinygltf::Accessor& accessor)
{
	ACL_ASSERT(accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
	ACL_ASSERT(accessor.type == TINYGLTF_TYPE_SCALAR, "Unexpected accessor type");
	ACL_ASSERT(!accessor.normalized, "Normalized sample value not supported");

	const acl::track_float1f sample_values = make_track_ref<acl::track_float1f>(model, accessor);

	rtm::scalarf min = rtm::scalar_set(1.0E38F);
	rtm::scalarf max = rtm::scalar_set(-1.0E38F);

	const uint32_t num_samples = sample_values.get_num_samples();
	for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
	{
		const rtm::scalarf sample_ = rtm::scalar_load(&sample_values[sample_index]);
		min = rtm::scalar_min(min, sample_);
		max = rtm::scalar_max(max, sample_);
	}

	accessor.minValues.resize(1);
	accessor.minValues[0] = rtm::scalar_cast(min);

	accessor.maxValues.resize(1);
	accessor.maxValues[0] = rtm::scalar_cast(max);
}

static bool decompress_transform_nodes(tinygltf::Model& model, tinygltf::Animation& animation, const acl::RigidSkeleton& skeleton, const hierarchy_description& hierarchy, int acl_buffer_view_index, int sample_time_buffer_view_index)
{
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
			return false;
		}

		// Copy our clip header, we'll add new buffers later and things might resize and move our clip memory
		clip_header = acl::get_clip_header(*compressed_clip);
		if (clip_header.num_bones != skeleton.get_num_bones())
		{
			printf("    Unexpected number of compressed bones\n");
			return false;
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

		tinygltf::BufferView sample_time_buffer_view;
		sample_time_buffer_view.buffer = animation_buffer_index;
		sample_time_buffer_view.byteLength = sizeof(float) * clip_header.num_samples;
		sample_time_buffer_view.byteOffset = 0;
		sample_time_buffer_view.byteStride = sizeof(float);
		sample_time_buffer_view.name = "";
		sample_time_buffer_view.target = 0;	// Optional
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
		tinygltf::AnimationSampler& time_sampler = animation.samplers[0];	// Use the first sampler to write the time values, they are shared with every channel

		for (uint32_t sample_index = 0; sample_index < clip_header.num_samples; ++sample_index)
		{
			const float sample_time = rtm::scalar_min(float(sample_index) / clip_header.sample_rate, clip_duration);

			// TODO: Use slerp
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

		// Now that all the data has been written, update the min/max for each accessor
		for (uint16_t transform_index = 0; transform_index < clip_header.num_bones; ++transform_index)
		{
			{
				const tinygltf::AnimationSampler& rotation_sampler = animation.samplers[(transform_index * num_channels_per_transform) + 0];

				tinygltf::Accessor& time_accessor = model.accessors[rotation_sampler.input];
				update_accessor_min_max_scalar(model, time_accessor);
				tinygltf::Accessor& rotation_accessor = model.accessors[rotation_sampler.output];
				update_accessor_min_max_vec4(model, rotation_accessor);
			}

			{
				const tinygltf::AnimationSampler& translation_sampler = animation.samplers[(transform_index * num_channels_per_transform) + 1];

				tinygltf::Accessor& time_accessor = model.accessors[translation_sampler.input];
				update_accessor_min_max_scalar(model, time_accessor);
				tinygltf::Accessor& translation_accessor = model.accessors[translation_sampler.output];
				update_accessor_min_max_vec3(model, translation_accessor);
			}

			if (clip_header.has_scale)
			{
				const tinygltf::AnimationSampler& scale_sampler = animation.samplers[(transform_index * num_channels_per_transform) + 2];

				tinygltf::Accessor& time_accessor = model.accessors[scale_sampler.input];
				update_accessor_min_max_scalar(model, time_accessor);
				tinygltf::Accessor& scale_accessor = model.accessors[scale_sampler.output];
				update_accessor_min_max_vec3(model, scale_accessor);
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

		acl::ANSIAllocator allocator;
		const acl::AnimationClip clip = build_clip(model, animation, hierarchy, skeleton, allocator);

		acl::uniformly_sampled::DecompressionContext<acl::uniformly_sampled::DebugDecompressionSettings> context;
		context.initialize(*compressed_clip);

		acl::qvvf_transform_error_metric error_metric;
		const acl::BoneError bone_error = acl::calculate_error_between_clips(allocator, error_metric, clip, context);

		const char* worst_bone_name = bone_error.index == acl::k_invalid_bone_index ? "<Unknown>" : skeleton.get_bone(bone_error.index).name.c_str();
		printf("    Largest error of %.2f mm on transform %u ('%s') @ %.2f sec\n", bone_error.error * 1000.0F, bone_error.index, worst_bone_name, bone_error.sample_time);
	}

	// Reset everything we no longer need, safe since we know ACL uses distinct buffers and views
	reset_buffer_view(model.bufferViews[acl_buffer_view_index]);
	reset_buffer(model.buffers[acl_buffer_index]);

	// Success
	return true;
}

static bool decompress_scalar_weights(tinygltf::Model& model, tinygltf::Animation& animation, const tinygltf::Animation& source_animation, int sample_time_buffer_view_index)
{
	// Note that transforms and weights share the same sample times buffer view

	acl::ANSIAllocator allocator;

	for (const tinygltf::AnimationChannel& source_channel : source_animation.channels)
	{
		int acl_weights_buffer_view_index;
		if (!has_compressed_weights_with_acl(model, source_animation, source_channel, acl_weights_buffer_view_index))
			continue;	// No weights in this channel, skip it

		const int acl_weights_buffer_index = model.bufferViews[acl_weights_buffer_view_index].buffer;

		acl::compressed_tracks* weight_tracks;
		{
			const tinygltf::Buffer& acl_buffer = model.buffers[acl_weights_buffer_index];

			const acl::compressed_tracks* tracks = reinterpret_cast<const acl::compressed_tracks*>(acl_buffer.data.data());
			const acl::ErrorResult is_valid_result = tracks->is_valid(true);
			if (is_valid_result.any())
			{
				printf("    ACL weight data is invalid: %s\n", is_valid_result.c_str());
				return false;
			}

			if (tracks->get_track_type() != acl::track_type8::float1f)
			{
				printf("    Unexpected track type for compressed weights\n");
				return false;
			}

			const int node_index = source_channel.target_node;
			const std::vector<double>& default_weights = find_default_weights(model, model.nodes[node_index]);
			const uint32_t num_weight_tracks = static_cast<uint32_t>(default_weights.size());
			if (num_weight_tracks != tracks->get_num_tracks())
			{
				printf("    Unexpected number of compressed weights\n");
				return false;
			}

			// Same duration/num samples/sample rate as transforms
			//printf("    Weights: %.4f seconds (%u samples @ %.2f FPS)\n", tracks->get_duration(), tracks->get_num_samples_per_track(), tracks->get_sample_rate());

			// Copy our compressed data since we'll add buffers and the data might be invalidated
			void* weight_tracks_ptr = allocator.allocate(tracks->get_size(), alignof(acl::compressed_tracks));
			weight_tracks = static_cast<acl::compressed_tracks*>(weight_tracks_ptr);
			std::memcpy(weight_tracks, tracks, tracks->get_size());
		}

		const uint32_t num_tracks = weight_tracks->get_num_tracks();
		const uint32_t num_samples_per_track = weight_tracks->get_num_samples_per_track();

		// Reserve our raw memory buffers and their views
		{
			uint32_t raw_size = 0;

			// Sample values
			// TODO: Handle constant tracks with a single sample
			raw_size += sizeof(float) * num_samples_per_track * num_tracks;

			const int weights_buffer_index = static_cast<int>(model.buffers.size());

			tinygltf::Buffer weights_buffer;
			weights_buffer.name = animation.name + "_weights_raw";
			weights_buffer.data.resize(raw_size);
			model.buffers.push_back(std::move(weights_buffer));

			animation.channels.resize(animation.channels.size() + 1);
			animation.samplers.resize(animation.samplers.size() + 1);

			const int node_index = source_channel.target_node;
			setup_channel(animation, "weights", node_index, static_cast<int>(animation.channels.size() - 1));

			tinygltf::AnimationSampler& weights_sampler = animation.samplers.back();
			setup_weights_sampler(model, weights_sampler, sample_time_buffer_view_index, weights_buffer_index, num_samples_per_track, num_tracks);
		}

		// Decompress our weights
		{
			acl::decompression_context<acl::debug_decompression_settings> context;
			context.initialize(*weight_tracks);

			const float duration = weight_tracks->get_duration();
			const float sample_rate = weight_tracks->get_sample_rate();

			acl::acl_impl::debug_track_writer tracks_writer(allocator, acl::track_type8::float1f, num_tracks);
			tinygltf::AnimationSampler& weights_sampler = animation.samplers.back();

			for (uint32_t sample_index = 0; sample_index < num_samples_per_track; ++sample_index)
			{
				const float sample_time = rtm::scalar_min(float(sample_index) / sample_rate, duration);

				context.seek(sample_time, acl::sample_rounding_policy::none);
				context.decompress_tracks(tracks_writer);

				for (uint32_t track_index = 0; track_index < num_tracks; ++track_index)
					write_scalar_sample(model, weights_sampler, (track_index * num_samples_per_track) + sample_index, tracks_writer.read_float1(track_index));
			}

			// Now that all the data has been written, update the min/max for our accessor
			{
				tinygltf::Accessor& weights_accessor = model.accessors[weights_sampler.output];
				update_accessor_min_max_scalar(model, weights_accessor);
			}

			const uint32_t raw_weights_size = num_tracks * num_samples_per_track * sizeof(float);
			const uint32_t compressed_weights_size = weight_tracks->get_size();
			const float compression_ratio = float(raw_weights_size) / float(compressed_weights_size);

			printf("    Decompressed weights %u ACL bytes into %u glTF bytes (ratio: %.2f : 1)\n", compressed_weights_size, raw_weights_size, compression_ratio);
		}

		// Measure the compression error
		{
			const tinygltf::AnimationChannel& channel = animation.channels.back();
			const acl::track_array_float1f raw_weight_tracks = build_weight_tracks(model, animation, channel, allocator);

			acl::decompression_context<acl::debug_decompression_settings> context;
			context.initialize(*weight_tracks);

			const acl::track_error track_err = acl::calculate_compression_error(allocator, raw_weight_tracks, context);

			printf("    Largest error of %.4f on weight track %u @ %.2f sec\n", track_err.error, track_err.index, track_err.sample_time);
		}

		// Reset everything we no longer need, safe since we know ACL uses distinct buffers and views
		reset_buffer_view(model.bufferViews[acl_weights_buffer_view_index]);
		reset_buffer(model.buffers[acl_weights_buffer_index]);

		// Free our compressed tracks, no longer needed
		allocator.deallocate(weight_tracks, weight_tracks->get_size());
	}

	// Success
	return true;
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
		printf("glTF input does not contain any animations, nothing to decompress\n");
		return true;
	}

	bool any_failed = false;
	bool any_decompressed = false;

	acl::ANSIAllocator allocator;

	const hierarchy_description hierarchy = build_hierarchy(model);
	const acl::RigidSkeleton skeleton = build_skeleton(model, hierarchy, allocator);

	for (tinygltf::Animation& animation : model.animations)
	{
		printf("Processing animation '%s' ...\n", animation.name.empty() ? "<unnamed>" : animation.name.c_str());

		int acl_buffer_view_index;
		if (!is_animation_compressed_with_acl(model, animation, acl_buffer_view_index))
		{
			printf("    No ACL data found, skipping\n");
			continue;
		}

		// Copy since we'll rewrite it
		const tinygltf::Animation source_animation = animation;

		// Buffer view for time samples
		const int sample_time_buffer_view_index = static_cast<int>(model.bufferViews.size());

		// Handle node transforms
		const bool decompressed_transforms = decompress_transform_nodes(model, animation, skeleton, hierarchy, acl_buffer_view_index, sample_time_buffer_view_index);
		if (!decompressed_transforms)
		{
			any_failed = true;
			continue;
		}

		// Handle scalar weights
		const bool decompressed_weights = decompress_scalar_weights(model, animation, source_animation, sample_time_buffer_view_index);
		if (!decompressed_weights)
		{
			any_failed = true;
			continue;
		}

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

	return true;
}
