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

	return true;
}

static std::vector<uint16_t> build_node_parent_indices(const tinygltf::Model& model)
{
	const size_t num_transforms = model.nodes.size();

	std::vector<uint16_t> node_parent_indices;
	node_parent_indices.resize(num_transforms, acl::k_invalid_bone_index);

	for (size_t transform_index = 0; transform_index < num_transforms; ++transform_index)
	{
		const tinygltf::Node& node = model.nodes[transform_index];

		for (int child_index : node.children)
			node_parent_indices[child_index] = static_cast<uint16_t>(transform_index);
	}

	return node_parent_indices;
}

static acl::RigidSkeleton build_skeleton(const tinygltf::Model& model, const std::vector<uint16_t>& node_parent_indices, acl::IAllocator& allocator)
{
	const uint16_t num_transforms = static_cast<uint16_t>(model.nodes.size());

	std::vector<acl::RigidBone> bones;
	bones.resize(num_transforms);

	for (uint16_t transform_index = 0; transform_index < num_transforms; ++transform_index)
	{
		const tinygltf::Node& node = model.nodes[transform_index];
		acl::RigidBone& bone = bones[transform_index];

		bone.name = acl::String(allocator, node.name.c_str());
		bone.parent_index = node_parent_indices[transform_index];

		// glTF units are in meters
		bone.vertex_distance = 1.0F;		// TODO: Use skinning information and mesh?

		if (node.matrix.empty())
		{
			if (!node.rotation.empty())
				bone.bind_transform.rotation = rtm::quat_load(node.rotation.data());

			if (!node.translation.empty())
				bone.bind_transform.translation = rtm::vector_load3(node.translation.data());

			if (!node.scale.empty())
				bone.bind_transform.scale = rtm::vector_load3(node.scale.data());
		}
		else
		{
			const rtm::vector4d x_axis = rtm::vector_load(node.matrix.data() + 0);
			const rtm::vector4d y_axis = rtm::vector_load(node.matrix.data() + 4);
			const rtm::vector4d z_axis = rtm::vector_load(node.matrix.data() + 8);
			const rtm::vector4d w_axis = rtm::vector_load(node.matrix.data() + 12);

			const rtm::matrix3x4d transform = rtm::matrix_set(x_axis, y_axis, z_axis, w_axis);
			bone.bind_transform.rotation = rtm::quat_from_matrix(transform);
			bone.bind_transform.translation = w_axis;
			bone.bind_transform.scale = rtm::vector_set(rtm::vector_length3(x_axis), rtm::vector_length3(y_axis), rtm::vector_length3(z_axis), 0.0);
		}
	}

	return acl::RigidSkeleton(allocator, bones.data(), num_transforms);
}

template<typename track_type>
static track_type make_track_ref(const tinygltf::Model& model, const tinygltf::Accessor& sample_time_accessor)
{
	using sample_type = typename track_type::sample_type;

	const tinygltf::BufferView& buffer_view = model.bufferViews[sample_time_accessor.bufferView];
	const tinygltf::Buffer& buffer = model.buffers[buffer_view.buffer];

	const size_t start_offset = buffer_view.byteOffset + sample_time_accessor.byteOffset;
	const int sample_stride = sample_time_accessor.ByteStride(buffer_view);
	ACL_ASSERT(sample_stride >= static_cast<int>(sizeof(sample_type)), "Invalid sample stride");

	const acl::track_desc_scalarf dummy_desc = { 0, 0.0F, 0.0F };

	return  track_type::make_ref(dummy_desc, reinterpret_cast<const sample_type*>(buffer.data.data() + start_offset), static_cast<uint32_t>(sample_time_accessor.count), 0.0F, sample_stride);
}

static float find_clip_duration(const tinygltf::Model& model, const tinygltf::Animation& animation)
{
	float max_duration = 0.0F;
	size_t max_num_samples = 0;

	for (const tinygltf::AnimationChannel& channel : animation.channels)
	{
		const tinygltf::AnimationSampler& sampler = animation.samplers[channel.sampler];
		const tinygltf::Accessor& sample_time_accessor = model.accessors[sampler.input];

		ACL_ASSERT(sample_time_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
		ACL_ASSERT(sample_time_accessor.type == TINYGLTF_TYPE_SCALAR, "Unexpected accessor type");
		ACL_ASSERT(!sample_time_accessor.normalized, "Normalized sample time not supported");

		const acl::track_float1f sample_times = make_track_ref<acl::track_float1f>(model, sample_time_accessor);
		const float last_sample_time = sample_times[sample_times.get_num_samples() - 1];

		max_duration = rtm::scalar_max(max_duration, last_sample_time);
		max_num_samples = std::max<size_t>(max_num_samples, sample_time_accessor.count);
	}

	ACL_ASSERT(rtm::scalar_is_finite(max_duration), "Invalid data");

	// See acl::calculate_duration(..) for details
	if (max_num_samples == 0)
		return 0.0F;	// No samples means we have no duration

	if (max_num_samples == 1)
		return std::numeric_limits<float>::infinity();	// A single sample means we have an indefinite duration (static pose)

	// Use the value of our last sample time
	return max_duration;
}

static float find_clip_sample_rate(const tinygltf::Model& model, const tinygltf::Animation& animation, float duration)
{
	// If our clip has no samples, or a single sample, or samples which aren't spaced linearly,
	// we will try and use a sample rate as close to 30 FPS as possible.
	const float target_sample_rate = 30.0F;
	if (duration == 0.0F || duration == std::numeric_limits<float>::infinity())
		return target_sample_rate;

	// First find the smallest amount of time between two samples, this is potentially
	// our sample rate.
	float smallest_delta_time = 1.0E10F;

	for (const tinygltf::AnimationChannel& channel : animation.channels)
	{
		const tinygltf::AnimationSampler& sampler = animation.samplers[channel.sampler];
		const tinygltf::Accessor& sample_time_accessor = model.accessors[sampler.input];

		ACL_ASSERT(sample_time_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
		ACL_ASSERT(sample_time_accessor.type == TINYGLTF_TYPE_SCALAR, "Unexpected accessor type");
		ACL_ASSERT(!sample_time_accessor.normalized, "Normalized sample time not supported");

		const acl::track_float1f sample_times = make_track_ref<acl::track_float1f>(model, sample_time_accessor);

		const uint32_t num_samples = sample_times.get_num_samples();
		for (uint32_t sample_index = 1; sample_index < num_samples; ++sample_index)
		{
			const float prev_sample_time = sample_times[sample_index - 1];
			const float curr_sample_time = sample_times[sample_index];
			const float delta_time = curr_sample_time - prev_sample_time;
			smallest_delta_time = rtm::scalar_min(smallest_delta_time, delta_time);
		}
	}

	ACL_ASSERT(rtm::scalar_is_finite(smallest_delta_time), "Invalid data");
	ACL_ASSERT(smallest_delta_time > 0.0F, "Invalid data");

	// If every sample is near a multiple of our sample rate, it is valid
	bool is_sample_rate_valid = true;

	for (const tinygltf::AnimationChannel& channel : animation.channels)
	{
		const tinygltf::AnimationSampler& sampler = animation.samplers[channel.sampler];
		const tinygltf::Accessor& sample_time_accessor = model.accessors[sampler.input];

		ACL_ASSERT(sample_time_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
		ACL_ASSERT(sample_time_accessor.type == TINYGLTF_TYPE_SCALAR, "Unexpected accessor type");
		ACL_ASSERT(!sample_time_accessor.normalized, "Normalized sample time not supported");

		const acl::track_float1f sample_times = make_track_ref<acl::track_float1f>(model, sample_time_accessor);

		const uint32_t num_samples = sample_times.get_num_samples();
		for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
		{
			const float sample_time = sample_times[sample_index];
			const float remainder = std::fmod(sample_time, smallest_delta_time);
			if (remainder > 0.001F)
			{
				is_sample_rate_valid = false;
				break;
			}
		}
	}

	if (is_sample_rate_valid)
	{
		const float sample_rate = 1.0F / smallest_delta_time;
		return sample_rate;
	}

	// Our samples aren't linearly spaced, do our best to approximate something decent
	const uint32_t num_samples = acl::calculate_num_samples(duration, target_sample_rate);
	return static_cast<float>(num_samples - 1) / duration;
}

static void find_linear_interpolation_values(const acl::track_float1f& sample_times, float sample_time, uint32_t& out_sample_index0, uint32_t& out_sample_index1, float& out_interpolation_alpha)
{
	const uint32_t num_gltf_sample_times = sample_times.get_num_samples();

	// Linear scan to find the samples we need
	uint32_t gltf_sample0 = num_gltf_sample_times;
	uint32_t gltf_sample1 = num_gltf_sample_times;
	for (uint32_t gltf_sample_index = 0; gltf_sample_index < num_gltf_sample_times; ++gltf_sample_index)
	{
		const float gltf_sample_time = sample_times[gltf_sample_index];
		if (gltf_sample_time >= sample_time)
		{
			// We went too far, use the last two samples
			if (gltf_sample_index == 0)
			{
				// Clamp to first sample
				gltf_sample0 = 0;
				gltf_sample1 = 0;
			}
			else
			{
				gltf_sample0 = gltf_sample_index - 1;
				gltf_sample1 = gltf_sample_index;
			}

			// Done
			break;
		}
	}

	if (gltf_sample0 == num_gltf_sample_times)
	{
		// Clamp to last sample
		gltf_sample0 = num_gltf_sample_times - 1;
		gltf_sample1 = num_gltf_sample_times - 1;
	}

	out_sample_index0 = gltf_sample0;
	out_sample_index1 = gltf_sample1;

	const float gltf_sample_time0 = sample_times[gltf_sample0];
	const float gltf_sample_time1 = sample_times[gltf_sample1];
	out_interpolation_alpha = rtm::scalar_clamp((sample_time - gltf_sample_time0) / (gltf_sample_time1 - gltf_sample_time0), 0.0F, 1.0F);
}

static acl::AnimationClip build_clip(const tinygltf::Model& model, const tinygltf::Animation& animation, const acl::RigidSkeleton& skeleton, acl::IAllocator& allocator)
{
	const float duration = find_clip_duration(model, animation);
	const float sample_rate = find_clip_sample_rate(model, animation, duration);
	const uint32_t num_samples = acl::calculate_num_samples(duration, sample_rate);

	acl::AnimationClip clip(allocator, skeleton, num_samples, sample_rate, acl::String(allocator, animation.name.c_str()));

	// Populate everything with the bind pose
	const uint16_t num_bones = skeleton.get_num_bones();
	for (uint16_t bone_index = 0; bone_index < num_bones; ++bone_index)
	{
		const rtm::qvvd& bind_transform = skeleton.get_bone(bone_index).bind_transform;
		acl::AnimatedBone& bone = clip.get_animated_bone(bone_index);

		for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
		{
			bone.rotation_track.set_sample(sample_index, bind_transform.rotation);
			bone.translation_track.set_sample(sample_index, bind_transform.translation);
			bone.scale_track.set_sample(sample_index, bind_transform.scale);
		}
	}

	// Sample the glTF animation and populate our data
	for (const tinygltf::AnimationChannel& channel : animation.channels)
	{
		const uint16_t bone_index = static_cast<uint16_t>(channel.target_node);
		acl::AnimatedBone& bone = clip.get_animated_bone(bone_index);

		const tinygltf::AnimationSampler& sampler = animation.samplers[channel.sampler];
		const tinygltf::Accessor& sample_time_accessor = model.accessors[sampler.input];
		const tinygltf::Accessor& sample_value_accessor = model.accessors[sampler.output];

		ACL_ASSERT(sample_time_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
		ACL_ASSERT(sample_time_accessor.type == TINYGLTF_TYPE_SCALAR, "Unexpected accessor type");
		ACL_ASSERT(!sample_time_accessor.normalized, "Normalized sample time not supported");

		ACL_ASSERT(sample_value_accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT, "Unexpected accessor component type");
		ACL_ASSERT(!sample_value_accessor.normalized, "Normalized sample time not supported");

		const acl::track_float1f sample_times = make_track_ref<acl::track_float1f>(model, sample_time_accessor);

		if (channel.target_path == "rotation")
		{
			ACL_ASSERT(sample_value_accessor.type == TINYGLTF_TYPE_VEC4, "Unexpected accessor type");

			const acl::track_float4f sample_values = make_track_ref<acl::track_float4f>(model, sample_value_accessor);

			if (sampler.interpolation == "LINEAR")
			{
				for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
				{
					const float sample_time = rtm::scalar_min(float(sample_index) / sample_rate, duration);

					uint32_t gltf_sample0 = 0;
					uint32_t gltf_sample1 = 0;
					float interpolation_alpha = 0.0F;
					find_linear_interpolation_values(sample_times, sample_time, gltf_sample0, gltf_sample1, interpolation_alpha);

					const rtm::quatf sample0 = rtm::vector_to_quat(rtm::vector_load(&sample_values[gltf_sample0]));
					const rtm::quatf sample1 = rtm::vector_to_quat(rtm::vector_load(&sample_values[gltf_sample1]));
					const rtm::quatf sample = rtm::quat_lerp(sample0, sample1, interpolation_alpha);

					bone.rotation_track.set_sample(sample_index, rtm::quat_cast(sample));
				}
			}
			else
			{
				ACL_ASSERT(false, "Interpolation mode not supported");
			}
		}
		else if (channel.target_path == "translation")
		{
			ACL_ASSERT(sample_value_accessor.type == TINYGLTF_TYPE_VEC3, "Unexpected accessor type");

			const acl::track_float3f sample_values = make_track_ref<acl::track_float3f>(model, sample_value_accessor);

			if (sampler.interpolation == "LINEAR")
			{
				for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
				{
					const float sample_time = rtm::scalar_min(float(sample_index) / sample_rate, duration);

					uint32_t gltf_sample0 = 0;
					uint32_t gltf_sample1 = 0;
					float interpolation_alpha = 0.0F;
					find_linear_interpolation_values(sample_times, sample_time, gltf_sample0, gltf_sample1, interpolation_alpha);

					const rtm::vector4f sample0 = rtm::vector_load3(&sample_values[gltf_sample0]);
					const rtm::vector4f sample1 = rtm::vector_load3(&sample_values[gltf_sample1]);
					const rtm::vector4f sample = rtm::vector_lerp(sample0, sample1, interpolation_alpha);

					bone.translation_track.set_sample(sample_index, rtm::vector_cast(sample));
				}
			}
			else
			{
				ACL_ASSERT(false, "Interpolation mode not supported");
			}
		}
		else if (channel.target_path == "scale")
		{
			ACL_ASSERT(sample_value_accessor.type == TINYGLTF_TYPE_VEC3, "Unexpected accessor type");

			const acl::track_float3f sample_values = make_track_ref<acl::track_float3f>(model, sample_value_accessor);

			if (sampler.interpolation == "LINEAR")
			{
				for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
				{
					const float sample_time = rtm::scalar_min(float(sample_index) / sample_rate, duration);

					uint32_t gltf_sample0 = 0;
					uint32_t gltf_sample1 = 0;
					float interpolation_alpha = 0.0F;
					find_linear_interpolation_values(sample_times, sample_time, gltf_sample0, gltf_sample1, interpolation_alpha);

					const rtm::vector4f sample0 = rtm::vector_load3(&sample_values[gltf_sample0]);
					const rtm::vector4f sample1 = rtm::vector_load3(&sample_values[gltf_sample1]);
					const rtm::vector4f sample = rtm::vector_lerp(sample0, sample1, interpolation_alpha);

					bone.scale_track.set_sample(sample_index, rtm::vector_cast(sample));
				}
			}
			else
			{
				ACL_ASSERT(false, "Interpolation mode not supported");
			}
		}
	}

	return clip;
}

static uint32_t get_raw_animation_size(const tinygltf::Model& model, const tinygltf::Animation& animation)
{
	uint32_t raw_size = 0;

	for (const tinygltf::AnimationChannel& channel : animation.channels)
	{
		const tinygltf::AnimationSampler& sampler = animation.samplers[channel.sampler];
		const tinygltf::Accessor& sample_time_accessor = model.accessors[sampler.input];
		const tinygltf::Accessor& sample_value_accessor = model.accessors[sampler.output];

		const acl::track_float1f sample_times = make_track_ref<acl::track_float1f>(model, sample_time_accessor);
		raw_size += sample_times.get_sample_size() * sample_times.get_num_samples();

		if (channel.target_path == "rotation")
		{
			const acl::track_float4f sample_values = make_track_ref<acl::track_float4f>(model, sample_value_accessor);
			raw_size += sample_values.get_sample_size() * sample_values.get_num_samples();
		}
		else if (channel.target_path == "translation" || channel.target_path == "scale")
		{
			const acl::track_float3f sample_values = make_track_ref<acl::track_float3f>(model, sample_value_accessor);
			raw_size += sample_values.get_sample_size() * sample_values.get_num_samples();
		}
	}

	return raw_size;
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
		printf("Failed to parse glTF\n");
		return false;
	}

	if (!validate_input(model))
		return false;

	bool any_failed = false;
	bool any_compressed = false;
	std::vector<int> animation_buffers;

	const bool is_already_compressed_with_acl = std::find(model.extensionsUsed.begin(), model.extensionsUsed.end(), std::string(k_acl_gltf_extension_str)) != model.extensionsUsed.end();
	if (!is_already_compressed_with_acl)
	{
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
				model.bufferViews[sample_time_accessor.bufferView] = tinygltf::BufferView();
				model.bufferViews[sample_value_accessor.bufferView] = tinygltf::BufferView();

				// Set our new buffer view
				sample_time_accessor.bufferView = static_cast<int>(model.bufferViews.size() - 1);
				sample_value_accessor.bufferView = static_cast<int>(model.bufferViews.size() - 1);
			}

			any_compressed = true;
		}
	}
	else
		printf("Input is already compressed with ACL\n");

	if (any_compressed || is_already_compressed_with_acl)
	{
		if (!is_already_compressed_with_acl)
		{
			model.extensionsRequired.push_back(k_acl_gltf_extension_str);
			model.extensionsUsed.push_back(k_acl_gltf_extension_str);
		}

		// Clear any buffers the animation data used if we managed to compress everything.
		// This assumes that buffers referenced by raw animation data contain only raw animation data
		// and that it is no longer needed once compressed.
		// If a single animation fails to compress, we do not remove any raw data.
		if (!any_failed)
		{
			for (int buffer_index : animation_buffers)
			{
				model.buffers[buffer_index] = tinygltf::Buffer();

				// HACK BEGIN
				// Due to a bug in tinygltf, buffers with 0 length perform an invalid access
				// Set them to 1 byte
				model.buffers[buffer_index].data.resize(1);
				// HACK END
			}
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
