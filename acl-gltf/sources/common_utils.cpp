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

#include "common_utils.h"

#include <queue>
#include <vector>

std::vector<uint16_t> build_node_parent_indices(const tinygltf::Model& model)
{
	const uint16_t num_transforms = static_cast<uint16_t>(model.nodes.size());

	std::vector<uint16_t> node_parent_indices;
	node_parent_indices.resize(num_transforms, acl::k_invalid_bone_index);

	for (uint16_t node_index = 0; node_index < num_transforms; ++node_index)
	{
		const tinygltf::Node& node = model.nodes[node_index];

		for (int child_node_index : node.children)
			node_parent_indices[child_node_index] = node_index;
	}

	return node_parent_indices;
}

std::vector<uint16_t> build_transform_to_node_mapping(const tinygltf::Model& model, const std::vector<uint16_t>& node_parent_indices)
{
	// The nodes in a glTF scene might not be sorted parent first and as such we must sort them for ACL.
	// When we read in our data, we'll read it in sorted bone order and when we decompress with ACL,
	// we'll do so in glTF node order (with the output index on the AnimationClip).

	const uint16_t num_transforms = static_cast<uint16_t>(model.nodes.size());

	std::vector<uint16_t> transform_to_node_mapping;
	std::queue<uint16_t> nodes_to_process;

	// Start off with the indices of each root node
	for (uint16_t node_index = 0; node_index < num_transforms; ++node_index)
	{
		if (node_parent_indices[node_index] == acl::k_invalid_bone_index)
			nodes_to_process.push(node_index);
	}

	while (!nodes_to_process.empty())
	{
		const uint16_t node_index = nodes_to_process.front();
		nodes_to_process.pop();

		// Add the node to our mapping, its children will follow
		transform_to_node_mapping.push_back(node_index);

		// Queue up the children for processing
		const tinygltf::Node& node = model.nodes[node_index];
		for (int child_node_index : node.children)
			nodes_to_process.push(static_cast<uint16_t>(child_node_index));
	}

	return transform_to_node_mapping;
}

std::vector<uint16_t> build_node_to_transform_mapping(const std::vector<uint16_t>& transform_to_node_mapping)
{
	const uint16_t num_transforms = static_cast<uint16_t>(transform_to_node_mapping.size());

	std::vector<uint16_t> node_to_transform_mapping;
	node_to_transform_mapping.resize(num_transforms);

	for (uint16_t transform_index = 0; transform_index < num_transforms; ++transform_index)
		node_to_transform_mapping[transform_to_node_mapping[transform_index]] = transform_index;

	return node_to_transform_mapping;
}

std::vector<uint16_t> build_transform_parent_indices(const std::vector<uint16_t> node_to_transform_mapping, const std::vector<uint16_t>& node_parent_indices, const std::vector<uint16_t> transform_to_node_mapping)
{
	const uint16_t num_transforms = static_cast<uint16_t>(node_to_transform_mapping.size());

	std::vector<uint16_t> transform_parent_indices;
	transform_parent_indices.resize(num_transforms);

	for (uint16_t transform_index = 0; transform_index < num_transforms; ++transform_index)
	{
		const uint16_t parent_node_index = node_parent_indices[transform_to_node_mapping[transform_index]];
		if (parent_node_index == acl::k_invalid_bone_index)
			transform_parent_indices[transform_index] = parent_node_index;
		else
			transform_parent_indices[transform_index] = node_to_transform_mapping[parent_node_index];
	}

	return transform_parent_indices;
}

hierarchy_description build_hierarchy(const tinygltf::Model& model)
{
	const std::vector<uint16_t> node_parent_indices = build_node_parent_indices(model);
	const std::vector<uint16_t> transform_to_node_mapping = build_transform_to_node_mapping(model, node_parent_indices);
	const std::vector<uint16_t> node_to_transform_mapping = build_node_to_transform_mapping(transform_to_node_mapping);
	const std::vector<uint16_t> transform_parent_indices = build_transform_parent_indices(node_to_transform_mapping, node_parent_indices, transform_to_node_mapping);

	hierarchy_description desc;
	desc.node_parent_indices = std::move(node_parent_indices);
	desc.transform_parent_indices = std::move(transform_parent_indices);
	desc.node_to_transform_mapping = std::move(node_to_transform_mapping);
	desc.transform_to_node_mapping = std::move(transform_to_node_mapping);
	desc.num_transforms = static_cast<uint16_t>(model.nodes.size());
	return desc;
}

acl::RigidSkeleton build_skeleton(const tinygltf::Model& model, const hierarchy_description& hierarchy, acl::IAllocator& allocator)
{
	const uint16_t num_transforms = hierarchy.num_transforms;

	std::vector<acl::RigidBone> bones;
	bones.resize(num_transforms);

	for (uint16_t transform_index = 0; transform_index < num_transforms; ++transform_index)
	{
		const uint16_t node_index = hierarchy.transform_to_node_mapping[transform_index];

		const tinygltf::Node& node = model.nodes[node_index];
		acl::RigidBone& bone = bones[transform_index];

		bone.name = acl::String(allocator, node.name.c_str());
		bone.parent_index = hierarchy.transform_parent_indices[transform_index];

		// glTF units are in meters
		bone.vertex_distance = 1.0F;		// TODO: Use skinning information and mesh?

		if (node.matrix.empty())
		{
			if (!node.rotation.empty())
				bone.bind_transform.rotation = rtm::quat_normalize(rtm::quat_load(node.rotation.data()));

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

acl::AnimationClip build_clip(const tinygltf::Model& model, const tinygltf::Animation& animation, const hierarchy_description& hierarchy, const acl::RigidSkeleton& skeleton, acl::IAllocator& allocator)
{
	const float duration = find_clip_duration(model, animation);
	const float sample_rate = find_clip_sample_rate(model, animation, duration);
	const uint32_t num_samples = acl::calculate_num_samples(duration, sample_rate);

	acl::AnimationClip clip(allocator, skeleton, num_samples, sample_rate, acl::String(allocator, animation.name.c_str()));

	// Populate everything with the bind pose and set our output indices in glTF node order
	const uint16_t num_bones = skeleton.get_num_bones();
	for (uint16_t bone_index = 0; bone_index < num_bones; ++bone_index)
	{
		const rtm::qvvd& bind_transform = skeleton.get_bone(bone_index).bind_transform;
		acl::AnimatedBone& bone = clip.get_animated_bone(bone_index);
		bone.output_index = hierarchy.transform_to_node_mapping[bone_index];

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
		const uint16_t node_index = static_cast<uint16_t>(channel.target_node);
		const uint16_t bone_index = hierarchy.node_to_transform_mapping[node_index];
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

					const rtm::quatf sample0 = rtm::quat_normalize(rtm::vector_to_quat(rtm::vector_load(&sample_values[gltf_sample0])));
					const rtm::quatf sample1 = rtm::quat_normalize(rtm::vector_to_quat(rtm::vector_load(&sample_values[gltf_sample1])));
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
		else
		{
			ACL_ASSERT(false, "Channel target not supported: %s", channel.target_path.c_str());
		}
	}

	return clip;
}

uint32_t get_raw_animation_size(const tinygltf::Model& model, const tinygltf::Animation& animation)
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

bool is_animation_compressed_with_acl(const tinygltf::Model& model, const tinygltf::Animation& animation, int& out_acl_buffer_view_index)
{
	out_acl_buffer_view_index = -1;

	if (animation.channels.empty())
		return false;	// No animation data

	for (const tinygltf::AnimationChannel& channel : animation.channels)
	{
		const tinygltf::AnimationSampler& sampler = animation.samplers[channel.sampler];
		const tinygltf::Accessor& sample_time_accessor = model.accessors[sampler.input];
		const tinygltf::Accessor& sample_value_accessor = model.accessors[sampler.output];

		if (sample_time_accessor.bufferView != sample_value_accessor.bufferView)
			return false;	// ACL requires a single buffer

		if (sample_time_accessor.byteOffset != 0 || sample_value_accessor.byteOffset != 0)
			return false;	// ACL requires a single buffer with no offset

		if (sample_time_accessor.componentType != TINYGLTF_COMPONENT_TYPE_BYTE || sample_value_accessor.componentType != TINYGLTF_COMPONENT_TYPE_BYTE)
			return false;	// ACL requires a byte buffer

		if (sample_time_accessor.type != TINYGLTF_TYPE_SCALAR || sample_value_accessor.type != TINYGLTF_TYPE_SCALAR)
			return false;	// ACL requires a scalar byte buffer
	}

	const tinygltf::AnimationSampler& sampler = animation.samplers[animation.channels[0].sampler];
	const tinygltf::Accessor& accessor = model.accessors[sampler.input];
	out_acl_buffer_view_index = accessor.bufferView;

	return true;
}

bool is_binary_gltf_filename(const std::string& filename)
{
	return filename.rfind(".gltf") == std::string::npos;
}

void reset_buffer_view(tinygltf::BufferView& buffer_view)
{
	buffer_view.buffer = 0;		// Index must always be valid
	buffer_view.byteLength = 0;	// Empty
	buffer_view.byteOffset = 0;
	buffer_view.byteStride = 0;
	buffer_view.name.clear();
	buffer_view.target = 0;		// TODO: Should this be set to TINYGLTF_TARGET_ARRAY_BUFFER ? glTF does not appear to define what is or isn't a valid value here
	buffer_view.extras = tinygltf::Value();
	buffer_view.dracoDecoded = false;
}

void reset_buffer(tinygltf::Buffer& buffer)
{
	// HACK BEGIN
	// Due to a bug in tinygltf, buffers with 0 length perform an invalid access
	// Set them to 1 byte
	buffer.data.resize(1);
	// HACK END

	buffer.name.clear();
	buffer.uri.clear();
	buffer.extras = tinygltf::Value();
}
