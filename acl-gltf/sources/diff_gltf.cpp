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
#include "diff_gltf.h"
#include "tinygltf_impl.h"

#include <acl/algorithm/uniformly_sampled/decoder.h>
#include <acl/compression/skeleton.h>
#include <acl/compression/skeleton_error_metric.h>
#include <acl/compression/track.h>
#include <acl/compression/track_array.h>
#include <acl/compression/track_error.h>
#include "acl/compression/utils.h"
#include <acl/decompression/decompress.h>
#include "acl/decompression/default_output_writer.h"
#include <acl/core/ansi_allocator.h>
#include <acl/core/utils.h>

#include <rtm/scalarf.h>

#include <cmath>
#include <limits>
#include <memory>
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
		printf("glTF input does not contain any nodes, nothing to diff\n");
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

	return true;
}

static bool measure_transform_nodes_error(const tinygltf::Model& model0, const tinygltf::Animation& animation0, const tinygltf::Model& model1, const tinygltf::Animation& animation1)
{
	acl::ANSIAllocator allocator;

	// Load up our skeleton, they should be identical, pick one
	const hierarchy_description hierarchy0 = build_hierarchy(model0);
	const acl::RigidSkeleton skeleton0 = build_skeleton(model0, hierarchy0, allocator);

	// Load our clips into ACL structures
	std::unique_ptr<acl::AnimationClip> clip0;
	std::unique_ptr<acl::AnimationClip> clip1;
	const acl::CompressedClip* compressed_clip0 = nullptr;
	const acl::CompressedClip* compressed_clip1 = nullptr;

	int acl_buffer_view_index0 = -1;
	if (is_animation_compressed_with_acl(model0, animation0, acl_buffer_view_index0))
	{
		const int acl_buffer_index0 = model0.bufferViews[acl_buffer_view_index0].buffer;

		const tinygltf::Buffer& acl_buffer0 = model0.buffers[acl_buffer_index0];

		compressed_clip0 = reinterpret_cast<const acl::CompressedClip*>(acl_buffer0.data.data());
		const acl::ErrorResult is_valid_result = compressed_clip0->is_valid(true);
		if (is_valid_result.any())
		{
			printf("    ACL data is invalid: %s\n", is_valid_result.c_str());
			return false;
		}
	}
	else
	{
		clip0.reset(new acl::AnimationClip(build_clip(model0, animation0, hierarchy0, skeleton0, allocator)));
	}

	int acl_buffer_view_index1 = -1;
	if (is_animation_compressed_with_acl(model1, animation1, acl_buffer_view_index1))
	{
		const int acl_buffer_index1 = model1.bufferViews[acl_buffer_view_index1].buffer;

		const tinygltf::Buffer& acl_buffer1 = model1.buffers[acl_buffer_index1];

		compressed_clip1 = reinterpret_cast<const acl::CompressedClip*>(acl_buffer1.data.data());
		const acl::ErrorResult is_valid_result = compressed_clip1->is_valid(true);
		if (is_valid_result.any())
		{
			printf("    ACL data is invalid: %s\n", is_valid_result.c_str());
			return false;
		}
	}
	else
	{
		clip1.reset(new acl::AnimationClip(build_clip(model1, animation1, hierarchy0, skeleton0, allocator)));
	}

	// Calculate the error between the two clips
	acl::qvvf_transform_error_metric error_metric;
	acl::BoneError bone_error;

	if (clip0 && clip1)
	{
		// Both clips are raw
		bone_error = acl::calculate_error_between_clips(allocator, error_metric, *clip0, *clip1);
	}
	else if (clip0 && compressed_clip1 != nullptr)
	{
		// First clip is raw, the second has compressed ACL data
		acl::uniformly_sampled::DecompressionContext<acl::uniformly_sampled::DebugDecompressionSettings> context;
		context.initialize(*compressed_clip1);

		bone_error = acl::calculate_error_between_clips(allocator, error_metric, *clip0, context);
	}
	else if (compressed_clip0 != nullptr && clip1)
	{
		// First has compressed ACL data, the second is raw
		acl::uniformly_sampled::DecompressionContext<acl::uniformly_sampled::DebugDecompressionSettings> context;
		context.initialize(*compressed_clip0);

		bone_error = acl::calculate_error_between_clips(allocator, error_metric, *clip1, context);
	}
	else if (compressed_clip0 != nullptr && compressed_clip1 != nullptr)
	{
		// Both have compressed ACL data
		acl::uniformly_sampled::DecompressionContext<acl::uniformly_sampled::DebugDecompressionSettings> context0;
		context0.initialize(*compressed_clip0);

		acl::uniformly_sampled::DecompressionContext<acl::uniformly_sampled::DebugDecompressionSettings> context1;
		context1.initialize(*compressed_clip1);

		bone_error = acl::calculate_error_between_clips(allocator, error_metric, skeleton0, context0, context1);
	}
	else
	{
		ACL_ASSERT(false, "Invalid state");
		return false;
	}

	const char* worst_bone_name = bone_error.index == acl::k_invalid_bone_index ? "<Unknown>" : skeleton0.get_bone(bone_error.index).name.c_str();
	printf("    Largest error of %.2f mm on transform %u ('%s') @ %.2f sec\n", bone_error.error * 1000.0F, bone_error.index, worst_bone_name, bone_error.sample_time);

	// Success
	return true;
}

static bool measure_scalar_weights_error(const tinygltf::Model& model0, const tinygltf::Animation& animation0, const tinygltf::Model& model1, const tinygltf::Animation& animation1)
{
	acl::ANSIAllocator allocator;

	const std::vector<const tinygltf::AnimationChannel*> channels0 = find_weight_channels(animation0);
	const std::vector<const tinygltf::AnimationChannel*> channels1 = find_weight_channels(animation1);

	const size_t num_channels = channels0.size();
	if (num_channels != channels1.size())
	{
		printf("Input animations do not have the same number of weight channels, cannot diff\n");
		return false;
	}

	for (size_t channel_index = 0; channel_index < num_channels; ++channel_index)
	{
		const tinygltf::AnimationChannel* channel0 = channels0[channel_index];
		const tinygltf::AnimationChannel* channel1 = channels1[channel_index];
		if (channel0->target_node != channel1->target_node)
		{
			printf("Animation weight channels do not target the same node, cannot diff\n");
			return false;
		}

		const std::vector<double>& default_weights0 = find_default_weights(model0, model0.nodes[channel0->target_node]);
		const std::vector<double>& default_weights1 = find_default_weights(model1, model1.nodes[channel1->target_node]);
		if (default_weights0 != default_weights1)
		{
			printf("Input animations do not have the same default weights, cannot diff\n");
			return false;
		}

		if (default_weights0.empty())
			continue;	// No weights to diff

		// Load our scalar weights into ACL structures
		std::unique_ptr<acl::track_array_float1f> raw_tracks0;
		std::unique_ptr<acl::track_array_float1f> raw_tracks1;
		const acl::compressed_tracks* compressed_tracks0 = nullptr;
		const acl::compressed_tracks* compressed_tracks1 = nullptr;

		int acl_buffer_view_index0 = -1;
		if (is_animation_compressed_with_acl(model0, animation0, acl_buffer_view_index0))
		{
			int acl_weights_buffer_view_index;
			if (!has_compressed_weights_with_acl(model0, animation0, *channel0, acl_weights_buffer_view_index))
			{
				printf("Expected scalar weights to be compressed with ACL, cannot diff\n");
				return false;
			}

			const int acl_weights_buffer_index = model0.bufferViews[acl_weights_buffer_view_index].buffer;
			const tinygltf::Buffer& acl_buffer = model0.buffers[acl_weights_buffer_index];

			compressed_tracks0 = reinterpret_cast<const acl::compressed_tracks*>(acl_buffer.data.data());
			const acl::ErrorResult is_valid_result = compressed_tracks0->is_valid(true);
			if (is_valid_result.any())
			{
				printf("    ACL weight data is invalid: %s\n", is_valid_result.c_str());
				return false;
			}
		}
		else
		{
			raw_tracks0.reset(new acl::track_array_float1f(build_weight_tracks(model0, animation0, *channel0, allocator)));
		}

		int acl_buffer_view_index1 = -1;
		if (is_animation_compressed_with_acl(model1, animation1, acl_buffer_view_index1))
		{
			int acl_weights_buffer_view_index;
			if (!has_compressed_weights_with_acl(model1, animation1, *channel1, acl_weights_buffer_view_index))
			{
				printf("Expected scalar weights to be compressed with ACL, cannot diff\n");
				return false;
			}

			const int acl_weights_buffer_index = model1.bufferViews[acl_weights_buffer_view_index].buffer;
			const tinygltf::Buffer& acl_buffer = model1.buffers[acl_weights_buffer_index];

			compressed_tracks1 = reinterpret_cast<const acl::compressed_tracks*>(acl_buffer.data.data());
			const acl::ErrorResult is_valid_result = compressed_tracks1->is_valid(true);
			if (is_valid_result.any())
			{
				printf("    ACL weight data is invalid: %s\n", is_valid_result.c_str());
				return false;
			}
		}
		else
		{
			raw_tracks1.reset(new acl::track_array_float1f(build_weight_tracks(model1, animation1, *channel1, allocator)));
		}

		// Calculate the error between the track list inputs
		acl::track_error track_err;

		if (raw_tracks0 && raw_tracks1)
		{
			// Both track lists are raw
			track_err = acl::calculate_compression_error(allocator, *raw_tracks0, *raw_tracks1);
		}
		else if (raw_tracks0 && compressed_tracks1 != nullptr)
		{
			// First track list is raw, the second has compressed ACL data
			acl::decompression_context<acl::debug_decompression_settings> context;
			context.initialize(*compressed_tracks1);

			track_err = acl::calculate_compression_error(allocator, *raw_tracks0, context);
		}
		else if (compressed_tracks0 != nullptr && raw_tracks1)
		{
			// First has compressed ACL data, the second is raw
			acl::decompression_context<acl::debug_decompression_settings> context;
			context.initialize(*compressed_tracks0);

			track_err = acl::calculate_compression_error(allocator, *raw_tracks1, context);
		}
		else if (compressed_tracks0 != nullptr && compressed_tracks1 != nullptr)
		{
			// Both have compressed ACL data
			acl::decompression_context<acl::debug_decompression_settings> context0;
			context0.initialize(*compressed_tracks0);

			acl::decompression_context<acl::debug_decompression_settings> context1;
			context1.initialize(*compressed_tracks1);

			track_err = acl::calculate_compression_error(allocator, context0, context1);
		}
		else
		{
			ACL_ASSERT(false, "Invalid state");
			return false;
		}

		printf("    Largest weight error of %.4f on weight track %u @ %.2f sec\n", track_err.error, track_err.index, track_err.sample_time);
	}

	// Success
	return true;
}

bool diff_gltf(const command_line_options& options)
{
	if (options.input_filename0 == options.input_filename1)
	{
		printf("Both inputs cannot be the same file\n");
		return false;
	}

	tinygltf::Model model0;
	tinygltf::TinyGLTF loader0;

	tinygltf::Model model1;
	tinygltf::TinyGLTF loader1;

	std::string err;
	std::string warn;

	bool success;
	if (is_binary_gltf_filename(options.input_filename0))
		success = loader0.LoadBinaryFromFile(&model0, &err, &warn, options.input_filename0);
	else
		success = loader0.LoadASCIIFromFile(&model0, &err, &warn, options.input_filename0);

	if (!warn.empty())
		printf("Warn: %s\n", warn.c_str());

	if (!err.empty())
		printf("Err: %s\n", err.c_str());

	if (!success)
	{
		printf("Failed to parse glTF input: %s\n", options.input_filename0.c_str());
		return false;
	}

	if (!validate_input(model0))
		return false;

	if (is_binary_gltf_filename(options.input_filename1))
		success = loader1.LoadBinaryFromFile(&model1, &err, &warn, options.input_filename1);
	else
		success = loader1.LoadASCIIFromFile(&model1, &err, &warn, options.input_filename1);

	if (!warn.empty())
		printf("Warn: %s\n", warn.c_str());

	if (!err.empty())
		printf("Err: %s\n", err.c_str());

	if (!success)
	{
		printf("Failed to parse glTF input: %s\n", options.input_filename1.c_str());
		return false;
	}

	if (!validate_input(model1))
		return false;

	const size_t num_animations = model0.animations.size();
	if (num_animations != model1.animations.size())
	{
		printf("Inputs do not have the same number of animations, cannot diff\n");
		return false;
	}

	if (model0.animations.empty())
	{
		printf("glTF inputs do not contain any animations, nothing to diff\n");
		return true;
	}

	bool any_failed = false;

	for (size_t animation_index = 0; animation_index < num_animations; ++animation_index)
	{
		const tinygltf::Animation& animation0 = model0.animations[animation_index];
		const tinygltf::Animation& animation1 = model1.animations[animation_index];

		if (animation0.name != animation1.name)
		{
			printf("Animations do not have the same name at index %u, cannot diff\n", (uint32_t)animation_index);
			any_failed = true;
			continue;
		}

		printf("Processing animation '%s' ...\n", animation0.name.empty() ? "<unnamed>" : animation0.name.c_str());

		// Measure the error for our hierarchical transforms
		const bool measured_transform_error = measure_transform_nodes_error(model0, animation0, model1, animation1);
		if (!measured_transform_error)
		{
			any_failed = true;
			continue;
		}

		// Measure the error for our scalar weights
		const bool measured_scalar_weights_error = measure_scalar_weights_error(model0, animation0, model1, animation1);
		if (!measured_scalar_weights_error)
		{
			any_failed = true;
			continue;
		}
	}

	return !any_failed;
}
