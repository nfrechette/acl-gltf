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

	acl::ANSIAllocator allocator;
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

		// Load up our skeleton, they should be identical, pick one
		const std::vector<uint16_t> node_parent_indices0 = build_node_parent_indices(model0);
		const acl::RigidSkeleton skeleton0 = build_skeleton(model0, node_parent_indices0, allocator);

		// Load our clips into ACL structures
		const acl::AnimationClip* clip0 = nullptr;
		const acl::AnimationClip* clip1 = nullptr;
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
				any_failed = true;
				continue;
			}
		}
		else
		{
			clip0 = new acl::AnimationClip(build_clip(model0, animation0, skeleton0, allocator));
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
				any_failed = true;
				continue;
			}
		}
		else
		{
			clip1 = new acl::AnimationClip(build_clip(model1, animation1, skeleton0, allocator));
		}

		// Calculate the error between the two clips
		acl::qvvf_transform_error_metric error_metric;
		acl::BoneError bone_error;

		if (clip0 != nullptr && clip1 != nullptr)
		{
			// Both clips are raw
			bone_error = acl::calculate_error_between_clips(allocator, error_metric, *clip0, *clip1);
		}
		else if (clip0 != nullptr && compressed_clip1 != nullptr)
		{
			// First clip is raw, the second has compressed ACL data
			acl::uniformly_sampled::DecompressionContext<acl::uniformly_sampled::DebugDecompressionSettings> context;
			context.initialize(*compressed_clip1);

			bone_error = acl::calculate_error_between_clips(allocator, error_metric, *clip0, context);
		}
		else if (compressed_clip0 != nullptr && clip1 != nullptr)
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
			any_failed = true;
		}

		const char* worst_bone_name = bone_error.index == acl::k_invalid_bone_index ? "<Unknown>" : skeleton0.get_bone(bone_error.index).name.c_str();
		printf("    Largest error of %.2f mm on transform %u ('%s') @ %.2f sec\n", bone_error.error * 100.0F * 100.0F, bone_error.index, worst_bone_name, bone_error.sample_time);

		// Free our raw clips if they were allocated
		delete clip0;
		delete clip1;
	}

	return !any_failed;
}
