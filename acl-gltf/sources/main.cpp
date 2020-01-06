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
#include "compress_gltf.h"
#include "decompress_gltf.h"
#include "diff_gltf.h"

#include <acl/core/error.h>
#include <acl/core/floating_point_exceptions.h>

int main(int argc, char* argv[])
{
	command_line_options options;
	const bool arguments_valid = parse_command_line_arguments(argc, argv, options);
	if (!arguments_valid)
		return 1;

	// Disable floating point exceptions
	acl::scope_disable_fp_exceptions fp_off;

	int exit_code = 0;
	switch (options.action)
	{
	case command_line_action::none:
	default:
		// Unknown state, should never happen
		ACL_ASSERT(false, "Unknown state");
		exit_code = 1;
		break;
	case command_line_action::compress:
		exit_code = compress_gltf(options) ? 0 : 1;
		break;
	case command_line_action::decompress:
		exit_code = decompress_gltf(options) ? 0 : 1;
		break;
	case command_line_action::diff:
		exit_code = diff_gltf(options) ? 0 : 1;
		break;
	}

	return exit_code;
}
