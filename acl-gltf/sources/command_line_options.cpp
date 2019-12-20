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

command_line_options::command_line_options()
	: compress(false)
	, input_filename0()
	, output_filename()
{}

static void print_usage()
{
	printf("Usage: acl-gltf --compress <input_file> [<output_file>]\n");
	printf("Usage: acl-gltf --decompress <input_file> <output_file>\n");
	printf("Usage: acl-gltf --diff <input_file1> <input_file2>\n");
}

bool parse_command_line_arguments(int argc, char* argv[], command_line_options& out_options)
{
	command_line_options options;

	for (int arg_index = 1; arg_index < argc; ++arg_index)
	{
		const char* argument = argv[arg_index];

		const char* compress_option = "--compress";
		size_t option_length = std::strlen(compress_option);
		if (std::strncmp(argument, compress_option, option_length) == 0)
		{
			if (arg_index + 1 == argc)
			{
				printf("--compress requires an input glTF/glB file\n");
				print_usage();
				return false;
			}

			arg_index++;

			options.compress = true;
			options.input_filename0 = argv[arg_index];

			// If we have no output file, we'll just end up printing the stats which is fine
			if (arg_index + 1 < argc)
			{
				arg_index++;
				options.output_filename = argv[arg_index];
			}
		}

		// Unknown arguments are ignored silently
	}

	if (!options.compress)
	{
		// No options provided, print usage
		print_usage();
		return false;
	}

	out_options = options;
	return true;
}
