import argparse
import multiprocessing
import os
import platform
import shutil
import subprocess
import sys

def parse_argv():
	parser = argparse.ArgumentParser(add_help=False)

	actions = parser.add_argument_group(title='Actions', description='If no action is specified, on Windows, OS X, and Linux the solution/make files are generated.  Multiple actions can be used simultaneously.')
	actions.add_argument('-build', action='store_true')
	actions.add_argument('-clean', action='store_true')
	actions.add_argument('-compress', nargs=2)

	target = parser.add_argument_group(title='Target')
	target.add_argument('-compiler', choices=['vs2015', 'vs2017', 'vs2019', 'clang4', 'clang5', 'clang6', 'clang7', 'clang8', 'clang9', 'gcc5', 'gcc6', 'gcc7', 'gcc8', 'gcc9', 'osx'], help='Defaults to the host system\'s default compiler')
	target.add_argument('-config', choices=['Debug', 'Release'], type=str.capitalize)
	target.add_argument('-cpu', choices=['x86', 'x64'], help='Only supported for Windows, OS X, and Linux; defaults to the host system\'s architecture')

	misc = parser.add_argument_group(title='Miscellaneous')
	misc.add_argument('-avx', dest='use_avx', action='store_true', help='Compile using AVX instructions on Windows, OS X, and Linux')
	misc.add_argument('-nosimd', dest='use_simd', action='store_false', help='Compile without SIMD instructions')
	misc.add_argument('-num_threads', help='No. to use while compiling and regressing')
	misc.add_argument('-help', action='help', help='Display this usage information')

	num_threads = multiprocessing.cpu_count()
	if platform.system() == 'Linux' and sys.version_info >= (3, 4):
		num_threads = len(os.sched_getaffinity(0))
	if not num_threads or num_threads == 0:
		num_threads = 4

	parser.set_defaults(build=False, clean=False, compiler=None, config='Release', cpu='x64', use_avx=False, use_simd=True, num_threads=num_threads)

	args = parser.parse_args()

	# Sanitize and validate our options
	if args.use_avx and not args.use_simd:
		print('SIMD is explicitly disabled, AVX will not be used')
		args.use_avx = False

	return args

def get_cmake_exes():
	if platform.system() == 'Windows':
		return ('cmake.exe', 'ctest.exe')
	else:
		return ('cmake', 'ctest')

def get_generator(compiler, cpu):
	if compiler == None:
		return None

	if platform.system() == 'Windows':
		if compiler == 'vs2015':
			if cpu == 'x86':
				return 'Visual Studio 14'
			elif cpu == 'x64':
				return 'Visual Studio 14 Win64'
		elif compiler == 'vs2017':
			if cpu == 'x86':
				return 'Visual Studio 15'
			elif cpu == 'x64':
				return 'Visual Studio 15 Win64'
		elif compiler == 'vs2019':
			return 'Visual Studio 16 2019'
		elif compiler == 'android':
			return 'Visual Studio 14'
	elif platform.system() == 'Darwin':
		if compiler == 'osx':
			return 'Xcode'
	else:
		return 'Unix Makefiles'

	print('Unknown compiler: {}'.format(compiler))
	print('See help with: python make.py -help')
	sys.exit(1)

def get_architecture(compiler, cpu):
	if compiler == None:
		return None

	if platform.system() == 'Windows':
		if compiler == 'vs2019':
			if cpu == 'x86':
				return 'Win32'
			else:
				return cpu

	# This compiler/cpu pair does not need the architecture switch
	return None

def set_compiler_env(compiler, args):
	if platform.system() == 'Linux':
		os.environ['MAKEFLAGS'] = '-j{}'.format(args.num_threads)
		if compiler == 'clang4':
			os.environ['CC'] = 'clang-4.0'
			os.environ['CXX'] = 'clang++-4.0'
		elif compiler == 'clang5':
			os.environ['CC'] = 'clang-5.0'
			os.environ['CXX'] = 'clang++-5.0'
		elif compiler == 'clang6':
			os.environ['CC'] = 'clang-6.0'
			os.environ['CXX'] = 'clang++-6.0'
		elif compiler == 'clang7':
			os.environ['CC'] = 'clang-7'
			os.environ['CXX'] = 'clang++-7'
		elif compiler == 'clang8':
			os.environ['CC'] = 'clang-8'
			os.environ['CXX'] = 'clang++-8'
		elif compiler == 'clang9':
			os.environ['CC'] = 'clang-9'
			os.environ['CXX'] = 'clang++-9'
		elif compiler == 'gcc5':
			os.environ['CC'] = 'gcc-5'
			os.environ['CXX'] = 'g++-5'
		elif compiler == 'gcc6':
			os.environ['CC'] = 'gcc-6'
			os.environ['CXX'] = 'g++-6'
		elif compiler == 'gcc7':
			os.environ['CC'] = 'gcc-7'
			os.environ['CXX'] = 'g++-7'
		elif compiler == 'gcc8':
			os.environ['CC'] = 'gcc-8'
			os.environ['CXX'] = 'g++-8'
		elif compiler == 'gcc9':
			os.environ['CC'] = 'gcc-9'
			os.environ['CXX'] = 'g++-9'
		else:
			print('Unknown compiler: {}'.format(compiler))
			print('See help with: python make.py -help')
			sys.exit(1)

def do_generate_solution(cmake_exe, build_dir, cmake_script_dir, args):
	compiler = args.compiler
	cpu = args.cpu
	config = args.config

	if not compiler == None:
		set_compiler_env(compiler, args)

	extra_switches = ['--no-warn-unused-cli']
	extra_switches.append('-DCPU_INSTRUCTION_SET:STRING={}'.format(cpu))

	if args.use_avx:
		print('Enabling AVX usage')
		extra_switches.append('-DUSE_AVX_INSTRUCTIONS:BOOL=true')

	if not args.use_simd:
		print('Disabling SIMD instruction usage')
		extra_switches.append('-DUSE_SIMD_INSTRUCTIONS:BOOL=false')

	if not platform.system() == 'Windows' and not platform.system() == 'Darwin':
		extra_switches.append('-DCMAKE_BUILD_TYPE={}'.format(config.upper()))

	# Generate IDE solution
	print('Generating build files ...')
	cmake_cmd = '"{}" .. -DCMAKE_INSTALL_PREFIX="{}" {}'.format(cmake_exe, build_dir, ' '.join(extra_switches))
	cmake_generator = get_generator(compiler, cpu)
	if cmake_generator == None:
		print('Using default generator')
	else:
		print('Using generator: {}'.format(cmake_generator))
		cmake_cmd += ' -G "{}"'.format(cmake_generator)

	cmake_arch = get_architecture(compiler, cpu)
	if cmake_arch:
		print('Using architecture: {}'.format(cmake_arch))
		cmake_cmd += ' -A {}'.format(cmake_arch)

	result = subprocess.call(cmake_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def do_build(cmake_exe, args):
	config = args.config

	print('Building ...')
	cmake_cmd = '"{}" --build .'.format(cmake_exe)
	if platform.system() == 'Windows':
		if args.compiler == 'android':
			cmake_cmd += ' --config {}'.format(config)
		else:
			cmake_cmd += ' --config {} --target INSTALL'.format(config)
	elif platform.system() == 'Darwin':
		cmake_cmd += ' --config {} --target install'.format(config)
	else:
		cmake_cmd += ' --target install'

	result = subprocess.call(cmake_cmd, shell=True)
	if result != 0:
		sys.exit(result)

def compress_file(acl_gltf_exe_path, input_filename, output_filename):
	print('Compressing \'{}\' ...'.format(os.path.basename(input_filename)))
	args = [acl_gltf_exe_path, '--compress', input_filename, output_filename]

	try:
		output = subprocess.check_output(args)
		print(output.decode(sys.stdout.encoding))
		return True
	except subprocess.CalledProcessError as e:
		print('Failed to compress glTF file: {}'.format(" ".join(args)))
		print(e.output.decode(sys.stdout.encoding))
		return False

def do_compress(args):
	# We are already located in the build directory
	if platform.system() == 'Windows':
		acl_gltf_exe_path = 'bin/acl-gltf.exe'
	else:
		acl_gltf_exe_path = 'bin/acl-gltf'

	acl_gltf_exe_path = os.path.abspath(acl_gltf_exe_path)
	if not os.path.exists(acl_gltf_exe_path):
		print('acl-gltf executable not found: {}'.format(acl_gltf_exe_path))
		sys.exit(1)

	input_path = args.compress[0]
	output_path = args.compress[1]

	if os.path.abspath(input_path) == os.path.abspath(output_path):
		print('Input and output must be different')
		sys.exit(1)

	# Handle long paths on Windows
	if platform.system() == 'Windows':
		input_path = '\\\\?\\{}'.format(input_path)
		output_path = '\\\\?\\{}'.format(output_path)

	is_input_dir = os.path.isdir(input_path)
	is_output_dir = os.path.isdir(output_path)
	does_output_exist = os.path.exists(output_path)
	if is_input_dir and (is_output_dir or not does_output_exist):
		# Input and output are directories, we'll iterate over every input file and generate an output with the same name
		print('')
		print('Compressing \'{}\\*\' -> \'{}\\*\' ...'.format(args.compress[0], args.compress[1]))

		if not does_output_exist:
			os.makedirs(output_path)

		# tinygltf attemps to load dependent binary/texture data from the current working directory
		old_cwd = os.getcwd()
		os.chdir(input_path)

		success = True

		for (dirpath, dirnames, filenames) in os.walk(input_path):
			for filename in filenames:
				if not filename.endswith('.gltf') and not filename.endswith('.glb'):
					continue

				input_filename = os.path.join(input_path, filename)
				output_filename = os.path.join(output_path, filename)

				if not compress_file(acl_gltf_exe_path, input_filename, output_filename):
					success = False

		os.chdir(old_cwd)

		if not success:
			sys.exit(1)
	elif not is_input_dir and (not is_output_dir or not does_output_exist):
		# Input and output are files
		print('')
		print('Compressing \'{}\' -> \'{}\' ...'.format(args.compress[0], args.compress[1]))

		# tinygltf attemps to load dependent binary/texture data from the current working directory
		old_cwd = os.getcwd()
		os.chdir(os.path.dirname(input_path))

		success = compress_file(acl_gltf_exe_path, input_path, output_path)

		os.chdir(old_cwd)

		if not success:
			sys.exit(1)
	else:
		print('Both input and output must either be files or directories, mixing not supported')
		sys.exit(1)

if __name__ == "__main__":
	args = parse_argv()

	cmake_exe, ctest_exe = get_cmake_exes()
	compiler = args.compiler
	cpu = args.cpu
	config = args.config

	# Set the ACL_GLTF_CMAKE_HOME environment variable to point to CMake
	# otherwise we assume it is already in the user PATH
	if 'ACL_GLTF_CMAKE_HOME' in os.environ:
		cmake_home = os.environ['ACL_GLTF_CMAKE_HOME']
		cmake_exe = os.path.join(cmake_home, 'bin', cmake_exe)
		ctest_exe = os.path.join(cmake_home, 'bin', ctest_exe)

	build_dir = os.path.join(os.getcwd(), 'build')
	cmake_script_dir = os.path.join(os.getcwd(), 'cmake')

	if args.clean and os.path.exists(build_dir):
		print('Cleaning previous build ...')
		shutil.rmtree(build_dir)

	if not os.path.exists(build_dir):
		os.makedirs(build_dir)

	os.chdir(build_dir)

	print('Using config: {}'.format(config))
	print('Using cpu: {}'.format(cpu))
	if not compiler == None:
		print('Using compiler: {}'.format(compiler))
	print('Using {} threads'.format(args.num_threads))

	do_generate_solution(cmake_exe, build_dir, cmake_script_dir, args)

	if args.build:
		do_build(cmake_exe, args)

	if args.compress:
		do_compress(args)

	sys.exit(0)
