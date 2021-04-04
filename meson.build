project('rs_logger', 'cpp',
  version : '0.1',
  default_options : ['warning_level=1', 'cpp_std=c++17'])

if host_machine.cpu_family() == 'aarch64'
  # ABSL on ARM64 needs this flag: https://www.gitmemory.com/issue/abseil/abseil-cpp/662/612517861
  add_global_arguments('-march=armv8-a+crypto', language: 'cpp')
endif

# Subprojects.
prj_absl = subproject('abseil-cpp')

# Dependencies.
dep_realsense = dependency('realsense2')
dep_absl_flags = prj_absl.get_variable('absl_flags_dep')
dep_absl_strings = prj_absl.get_variable('absl_strings_dep')

executable('rs_logger',
           'src/rs-record-trajectory.cpp',
           dependencies: [dep_realsense, dep_absl_flags, dep_absl_strings],
           install : true)
