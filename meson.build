project('rs_logger', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'cpp_std=c++17'])

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
