project('rs_logger', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'cpp_std=c++14'])

dep_realsense = dependency('realsense2')

executable('rs_logger',
           'src/rs-record-trajectory.cpp',
           dependencies: dep_realsense,
           install : true)
