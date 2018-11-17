project('perf_counters', 'cpp',default_options : ['cpp_std=c++14'])

libdrm = dependency('libdrm')

mesa_inc = include_directories('third_party/mesa')
perf_counters = executable('perf_counters',
                           ['source/drm_device.cpp',
                            'source/drm_device.hpp',
                            'source/main.cpp',
                            'source/native.cpp',
                            'source/native.hpp'],
                           dependencies: [libdrm],
                           include_directories: mesa_inc,
                           install: true)
