project(
  'botcli',
  'cpp',
  version : '0.1.0',
  default_options : [
    'cpp_std=c++17',
    'warning_level=2',
  ],
)

cpp = meson.get_compiler('cpp')

project_includes = include_directories('src', 'third_party')

kipr_dep = cpp.find_library('kipr', dirs : ['/usr/local/lib'])
math_dep = cpp.find_library('m', required : false)
zlib_dep = dependency('zlib', required : false)
threads_dep = dependency('threads')

executable(
  'botcli',
  'src/main.cpp',
  include_directories : project_includes,
  cpp_args : [
    '-I/usr/local/include',
    '-I/usr/local/include/include',
  ],
  dependencies : [
    kipr_dep,
    math_dep,
    zlib_dep,
    threads_dep,
  ],
  install : true,
)

request_id_test = executable(
  'request_id_test',
  'tests/request_id_test.cpp',
  include_directories : project_includes,
)

test('request_id', request_id_test)
