#!/usr/bin/env python

import os
import ycm_core

flags = [
'-Wall',
'-Wextra',
'-Werror',
'-fexceptions',
'-DNDEBUG',
'-std=c++11',
'-x',
'c++',
'-isystem',
'/usr/include',
'-isystem',
'/usr/local/include',
'-isystem',
'/opt/ros/' + os.getenv('ROS_DISTRO') + '/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/devel/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/CMakeLists.txt/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/__init__.py/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/baxter_pkg/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/bot_description/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/bot_gazebo/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/bot_head/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/bot_moveit/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/bot_nav/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/bot_utils/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/bot_vision/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/cv_bridge/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/image_geometry/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/opencv_tests/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/pr2_pkg/include',
'-isystem',
'/home/mtb/sim_ws/bot_ws/src/vision_opencv/include'
]

compilation_database_folder = ''

if os.path.exists( compilation_database_folder ):
  database = ycm_core.CompilationDatabase( compilation_database_folder )
else:
  database = None

SOURCE_EXTENSIONS = [ '.cpp', '.cxx', '.cc', '.c' ]

def DirectoryOfThisScript():
  return os.path.dirname( os.path.abspath( __file__ ) )


def MakeRelativePathsInFlagsAbsolute( flags, working_directory ):
  if not working_directory:
    return list( flags )
  new_flags = []
  make_next_absolute = False
  path_flags = [ '-isystem', '-I', '-iquote', '--sysroot=' ]
  for flag in flags:
    new_flag = flag

    if make_next_absolute:
      make_next_absolute = False
      if not flag.startswith( '/' ):
        new_flag = os.path.join( working_directory, flag )

    for path_flag in path_flags:
      if flag == path_flag:
        make_next_absolute = True
        break

      if flag.startswith( path_flag ):
        path = flag[ len( path_flag ): ]
        new_flag = path_flag + os.path.join( working_directory, path )
        break

    if new_flag:
      new_flags.append( new_flag )
  return new_flags


def IsHeaderFile( filename ):
  extension = os.path.splitext( filename )[ 1 ]
  return extension in [ '.h', '.hxx', '.hpp', '.hh' ]


def GetCompilationInfoForFile( filename ):
  if IsHeaderFile( filename ):
    basename = os.path.splitext( filename )[ 0 ]
    for extension in SOURCE_EXTENSIONS:
      replacement_file = basename + extension
      if os.path.exists( replacement_file ):
        compilation_info = database.GetCompilationInfoForFile(
          replacement_file )
        if compilation_info.compiler_flags_:
          return compilation_info
    return None
  return database.GetCompilationInfoForFile( filename )


def FlagsForFile( filename, **kwargs ):
  if database:
    compilation_info = GetCompilationInfoForFile( filename )
    if not compilation_info:
      return None

    final_flags = MakeRelativePathsInFlagsAbsolute(
      compilation_info.compiler_flags_,
      compilation_info.compiler_working_dir_ )
  else:
    relative_to = DirectoryOfThisScript()
    final_flags = MakeRelativePathsInFlagsAbsolute( flags, relative_to )

  return {
    'flags': final_flags,
    'do_cache': True
  }
