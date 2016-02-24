cmake_minimum_required(VERSION 2.8.0)
project(gtk)

# PODs out-of-source build logic
if (CMAKE_INSTALL_PREFIX STREQUAL "/usr/local" OR CMAKE_INSTALL_PREFIX STREQUAL "C:/Program Files/${PROJECT_NAME}") 
  find_file(_build_dir build PATHS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/.. ${PROJECT_SOURCE_DIR}/../.. ${PROJECT_SOURCE_DIR}/../../.. ${PROJECT_SOURCE_DIR}/../../../..)
  if (_build_dir)
      set(CMAKE_INSTALL_PREFIX "${_build_dir}")
  else()
    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/build)
    set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/build)
  endif() 
endif()
message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

if (WIN32)
  include(ExternalProject)
  
  string(FIND ${CMAKE_GENERATOR} Win64 is_win64)
  if (is_win64)
    set(url http://win32builder.gnome.org/gtk+-bundle_3.6.4-20131201_win64.zip)
  else()
    set(url http://win32builder.gnome.org/gtk+-bundle_3.6.4-20130921_win32.zip)
  endif()
  ExternalProject_Add(gtk3
    URL ${url}
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/gtk3
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND "")

  install(FILES gtk3/bin/libglib-2.0-0.dll
                gtk3/bin/libintl-8.dll
                gtk3/bin/libgthread-2.0-0.dll
                gtk3/bin/libiconv-2.dll
                gtk3/bin/pthreadGC2.dll
          DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

  # note: using gtk3 dir libpath (instead of installed path) because of .lib files
  # using gtk3 dir for include files
  # would be cleaner to install all of them.  will do it another day!
  file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/)

  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/glib-2.0.pc
      "Name: glib-2.0\n"
      "Description: GLIB as packaged by the GTK+ bundle\n"
      "Requires:\n"
      "Version: 2.34.3\n"
      "Libs: -L${PROJECT_SOURCE_DIR}/gtk3/lib -lglib-2.0 -lws2_32 -lwinmm\n"
      "Cflags: -I${PROJECT_SOURCE_DIR}/gtk3/include/glib-2.0 -I${PROJECT_SOURCE_DIR}/gtk3/lib/glib-2.0/include\n"
      )

  file(WRITE ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/gthread-2.0.pc
      "Name: gthread-2.0\n"
      "Description: GThread as packaged by the GTK+ bundle\n"
      "Requires: glib-2.0\n"
      "Version: 2.34.3\n"
      "Libs: -L${PROJECT_SOURCE_DIR}/gtk3/lib -lgthread-2.0\n"
      "Cflags:\n"
      )

  message(STATUS "Writing glib-2.0.pc and gthread-2.0.pc to ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/")
endif()
