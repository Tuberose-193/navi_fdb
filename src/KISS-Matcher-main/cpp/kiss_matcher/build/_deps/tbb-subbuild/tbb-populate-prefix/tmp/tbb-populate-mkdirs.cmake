# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/tbb-src"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/tbb-build"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/tbb-subbuild/tbb-populate-prefix"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/tbb-subbuild/tbb-populate-prefix/tmp"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/tbb-subbuild/tbb-populate-prefix/src/tbb-populate-stamp"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/tbb-subbuild/tbb-populate-prefix/src"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/tbb-subbuild/tbb-populate-prefix/src/tbb-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/tbb-subbuild/tbb-populate-prefix/src/tbb-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/tbb-subbuild/tbb-populate-prefix/src/tbb-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
