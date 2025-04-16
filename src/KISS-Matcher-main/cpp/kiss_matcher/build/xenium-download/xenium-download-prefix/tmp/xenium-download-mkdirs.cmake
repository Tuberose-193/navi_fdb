# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-src"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-build"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-download/xenium-download-prefix"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-download/xenium-download-prefix/tmp"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-download/xenium-download-prefix/src/xenium-download-stamp"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-download/xenium-download-prefix/src"
  "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-download/xenium-download-prefix/src/xenium-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-download/xenium-download-prefix/src/xenium-download-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-download/xenium-download-prefix/src/xenium-download-stamp${cfgdir}") # cfgdir has leading slash
endif()
