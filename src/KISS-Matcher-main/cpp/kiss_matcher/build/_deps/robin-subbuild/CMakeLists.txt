# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.24.0)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(robin-populate NONE)



include(ExternalProject)
ExternalProject_Add(robin-populate
                     "UPDATE_DISCONNECTED" "False" "DOWNLOAD_EXTRACT_TIMESTAMP" "FALSE" "URL" "https://github.com/MIT-SPARK/ROBIN/archive/refs/tags/v.1.2.3.tar.gz"
                    SOURCE_DIR          "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/robin-src"
                    BINARY_DIR          "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/_deps/robin-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
                    USES_TERMINAL_PATCH     YES
)


