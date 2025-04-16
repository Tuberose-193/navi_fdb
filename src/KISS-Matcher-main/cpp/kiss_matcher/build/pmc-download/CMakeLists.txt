# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(pmc-download NONE)

include(ExternalProject)
ExternalProject_Add(pmc-download
                    GIT_REPOSITORY;https://github.com/jingnanshi/pmc.git;GIT_TAG;master
                    SOURCE_DIR          "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/pmc-src"
                    BINARY_DIR          "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/pmc-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
