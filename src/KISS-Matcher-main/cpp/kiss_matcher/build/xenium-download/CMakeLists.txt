# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(xenium-download NONE)

include(ExternalProject)
ExternalProject_Add(xenium-download
                    GIT_REPOSITORY;https://github.com/mpoeter/xenium;GIT_TAG;7ee5ed18b858106fc6e0fc19305aebfebce1bdf4
                    SOURCE_DIR          "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-src"
                    BINARY_DIR          "/home/tuberose/ws_livox/src/KISS-Matcher-main/cpp/kiss_matcher/build/xenium-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
