# The LDPC and Polar families are AFF3CT's, and AFF3CT is an option rather than a dependency: the module builds whole
# without it and the four blocks that need it are registered only when the gate is on. That is the network module's
# libzmq split, with one addition — the AFF3CT headers reach no installed header of ours, so the wrap is a compiled
# translation unit behind a pointer and the block headers name the tree's types only.
option(GR4_ENABLE_AFF3CT "build the LDPC and Polar blocks against a pinned AFF3CT release" OFF)

set(GR4_FEC_HAVE_AFF3CT FALSE)
if(GR4_ENABLE_AFF3CT)
  # Threads first: the exported aff3ct config names Threads::Threads without finding it.
  find_package(Threads REQUIRED)
  find_package(aff3ct CONFIG REQUIRED)
  set(GR4_FEC_HAVE_AFF3CT TRUE)
  # The release ships its parity-check matrices as data beside the package, and the package does not name where. The
  # directory is derived from where the config was found so that the 'standard' setting can resolve a name, and it is a
  # cache variable so a different layout is one -D away.
  get_filename_component(GR4_AFF3CT_PREFIX "${aff3ct_DIR}/../../.." ABSOLUTE)
  set(GR4_AFF3CT_LDPC_CONF_DIR "${GR4_AFF3CT_PREFIX}/share/aff3ct-${aff3ct_VERSION}/conf/dec/LDPC"
      CACHE PATH "directory holding the AFF3CT release's LDPC parity-check matrices")
  set(GR4_AFF3CT_POLAR_CONF_DIR "${GR4_AFF3CT_PREFIX}/share/aff3ct-${aff3ct_VERSION}/conf/cde/awgn_polar_codes"
      CACHE PATH "directory holding the AFF3CT release's polar reliability sequences")
  foreach(dir "${GR4_AFF3CT_LDPC_CONF_DIR}" "${GR4_AFF3CT_POLAR_CONF_DIR}")
    if(NOT IS_DIRECTORY "${dir}")
      message(STATUS "GR4_ENABLE_AFF3CT is ON but ${dir} is not a directory — the settings that name one of the"
                     " release's own constructions will refuse, and the explicit routes are the only ones")
    endif()
  endforeach()
endif()

add_library(gr-fec INTERFACE)
add_library(gnuradio4::gr-fec ALIAS gr-fec)
target_link_libraries(gr-fec INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-fec INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                            $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-fec EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/fec"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

set(GrFecBlocks_HDRS
    include/gnuradio-4.0/fec/ConvBlocks.hpp
    include/gnuradio-4.0/fec/FecBlocks.hpp
    include/gnuradio-4.0/fec/InterleaveBlocks.hpp
    include/gnuradio-4.0/fec/PunctureBlocks.hpp
    include/gnuradio-4.0/fec/RsBlocks.hpp)

if(GR4_FEC_HAVE_AFF3CT)
  # The wall: the only translation unit in this tree that sees an AFF3CT header. It is position independent because the
  # block library it is linked into is a shared object.
  add_library(gr-fec-aff3ct STATIC src/Aff3ctWall.cpp)
  set_target_properties(gr-fec-aff3ct PROPERTIES POSITION_INDEPENDENT_CODE ON)
  # gnuradio-options carries the tree's warning set and is not itself installed, so it is a build-time link only; the
  # AFF3CT headers arrive through an imported target and are therefore system includes, which is what keeps that warning
  # set usable over them.
  target_link_libraries(gr-fec-aff3ct PUBLIC gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm
                        PRIVATE $<BUILD_INTERFACE:gnuradio-options> aff3ct::aff3ct-shared-lib Threads::Threads)
  target_include_directories(gr-fec-aff3ct PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                                  $<INSTALL_INTERFACE:include/>)
  target_compile_definitions(gr-fec-aff3ct PRIVATE GR4_AFF3CT_LDPC_CONF_DIR="${GR4_AFF3CT_LDPC_CONF_DIR}"
                             GR4_AFF3CT_POLAR_CONF_DIR="${GR4_AFF3CT_POLAR_CONF_DIR}")
  install(TARGETS gr-fec-aff3ct EXPORT gnuradio4Targets)
  target_link_libraries(gr-fec INTERFACE gr-fec-aff3ct)
else()
endif()

gr_add_block_library(
  GrFecBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  ${GrFecBlocks_HDRS}
  LINK_LIBRARIES
  gr-fec)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(bench)
