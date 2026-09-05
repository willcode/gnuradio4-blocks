set(GR4_BLOCK_STATIC_LIBRARY "")
if(EMSCRIPTEN)
  set(GR4_BLOCK_STATIC_LIBRARY MAKE_STATIC_LIBRARY)
endif()

add_subdirectory(common)
add_subdirectory(testing)
add_subdirectory(basic)
add_subdirectory(analog)
add_subdirectory(channel)
add_subdirectory(digital)
add_subdirectory(electrical)
add_subdirectory(fec)
add_subdirectory(fileio)
add_subdirectory(filter)
add_subdirectory(fourier)
add_subdirectory(http)
add_subdirectory(math)
add_subdirectory(sync)
add_subdirectory(timing)

# optional families, added before libs so the registry audit there can see their shared libraries when they are built.
# The audit detects a family from its generated header and links the library the family's target provides, so a family
# added after libs is one the audit calls into and cannot link against.
if(GR4_ENABLE_NETWORK)
  add_subdirectory(network)
endif()

if(GR4_ENABLE_AUDIO)
  add_subdirectory(audio)
endif()

if(GR4_ENABLE_SDR)
  add_subdirectory(sdr)
endif()
