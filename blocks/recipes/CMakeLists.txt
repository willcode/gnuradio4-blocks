# Recipes are YAML subgraph definitions and hold no code. They are installed beside the block libraries, where an
# installed loader can find them. Each recipe also ships a generated typed C++ header in include/gnuradio-4.0/recipes/.
# gr4-recipe-gen writes the headers, and they are kept in the source tree. qa_Recipes diffs them against a fresh
# emission. The YAML stays the single source of truth.
install(
  FILES index.yaml
        AfskDemod.yaml
        BpskDemod.yaml
        CcsdsConcatenatedFrames.yaml
        CcsdsRsFrames.yaml
        BpskFrontEnd.yaml
        DbpskDemod.yaml
        FskDemod.yaml
        FskDemodAudio.yaml
        FskDemodDcBlock.yaml
        HdlcDeframe.yaml
        KissFileWrite.yaml
        KissFileRead.yaml
        KissServe.yaml
        KissStreamDecode.yaml
        NbfmDemod.yaml
        OfdmDemodulator.yaml
        OfdmModulator.yaml
        SampleClockOffset.yaml
        WbfmMonoDemod.yaml
  DESTINATION "${CMAKE_INSTALL_DATADIR}/gnuradio-4.0/recipes")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/recipes")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/recipes"
          DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")
endif()

# The generator reads the catalog and writes the typed headers. Run it by hand, or by name through ninja, after editing
# a recipe. Keep the regenerated headers in the source tree.
add_executable(gr4-recipe-gen tools/gr4-recipe-gen.cpp)
target_link_libraries(gr4-recipe-gen PRIVATE gnuradio-core gnuradio-blocklib-core)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
