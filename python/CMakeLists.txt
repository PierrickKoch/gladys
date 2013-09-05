set(PYTHON_SITE_PACKAGES "" CACHE PATH "Python site-packages")
if (NOT PYTHON_SITE_PACKAGES)
    execute_process(COMMAND "${PYTHON_EXECUTABLE}" -c "from distutils.sysconfig import get_python_lib; print(get_python_lib(1,0,''))"
        OUTPUT_VARIABLE
        PYTHON_SITE_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(PYTHON_SITE_PACKAGES ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES})
endif(NOT PYTHON_SITE_PACKAGES)

add_library( gladys_python SHARED gladys_python.cpp )
target_link_libraries( gladys_python gladys )
target_link_libraries( gladys_python ${Boost_LIBRARIES} )
target_link_libraries( gladys_python ${PYTHON_LIBRARIES} )
target_link_libraries( gladys_python ${GDAL_LIBRARIES} )

install(TARGETS gladys_python DESTINATION ${PYTHON_SITE_PACKAGES})
install(FILES   gladys.py     DESTINATION ${PYTHON_SITE_PACKAGES})
install(PROGRAMS gladysplay   DESTINATION ${CMAKE_INSTALL_BINDIR})
