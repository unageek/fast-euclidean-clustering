set(TARGET ec)

add_executable(${TARGET} ec/main.cc)

if (UNIX)
  target_compile_options(${TARGET} PRIVATE -Wall -Wextra -Werror)
elseif (MSVC)
  target_compile_options(${TARGET} PRIVATE /W4 /WX)
endif ()

target_link_libraries(${TARGET} PRIVATE
  ${PCL_LIBRARIES}
)



set(TARGET fec)

add_executable(${TARGET} fec/main.cc)

if (UNIX)
  target_compile_options(${TARGET} PRIVATE -Wall -Wextra -Werror)
elseif (MSVC)
  target_compile_options(${TARGET} PRIVATE /W4 /WX)
endif ()

target_include_directories(${TARGET} PRIVATE
  ${PROJECT_SOURCE_DIR}
)

target_link_libraries(${TARGET} PRIVATE
  ${PCL_LIBRARIES}
)
