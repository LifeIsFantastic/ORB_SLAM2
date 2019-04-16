set(SUBSYS_NAME visualization)
set(SUBSYS_DEPS ${PROJECT_NAME}_common)

if(BUILD_VISUALIZATION)
  set(INCS
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/visualizer_base.h"
  )

  set(IMPL_INCS)

  set(SRCS
    "src/visualizer_base.cpp"
  )

  set(LIB_NAME "${PROJECT_NAME}_${SUBSYS_NAME}")

  add_library(${LIB_NAME} SHARED
    ${SRCS}
  )

  target_link_libraries(${LIB_NAME}
    ${CUDA_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${SUBSYS_DEPS}
  )

endif(BUILD_VISUALIZATION)
