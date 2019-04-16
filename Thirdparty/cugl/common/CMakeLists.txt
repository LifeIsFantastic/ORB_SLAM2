set(SUBSYS_NAME common)
set(SUBSYS_DEPS)

if(BUILD_COMMON)
  set(INCS
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/helper_cuda.h"
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/helper_cuda_gl.h"
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/helper_gl.h"
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/helper_string.h"
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/containers/device_array_handle.h"
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/containers/device_array.h"
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/containers/texture_bridge.h"
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/geometry/affine3d.h"
  )

  set(IMPL_INCS
    "include/${PROJECT_NAME}/${SUBSYS_NAME}/containers/impl/device_array.hpp"
  )

  set(SRCS
    "src/containers/texture_bridge.cpp"
  )

  set(LIB_NAME "${PROJECT_NAME}_${SUBSYS_NAME}")

  CUDA_ADD_LIBRARY(${LIB_NAME} SHARED
    ${SRCS}
  )

  target_link_libraries(${LIB_NAME}
    ${CUDA_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${SUBSYS_DEPS}
  )

endif(BUILD_COMMON)
