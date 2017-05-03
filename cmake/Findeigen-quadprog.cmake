find_path(EIGEN_QUADPROG_INCLUDE_DIR NAMES
  eigen-quadprog/eigen_quadprog_api.h
  eigen-quadprog/QuadProg.h
)

find_library(EIGEN_QUADPROG NAMES eigen-quadprog HINTS /usr/local/lib)
set(EIGEN_QUADPROG_LIBRARY ${EIGEN_QUADPROG})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OOQP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(eigen-quadprog  DEFAULT_MSG
                                  EIGEN_QUADPROG_LIBRARY EIGEN_QUADPROG_INCLUDE_DIR)

mark_as_advanced(EIGEN_QUADPROG_INCLUDE_DIR EIGEN_QUADPROG_LIBRARY)
set(eigen-quadprog_LIBRARIES ${EIGEN_QUADPROG_LIBRARY})
set(eigen-quadprog_INCLUDE_DIRS ${EIGEN_QUADPROG_INCLUDE_DIR})
