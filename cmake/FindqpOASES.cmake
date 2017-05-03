find_path(QPOASES_INCLUDE_DIR NAMES
  qpOASES/QProblem.hpp
  HINTS
  /usr/local/include
)

find_library(QPOASES NAMES qpOASES HINTS /usr/local/lib)
set(QPOASES_LIBRARY ${QPOASES})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OOQP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(qpOASES  DEFAULT_MSG
QPOASES_LIBRARY QPOASES_INCLUDE_DIR)

mark_as_advanced(QPOASES_INCLUDE_DIR QPOASES_LIBRARY)
set(qpOASES_LIBRARIES ${QPOASES_LIBRARY})
set(qpOASES_INCLUDE_DIRS ${QPOASES_INCLUDE_DIR})
