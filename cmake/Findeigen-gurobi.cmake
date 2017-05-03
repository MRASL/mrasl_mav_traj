find_path(EIGEN_GUROBI_INCLUDE_DIR NAMES
  eigen-gurobi/eigen_gurobi_api.h
  eigen-gurobi/Gurobi.h
)

find_library(EIGEN_GUROBI NAMES eigen-gurobi HINTS /usr/local/lib)
set(EIGEN_GUROBI_LIBRARY ${EIGEN_GUROBI})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OOQP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(eigen-gurobi  DEFAULT_MSG
                                  EIGEN_GUROBI_LIBRARY EIGEN_GUROBI_INCLUDE_DIR)

mark_as_advanced(EIGEN_GUROBI_INCLUDE_DIR EIGEN_GUROBI_LIBRARY)
set(eigen-gurobi_LIBRARIES ${EIGEN_GUROBI_LIBRARY})
set(eigen-gurobi_INCLUDE_DIRS ${EIGEN_GUROBI_INCLUDE_DIR})
