
# Copyright (c) 2011-2015 Stefan Eilemann <eile@eyescale.ch>
#                    2015 Enrique G. Paredes <egparedes@ifi.uzh.ch>

set(TRIPLY_PUBLIC_HEADERS
  api.h
  ply.h
  typedefs.h
  modelTreeBase.h
  modelTreeData.h
  modelTreeDist.h
  modelTreeLeaf.h
  modelTreeNode.h
  modelTreeRoot.h
  treeRenderState.h
  vertexData.h
  pagedTreeData.h)

set(TRIPLY_SOURCES
  plyfile.cpp
  modelTreeBase.cpp
  modelTreeDist.cpp
  modelTreeLeaf.cpp
  modelTreeNode.cpp
  modelTreeRoot.cpp
  treeRenderState.cpp
  vertexData.cpp
  pagedTreeData.cpp)


set(TRIPLY_LINK_LIBRARIES
  ${COLLAGE_LIBRARIES} ${EQ_GL_LIBRARIES} ${GLEW_LIBRARY})
set(PROJECT_INCLUDE_NAME triply)

list(APPEND CPPCHECK_EXTRA_ARGS
  --suppress=*:${CMAKE_CURRENT_SOURCE_DIR}/plyfile.cpp)

add_definitions(-DPLYLIB_SHARED)
common_library(triply)
