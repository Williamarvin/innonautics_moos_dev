#--------------------------------------------------------
# The CMakeLists.txt for:                  lib_bhv_pmview
# Author(s):                                Mike Benjamin 
#--------------------------------------------------------
#                                          BHV_PModelView
#--------------------------------------------------------
ADD_LIBRARY(BHV_PModelView SHARED 
   BHV_PModelView.cpp)
TARGET_LINK_LIBRARIES(BHV_PModelView
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
)
