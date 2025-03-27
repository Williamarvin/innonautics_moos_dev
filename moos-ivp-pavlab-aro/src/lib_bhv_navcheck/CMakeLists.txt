#--------------------------------------------------------
# The CMakeLists.txt for:                lib_bhv_navcheck
# Author(s):                                Mike Benjamin 
#--------------------------------------------------------
#                                            BHV_NavCheck
#--------------------------------------------------------
ADD_LIBRARY(BHV_NavCheck SHARED 
   BHV_NavCheck.cpp)

 TARGET_LINK_LIBRARIES(BHV_NavCheck
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
)
