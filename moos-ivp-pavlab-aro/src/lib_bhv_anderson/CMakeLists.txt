#--------------------------------------------------------
# The CMakeLists.txt for:                lib_bhv_anderson
# Author(s):                                Mike Benjamin 
#--------------------------------------------------------
#                                        BHV_AndersonTurn
#--------------------------------------------------------
ADD_LIBRARY(BHV_AndersonTurn SHARED 
   BHV_AndersonTurn.cpp)
TARGET_LINK_LIBRARIES(BHV_AndersonTurn
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
)
