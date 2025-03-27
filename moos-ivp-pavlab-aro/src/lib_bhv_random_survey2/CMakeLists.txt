#--------------------------------------------------------
# The CMakeLists.txt for:          lib_bhv_random_survey2
# Author(s):                                    T. Paine
#--------------------------------------------------------
#                                      BHV_Random_Survey2
#--------------------------------------------------------
ADD_LIBRARY(BHV_Random_Survey2 SHARED 
   BHV_Scout.cpp)
TARGET_LINK_LIBRARIES(BHV_Random_Survey2
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
)


