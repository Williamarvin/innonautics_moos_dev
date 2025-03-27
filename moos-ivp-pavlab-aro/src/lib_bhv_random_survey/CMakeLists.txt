#--------------------------------------------------------
# The CMakeLists.txt for:           lib_bhv_random_survey
# Author(s):                              Filip Stromstad
#--------------------------------------------------------
#                                        BHV_RandomSurvey
#--------------------------------------------------------
ADD_LIBRARY(BHV_RandomSurvey SHARED 
   BHV_RandomSurvey.cpp)
TARGET_LINK_LIBRARIES(BHV_RandomSurvey
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
)


