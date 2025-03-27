#--------------------------------------------------------
#                                      BHV_Shell
#--------------------------------------------------------
ADD_LIBRARY(BHV_Shell SHARED 
   BHV_Shell.cpp)
TARGET_LINK_LIBRARIES(BHV_Shell
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
   ${SYSTEM_LIBS} )
