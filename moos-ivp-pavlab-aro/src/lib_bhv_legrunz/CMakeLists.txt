#--------------------------------------------------------
# The CMakeLists.txt for:                 lib_bhv_legrunz
# Author(s):                                Mike Benjamin 
#--------------------------------------------------------
#                                             BHV_LegRunZ
#--------------------------------------------------------
ADD_LIBRARY(BHV_LegRunZ SHARED 
   BHV_LegRunZ.cpp
   RingMaster.cpp
   SpdModulator.cpp
)
   
TARGET_LINK_LIBRARIES(BHV_LegRunZ
   helmivp
   legrun
   behaviors 
   turngeo
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
)

ADD_LIBRARY(BHV_LegRunCoord SHARED 
   BHV_LegRunCoord.cpp
   RingMaster.cpp
   SpdModulator.cpp
)
   
TARGET_LINK_LIBRARIES(BHV_LegRunCoord
   helmivp
   behaviors-marine
   behaviors 
   turngeo
   bhvutil 
   ivpbuild 
   logic 
   ivpcore 
   mbutil 
   geometry 
)
