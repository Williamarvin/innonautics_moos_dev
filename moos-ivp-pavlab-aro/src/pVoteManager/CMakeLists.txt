#--------------------------------------------------------
# The CMakeLists.txt for:                       pVoteManager
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  VoteRecord.cpp
  VoteManager.cpp
  VoteManager_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pVoteManager ${SRC})

TARGET_LINK_LIBRARIES(pVoteManager
  ${MOOS_LIBRARIES}
  contacts
  apputil
  mbutil
  m
  pthread)

