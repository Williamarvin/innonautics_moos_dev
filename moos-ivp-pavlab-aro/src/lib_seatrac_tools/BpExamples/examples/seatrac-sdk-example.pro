QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Application.cpp \
    main.cpp \
    MainWindow.cpp \
	../src/BpClasses/conversion.cpp \
	../src/BpClasses/crc16.cpp \
	../src/BpClasses/crc32.cpp \
	../src/BpClasses/macros.c \
	../src/BpClasses/maths.cpp \
	../src/BpClasses/serialize.cpp \
	../src/BpClasses/serializebuffer.cpp \
	../src/BpSeatrac/seatrac_beacon.cpp \
	../src/BpSeatrac/seatrac_cmdmsg.cpp \
	../src/BpSeatrac/seatrac_cmdproc.cpp \
	../src/BpSeatrac/seatrac_diagmsg.cpp \
	../src/BpSeatrac/seatrac_utils.cpp

HEADERS += \
    Application.h \
    MainWindow.h \
	../src/BpClasses/common.h \
	../src/BpClasses/conversion.hpp \
	../src/BpClasses/crc16.hpp \
	../src/BpClasses/crc32.hpp \
	../src/BpClasses/events.hpp \
	../src/BpClasses/macros.h \
	../src/BpClasses/maths.cpp2 \
	../src/BpClasses/maths.hpp \
	../src/BpClasses/maths.hpp2 \
	../src/BpClasses/matrix.hpp \
	../src/BpClasses/serialize.hpp \
	../src/BpClasses/serializebuffer.hpp \
	../src/BpClasses/types.h \
	../src/BpSeatrac/seatrac_beacon.hpp \
	../src/BpSeatrac/seatrac_cmdmsg.hpp \
	../src/BpSeatrac/seatrac_cmdproc.hpp \
	../src/BpSeatrac/seatrac_diagmsg.hpp \
	../src/BpSeatrac/seatrac_types.h \
	../src/BpSeatrac/seatrac_utils.hpp

INCLUDEPATH += \
    ../src/BpClasses \
	../src/BpSeatrac

win32 {
    SOURCES += \
	    ../src/BpSystem/Win/mutex.cpp \
		../src/BpSystem/Win/ticks.cpp

    HEADERS += \
	    ../src/BpSystem/Win/mutex.hpp \
		../src/BpSystem/Win/ticks.hpp

    INCLUDEPATH += ../src/BpSystem/Win
}

unix {
    SOURCES += \
	    ../src/BpSystem/Linux/mutex.cpp \
		../src/BpSystem/Linux/ticks.cpp

    HEADERS += \
	    ../src/BpSystem/Linux/mutex.hpp \
		../src/BpSystem/Linux/ticks.hpp

    INCLUDEPATH += ../src/BpSystem/Linux
}

FORMS += \
    MainWindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
