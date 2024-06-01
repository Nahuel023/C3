QT       += core gui serialport network charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    debugger.cpp \
    main.cpp \
    maps.cpp \
    qform.cpp \
    settingsdialog.cpp

HEADERS += \
    debugger.h \
    maps.h \
    qform.h \
    settingsdialog.h

FORMS += \
    debugger.ui \
    maps.ui \
    qform.ui \
    settingsdialog.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
