#ifndef MODULES_RTT_JLINK_RTT_H
#define MODULES_RTT_JLINK_RTT_H

#define RTT_DEBUG

#ifdef RTT_DEBUG
/****** Enable RTT Debugging ******/

#include "SEGGER_RTT.h"

#define RTT_PKG_VERSION V620a

#define JLinkRTTSetup() \
    SEGGER_RTT_Setup()
#define JLinkRTTPrintf(chan, fmt, ...) \
    SEGGER_RTT_printf(chan, fmt, __VA_ARGS__)
#define JLinkWriteString(chan, str) \
    SEGGER_RTT_WriteString(chan, str)

#else
/****** Disable RTT Debugging ******/

#define JLinkRTTSetup()
#define JLinkRTTPrintf(chan, fmt, ...) 
#define JLinkWriteString(chan, str) 

#endif

/****** Public API functions ******/
/*

void JLinkRTTSetup();
void JLinkRTTPrintf();
void JLinkWriteString();

*/




#endif /* MODULES_RTT_JLINK_RTT_H */