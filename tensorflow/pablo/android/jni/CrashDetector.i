/* File : CrashDetector.i */
%include "enums.swg"

%module CrashDetector_Wrapper

/* Anything in the following section is added verbatim to the .cxx wrapper file */
%{
#include "CrashDetector.hpp"
#include "Logger.hpp"

#include <android/log.h>

// Native Android logging implementation
void loggerFn(int level, std::string& tag, std::string& message) {
    switch (level) {
        case LOG_LEVEL_VERBOSE:
            __android_log_print(ANDROID_LOG_VERBOSE, tag.c_str(), "%s", message.c_str());
            break;
        case LOG_LEVEL_DEBUG:
            __android_log_print(ANDROID_LOG_DEBUG, tag.c_str(), "%s", message.c_str());
            break;
        case LOG_LEVEL_INFO:
            __android_log_print(ANDROID_LOG_INFO, tag.c_str(), "%s", message.c_str());
            break;
        case LOG_LEVEL_WARN:
            __android_log_print(ANDROID_LOG_WARN, tag.c_str(), "%s", message.c_str());
            break;
        case LOG_LEVEL_ERROR:
            __android_log_print(ANDROID_LOG_ERROR, tag.c_str(), "%s", message.c_str());
            break;
        default:
            break;
    }
}
%}

/* This is the list of headers to be wrapped */
/* For Java, it seems we need the file of interest and all files up the inheritance tree */
%include "CrashDetector.hpp"
