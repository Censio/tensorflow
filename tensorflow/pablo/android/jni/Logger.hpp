//
//  Logger.hpp
//  Logger
//
//  Created by Steven J. Clark on 8/4/16.
//  Copyright © 2016 Steven J. Clark. All rights reserved.
//

#ifndef Logger_hpp
#define Logger_hpp

#include <stdio.h>
#include <string>
#include "Platform.hpp"

// Log levels; more verbose is a higher number.
#define LOG_LEVEL_ERROR   0
#define LOG_LEVEL_WARN    1
#define LOG_LEVEL_INFO    2
#define LOG_LEVEL_DEBUG   3
#define LOG_LEVEL_VERBOSE 4

// Now define the macros that are actually used.

#define LOG_ERROR Logger(LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__)
#define LOG_WARN Logger(LOG_LEVEL_WARN, __FILE__, __LINE__, __FUNCTION__)
#define LOG_INFO Logger(LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__)
#define LOG_VERBOSE Logger(LOG_LEVEL_VERBOSE, __FILE__, __LINE__, __FUNCTION__)

// LOG_DEBUG is always logged if built for debug and never for production.
#ifdef DEBUG
#define LOG_DEBUG Logger(LOG_LEVEL_DEBUG, __FILE__, __LINE__, __FUNCTION__)
#else
#define LOG_DEBUG DummyLogger()
#endif

// Client must provide an implementation of this function.
void loggerFn(int level, std::string& tag, std::string& message);

class Logger {
    int level;
    std::string tag;
    std::string message;
    Logger() { }
public:
    Logger & operator<<(const char *cstr) { message += cstr; return *this; }
    Logger & operator<<(const std::string &str) { message += str; return *this; }
    template<typename T>
    Logger & operator<<(T arg) { message += platform::to_string(arg); return *this; }
    
    Logger(int level_, const char *file, int line, const char *function)
    : level(level_)
    {
        const char* filetail = strrchr(file, '/');
        if (filetail == NULL) filetail = file;
        else filetail++;  // don't include the '/' itself
        tag = filetail;
        tag += ", line ";
        tag += platform::to_string(line);
        tag += " (";
        tag += function;
        tag += ")";
    }
    ~Logger() { loggerFn(level, tag, message); }
};

class DummyLogger {
public:
    template<typename T>
    DummyLogger & operator<<(T arg) { return *this; }
};

#endif /* Logger_hpp */
