//
//  CrashDetector.hpp
//  CrashSDK
//
//  Created by Brad Cordova on 6/29/16.
//  Copyright © 2016 TrueMotion. All rights reserved.
//

#ifndef CrashDetector_hpp
#define CrashDetector_hpp

#include <string>

// Client must provide an implementation of the following function for logging:
void loggerFn(int level, std::string& tag, std::string& message);

// Each algorithm may be turned off, on, or put in debug mode.
enum Mode {
    MODE_OFF,
    MODE_ON,
    MODE_DEBUG,
};

// The result of running:  there may be more work to do, and there may be a
// result to be fetched with get_xxx_result().
struct Result {
    bool more_work: 1;
    enum Type {
        NONE,            // No result at this time.
        CRASH_IMU_ONLY,  // An IMU-only crash was detected.
        CRASH_GPS        // A real crash was detected.
    } type: 31;
    
    Result(): more_work(false), type(NONE) {}
};

// Result type for CRASH_IMU_ONLY and CRASH_GPS
struct CrashResult {
    double time;
    double magnitude;
};

class CrashDetector {
protected:
    CrashDetector(); // constructor is not public -- use create().
    
public:
    static CrashDetector & create();
    
    // Push data into engine but do no actual work.
    bool add_accelerometer_sample(double time, double magnitude);
    bool add_gps_sample(double time, double magnitude);
    
    // Do a "small" amount of data processing.
    // Result.type indicates whether a result was produced and its type.
    // If result.more_work is false, don't bother calling again until more data is pushed in.
    Result process_unit();
    
    // Process data until either a crash is detected or there is no more work to do.
    // The return value is as for process_one().
    Result process_to_result();
    
    void set_crash_mode(Mode mode);
    void get_crash_result(CrashResult& result);
};

// Temporary hack, put data into a TensorFlow crash model and returns the result.

// filename is for the model file as it appears on the phone
// inputname and outputname are the names of the nodes in the model
// inputlength is the length of the input vector (of floats, I mean doubles)
void loadModel(const char* filename, const char* inputname, const char* outputname, unsigned inputlength);

// client provides inputlength floats, I just set them all to 0.5
double exercise_tf(double input[]);

#endif /* CrashDetector_hpp */
