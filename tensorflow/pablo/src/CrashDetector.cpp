//
//  CrashDetector.cpp
//  CrashSDK
//
//  Created by Brad Cordova on 6/29/16.
//  Copyright © 2016 TrueMotion. All rights reserved.
//

#include "CrashDetector.hpp"
//#include <stdio.h>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <list>
#include <cmath>
#include <numeric>
#include <string>
#include "spline.h"
#include "Logger.hpp"
#include "Platform.hpp"

const float ACCELERATION_DUE_TO_GRAVITY = 9.80665;
const double preprocessing_accelerometer_threshold = 2.1 * ACCELERATION_DUE_TO_GRAVITY; // meter / second / second
const double accelerometer_crash_threshold = 2.5 * ACCELERATION_DUE_TO_GRAVITY; // meter / second / second
const double gps_speed_threshold = 2.7; // meter / second

const float imu_half_window_time = 1.0; // seconds
const float minimum_time_between_crashes = 20.0; // seconds
const float gps_window_time = 30.0; // seconds
const float gps_delay_time = 90.0; //seconds

const int minimumim_number_gps_points_in_window = 0;
//const int minimumim_number_imu_points_in_window = 0;

const int imu_sampling_frequency = 9; // Hz
const int gps_sampling_frequency = 1; // Hz

//const double imu_window_comparison_precision = 0.0/imu_sampling_frequency; // second
const double gps_window_comparison_precision = 1.0/gps_sampling_frequency; // second

const int half_number_imu_points_in_window = round(imu_half_window_time);
const int number_gps_points_in_window = round(gps_window_time);

struct CrashDetectorImpl: CrashDetector {
    //
    // Data Processing stuff
    //
    
    // Input data buffers
    std::deque<std::pair<double, double>> accelerometer_buffer;
    std::deque<std::pair<double, double>> gps_buffer;
    
    // Intermediate data work queues
    std::queue<std::pair<double, double>> over_threshold_queue;
    std::queue<std::vector<std::pair<double, double>>> window_processing_queue;
    std::queue<std::pair<double, double>> potiential_crash_queue;
    
    // Remember most recent imu sample that's been processed by process_imu_over_threshold()
    double previous_thresholded_imu;
    
    // Remember the most recent GPS crash to avoid duplicates.
    bool crash_detected;
    double time_of_crash;
    double magnitude_of_crash;
    
    // Stage 1: look at IMU data, find candidate crashes, resample, and classify.
    std::vector<std::pair<double, double>> calculate_variable_window_vector(const std::vector<std::pair<double, double>> & fixed_window_vector);
    std::vector<std::pair<double, double>> calculate_fixed_window_vector(const std::pair<double, double> & window_center_point, bool sort_buffer);
    
    Result::Type run_imu_crash_classifier(const std::vector<std::pair<double, double>> & variable_window_vector);
    
    void process_imu_over_threshold();
    void process_over_threshold_queue();
    Result process_fixed_window_queue();
    // Encapsulate the functions above into one.
    Result process_into_potential_crash();
    
    // Stage 2: use GPS data to confirm or reject potential crashes.
    bool low_speed_in_gps_window();
    Result process_potential_crash_queue();
    
    //
    // Bookkeeping and other stuff
    //
    
    // input counters
    unsigned accelerometer_samples;
    unsigned gps_samples;
    
    // error counters
    unsigned accelerometer_out_of_order;
    unsigned gps_out_of_order;
    
    // Hold the most recent result until the client fetches it.
    CrashResult crash_result;  // CRASH_IMU_ONLY and CRASH_GPS
    
    CrashDetectorImpl()
    : previous_thresholded_imu(0.0)
    , crash_detected(false)
    , accelerometer_samples(0)
    , gps_samples(0)
    , accelerometer_out_of_order(0)
    , gps_out_of_order(0)
    { }
    
    void get_crash_result(CrashResult& cr);
};

//
// Data Processing Functions
//
// Order: helper functions, processing, control flow, and entering data samples.

#pragma mark helper functions

bool compare_pairs_by_second(const std::pair<double, double> & lhs, const std::pair<double, double> & rhs){
    return lhs.second < rhs.second;
}

bool compare_pairs_by_first(const std::pair<double, double> & lhs, const std::pair<double, double> & rhs){
    return lhs.first < rhs.first;
}

double get_first_element( const std::pair<double, double> & p){
    return p.first;
}

double get_second_element( const std::pair<double, double> & p){
    return p.second;
}

#pragma mark signal processing and variable window calculation

std::vector<std::pair<double, double>> CrashDetectorImpl::calculate_variable_window_vector(const std::vector<std::pair<double, double>> & fixed_window_vector){
    return fixed_window_vector;
}

std::vector<std::pair<double, double>> CrashDetectorImpl::calculate_fixed_window_vector(const std::pair<double, double> & window_center_point, bool sort_buffer=true){
    if (sort_buffer) {
        std::sort(accelerometer_buffer.begin(), accelerometer_buffer.end(), compare_pairs_by_first);
    }
    
    // interpolate accelerometer buffer using cubic spline interpolation
    std::vector<double> accelerometer_buffer_x;
    std::transform(accelerometer_buffer.begin(), accelerometer_buffer.end(), std::back_inserter(accelerometer_buffer_x), get_first_element);
    
    std::vector<double> accelerometer_buffer_y;
    std::transform(accelerometer_buffer.begin(), accelerometer_buffer.end(), std::back_inserter(accelerometer_buffer_y), get_second_element);
    
    tk::spline accelerometer_buffer_spline;
    accelerometer_buffer_spline.set_points(accelerometer_buffer_x, accelerometer_buffer_y);    // it's required that accelerometer_buffer_x is sorted
    
    // resample accelerometer buffer to correct frequency (imu_sampling_frequency)
    std::vector<std::pair<double, double>> fixed_window_vector;
    
    double window_start = window_center_point.first - imu_half_window_time;
    double window_end = window_center_point.first + imu_half_window_time;
    
    double t = window_start;
    while(t<=window_end) {
        fixed_window_vector.push_back(std::make_pair(t, accelerometer_buffer_spline(t)));
        t+=(1.0/imu_sampling_frequency);
    }
 
    return fixed_window_vector;
}

#pragma mark crash detection

Result::Type CrashDetectorImpl::run_imu_crash_classifier(const std::vector<std::pair<double, double>> & variable_window_vector){
    Result::Type result_type = Result::NONE;
//    double average = std::accumulate(variable_window_vector.begin(), variable_window_vector.end(), 0.0) / variable_window_vector.size();
    
    std::pair<double, double> max = *std::max_element(variable_window_vector.begin(), variable_window_vector.end(), compare_pairs_by_second);
    
#if 1
    LOG_VERBOSE << "max of variable window " << platform::to_string(max.first) << ", " << platform::to_string(max.second);
#endif
    
    if (max.second > accelerometer_crash_threshold) {
        LOG_VERBOSE << "there was a possible crash at t = " + platform::to_string(max.first) + ", and mag = " + platform::to_string(max.second) + "g's";
        
        result_type = Result::CRASH_IMU_ONLY;
        crash_result.time = max.first;
        crash_result.magnitude = max.second;

        // add to the queue of potiential crashes that need to be verified by GPS post processing
        potiential_crash_queue.push(variable_window_vector.back());
    }
    else{
        LOG_VERBOSE << "no crash no problem";
    }
    return result_type;
}

bool CrashDetectorImpl::low_speed_in_gps_window(){
    unsigned n_points_over_threshold = 0;
    
    for (std::pair<double, double> p : gps_buffer) {
        if (p.second > gps_speed_threshold) {
           LOG_VERBOSE << "gps_speed = " << platform::to_string(p.second) << " at t = " << platform::to_string(p.first);
            n_points_over_threshold += 1;
        }
        else{
            n_points_over_threshold = 0;
        }
        
        if (n_points_over_threshold >= 5) {
            return false;
        }
    }
    return true;
}

// Process the oldest element of the potential crash queue if we can.
// If it's a crash, fill in result.
// Return true if we may be able to process another element without
// getting more data first.
Result CrashDetectorImpl::process_potential_crash_queue() {
    Result result_value;
    
    // check if we have any points to process
    if (potiential_crash_queue.empty()) {
        return result_value;
    }
    
    // check if we have enough GPS points to verify
    if (gps_buffer.size() < minimumim_number_gps_points_in_window) {
        return result_value;
    }
    
    // potiential_crash_queue.front().first has the lowest (oldest) timestamp
    // gps_buffer.front().first has the lowest (oldest) timestamp
    
    // check if the GPS buffer covers enough time span
    if (gps_buffer.back().first - gps_buffer.front().first < gps_window_time) {
        return result_value;
    }

    // check if the oldest point in the gps buffer is at least gps_delay_time after the potential crash
    if ((potiential_crash_queue.front().first + gps_delay_time) > gps_buffer.front().first) {
        LOG_VERBOSE << "gps window has not reached the desired offset";
        return result_value;
    }
    
    // check if there has been enough time has passed since last crash happened
    // so we can protect against duplicate crashe reporting
    if (crash_detected && potiential_crash_queue.front().first < (time_of_crash + minimum_time_between_crashes)) {
        LOG_VERBOSE << "not enough time has elapsed since last crash";
        potiential_crash_queue.pop();
        return result_value;
    }

    // At this point we know we're processing one element from potential_crash_queue,
    // and there may be more processable elements to do.
    result_value.more_work = true;
    
    // check if 3 consecutive points over speed threshold
    if (low_speed_in_gps_window()) {
        crash_detected = true;
        time_of_crash = potiential_crash_queue.front().first;
        magnitude_of_crash = potiential_crash_queue.front().second;
        result_value.type = Result::CRASH_GPS;
        crash_result.time = time_of_crash;
        crash_result.magnitude = magnitude_of_crash;
    } else {
        LOG_VERBOSE << "no crash, due to GPS speed post processing";
    }
    potiential_crash_queue.pop();

    return result_value;
}

#pragma mark data queueing

void CrashDetectorImpl::process_imu_over_threshold() {
    // Look at all acceleration samples newer than previous_thresholded_imu.
    if (accelerometer_buffer.empty()) return;
    // return now if we've already processed the oldest sample
    if (accelerometer_buffer.back().first <= previous_thresholded_imu) return;
    
    // Now we know there's at least one sample to process.  Start with the newest:
    auto acc_it = accelerometer_buffer.end();
    // Walk backwards until we hit one that's older (or the beginning of the buffer)
    do {
        acc_it--;
    } while (acc_it->first > previous_thresholded_imu && acc_it > accelerometer_buffer.begin());
    // Now take one step forwards to get the first one newer.
    // (Works even at beginning and end of buffer.)
    if (acc_it->first <= previous_thresholded_imu) acc_it++;
    // Loop forward through to the end of the buffer.
    while (acc_it != accelerometer_buffer.end()) {
        std::pair<double, double> data_point = *acc_it;  // this is not the problem
        //        std::pair<double, double> data_point(acc_it->first, acc_it->second);
        if(data_point.second > preprocessing_accelerometer_threshold){
            over_threshold_queue.push(data_point);
#if 0
            LOG_VERBOSE << "point {" << platform::to_string(data_point.first) << ", " << platform::to_string(data_point.second) << "} add to threshold queue");
#endif
        }
        previous_thresholded_imu = data_point.first;
        acc_it++;
    }
}

void CrashDetectorImpl::process_over_threshold_queue() {
    LOG_VERBOSE << "processing over threshold queue";
    while (!over_threshold_queue.empty()) {
        std::pair<double, double> data_point = over_threshold_queue.front();
        
        // The "window" is centered on the data point.
        // First, has the accelerometer buffer passed us by?  Is the oldest point in the
        // acc. buffer newer than the oldest point we need?
        if ((data_point.first - imu_half_window_time) < accelerometer_buffer.front().first) {
            LOG_VERBOSE << "point is now outside of the accelerometer buffer";
            over_threshold_queue.pop();
            continue;  // we may be able to process the next data point
        }
        
        // Now check whether there is recent enough data.  Is the newest point in the acc.
        // buffer older than the newest point that we need?
        if ((data_point.first + imu_half_window_time) > accelerometer_buffer.back().first) {
            break;  // no work to do until we get more data
        }
        
        // calculate fixed window and add to the window processing queue
        std::vector<std::pair<double, double>> fixed_window = calculate_fixed_window_vector(data_point);
        
        window_processing_queue.push(fixed_window);
        
        LOG_VERBOSE << "window added to window processing queue";
        
        // remove processed point from over threshold queue
        over_threshold_queue.pop();
        
        // continue to loop in the unlikely case we can process the next one now too
    }
}

Result CrashDetectorImpl::process_fixed_window_queue(){
    Result result_value;
    if (window_processing_queue.empty()) return result_value;
    
    std::vector<std::pair<double, double>> fixed_window_vector = window_processing_queue.front();
    
#if 0
    {
        auto log = LOG_VERBOSE;
        log << "fixed window vector: { ";
        for (std::pair<double, double> & v : fixed_window_vector) {
            log << "{" << platform::to_string(v.first) << ", " << platform::to_string(v.second) << "}, ";
        }
        log << "}\n ------------- \n";
    }
#endif
    
    // create variable length vector
    std::vector<std::pair<double, double>> variable_window_vector = calculate_variable_window_vector(fixed_window_vector);
    
    {
        auto log = LOG_VERBOSE;
        log << "variable window vector: { ";
        for (std::pair<double, double> & v : variable_window_vector){
            log << "{" + platform::to_string(v.first) << ", " << platform::to_string(v.second) << "}, ";
        }
        log << "}\n ------------- \n";
    }
    
    // classify if crash or not
    result_value.type = run_imu_crash_classifier(variable_window_vector);
    
    // remove used window from window queue
    window_processing_queue.pop();
    
    result_value.more_work = !window_processing_queue.empty();
    return result_value;
}

Result CrashDetectorImpl::process_into_potential_crash() {
    // process all available data since previous call
    process_imu_over_threshold();
    
    // process all available data now
    process_over_threshold_queue();
    
    // This runs the ML classifier on at most one potential crash.
    return process_fixed_window_queue();
}

Result CrashDetector::process_unit() {
    // The following is safe only because no pointers to CrashDetector
    // exist that do not actually point to CrashDetectorImpl.
    CrashDetectorImpl *impl = static_cast<CrashDetectorImpl*>(this);
    Result result_value;
    result_value = impl->process_into_potential_crash();
    if (result_value.more_work || result_value.type != Result::NONE) return result_value;
    
    return impl->process_potential_crash_queue();
}

Result CrashDetector::process_to_result() {
    Result result_value;
    result_value.more_work = true;
    while (result_value.more_work && result_value.type == Result::NONE) {
        result_value = process_unit();
    }
    return result_value;
}

bool CrashDetector::add_accelerometer_sample(double time, double magnitude) {
    CrashDetectorImpl *impl = static_cast<CrashDetectorImpl*>(this);
    if (impl->accelerometer_buffer.empty() || time > impl->accelerometer_buffer.back().first) {
        impl->accelerometer_buffer.push_back(std::pair<double, double>(time, magnitude));
        
        // check if accelerometer buffer is too big, and trim if necesarry
        while (impl->accelerometer_buffer.back().first - impl->accelerometer_buffer.front().first > imu_half_window_time*2 + gps_window_comparison_precision) {
            LOG_VERBOSE << "removing point from accelerometer buffer";
            impl->accelerometer_buffer.pop_front();
        }
        return true;
    } else {
        impl->accelerometer_out_of_order++;
        return false;
    }
}

bool CrashDetector::add_gps_sample(double time, double magnitude) {
    CrashDetectorImpl *impl = static_cast<CrashDetectorImpl*>(this);
    if (impl->gps_buffer.empty() || time > impl->gps_buffer.back().first) {
        impl->gps_buffer.push_back(std::pair<double, double>(time, magnitude));

        // check if gps_buffer size is too big, trim down to gps_window_time
        while (!impl->gps_buffer.empty()) {
            if (std::abs(impl->gps_buffer.back().first - impl->gps_buffer.front().first) > gps_window_time + gps_window_comparison_precision) {
                impl->gps_buffer.pop_front();
                LOG_VERBOSE <<  "trimmed oldest gps point";
            } else {
                break;
            }
        }
        return true;
    } else {
        impl->gps_out_of_order++;
        return false;
    }
}

//
// Bookkeeping and other stuff
//

// Need to supply a constructor because we declared it protected.
CrashDetector::CrashDetector() { }

CrashDetector& CrashDetector::create() {
    return *(new CrashDetectorImpl);
}

void CrashDetector::set_crash_mode(Mode mode) {
    // TODO
}

void CrashDetector::get_crash_result(CrashResult& result) {
    CrashDetectorImpl *impl = static_cast<CrashDetectorImpl*>(this);
    result = impl->crash_result;
}

// TODO deal with counters
