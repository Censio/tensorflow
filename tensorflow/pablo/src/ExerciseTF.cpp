//
//  ExerciseTF.cpp
//  crash-osx
//
//  Created by Steven J. Clark on 8/19/16.
//  Copyright © 2016 Steven J. Clark. All rights reserved.
//

// Exercise the TensorFlow package and crash model in a simple way.

#include <string>
#include <vector>
#include <fstream>
#include "tensorflow/core/public/session.h"
#include "Logger.hpp"

// EWW GROSS uses some file-local variables

// A Priori information about the model.
// I assume one input node that takes a vector of inputlength floats
// and one output node that produces a single float.  (I mean, double
// in all cases.)

static unsigned s_inputlength = 0;
static std::string s_inputname;
static std::string s_outputname;

// the model
static tensorflow::Session* s_tf_session = NULL;


//
// Utility routines from Google (slightly scathed)
//

namespace {
    
    class IfstreamInputStream : public ::google::protobuf::io::CopyingInputStream {
    public:
        explicit IfstreamInputStream(const char* file_name)
        : ifs_(file_name, std::ios::in | std::ios::binary) {}
        ~IfstreamInputStream() { ifs_.close(); }
        
        int Read(void* buffer, int size) {
            if (!ifs_) {
                return -1;
            }
            ifs_.read(static_cast<char*>(buffer), size);
            return ifs_.gcount();
        }
        
    private:
        std::ifstream ifs_;
    };
    
    bool readFileToProto(const char* file_name,
                         ::google::protobuf::MessageLite* proto) {
        ::google::protobuf::io::CopyingInputStreamAdaptor stream(
                                                                 new IfstreamInputStream(file_name));
        stream.SetOwnsCopyingStream(true);
        ::google::protobuf::io::CodedInputStream coded_stream(&stream);
        // Total bytes hard limit / warning limit are set to 1GB and 512MB
        // respectively.
        coded_stream.SetTotalBytesLimit(1024LL << 20, 512LL << 20);
        return proto->ParseFromCodedStream(&coded_stream);
    }
    
}

void loadModel(const char* filename, const char* inputname, const char* outputname, unsigned inputlength) {
    s_inputname = inputname;
    s_outputname = outputname;
    s_inputlength = inputlength;
    tensorflow::SessionOptions options;
    tensorflow::Status session_status = tensorflow::NewSession(options, &s_tf_session);
    if (!session_status.ok()) {
        LOG_ERROR << "Could not create TensorFlow Session: " << session_status.error_message();
        return;
    }
    LOG_INFO << "Session created.";
    
    tensorflow::GraphDef tensorflow_graph;
    LOG_INFO << "Graph created.";
    
    const bool read_proto_succeeded = readFileToProto(filename, &tensorflow_graph);
    if (!read_proto_succeeded) {
        LOG_ERROR << "Failed to load model proto from" << filename;
        return;
    }
    
    LOG_INFO << "Creating session.";
    tensorflow::Status create_status = s_tf_session->Create(tensorflow_graph);
    if (!create_status.ok()) {
        LOG_ERROR << "Could not create TensorFlow Graph: " << create_status.error_message();
    }
}

double exercise_tf(double input[]) {
    tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, s_inputlength}));
    for (unsigned i = 0; i < s_inputlength; i++) input_tensor.flat<float>().data()[i] = input[i];
    std::vector<tensorflow::Tensor> outputs;
    LOG_INFO << "input, output, length = " << s_inputname << ", " << s_outputname << ", " << s_inputlength;
    tensorflow::Status run_status = s_tf_session->Run(
                                                  {{s_inputname, input_tensor},
                                                  },
                                                    {s_outputname},
                                                  {},
                                                  &outputs);
    if (!run_status.ok()) {
        LOG_ERROR << "Running model failed:" << run_status.error_message();
    } else {
        tensorflow::Tensor *output = &outputs[0];
        auto results = output->flat<float>();
        if (results.size() != 1) {
            LOG_ERROR << "Result vector of unexpected length: " << results.size();
        }
        if (results.size() >= 1) return results(0);
    }
    return 0;
}


