//
// Platform specific extras
// Created by Antoine on 8/23/16.
//

#ifndef Platform_hpp
#define Platform_hpp

#include <sstream>

namespace platform {


    // to_string doesn't exit on Android GNU C++ runtime so it must be redefined
    template<typename T>
    std::string to_string(T value) {
        std::ostringstream os;
        os << value;
        return os.str();
    }
}

#endif //Platform_hpp
