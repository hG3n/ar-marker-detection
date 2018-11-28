#include "path.h"

Path::Path()
    :path_()
{

}


void Path::push(const std::string &folder) {
    path_.push_back(folder);
}

void Path::pop() {
    path_.pop_back();
}

const std::string & Path::str() const {
    std::string out = "";
    for(unsigned long i = 0; i < path_.size(); ++i) {
        if (i == 0) {
            out += path_[i];
        } else {
            out += "/";
            out += path_[i];
        }
    }
    return out;
}



