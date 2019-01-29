#ifndef MISC_H
#define MISC_H

#include <string>
#include <opencv2/core.hpp>

class Misc
{
    public:
        static std::string type2str(int type);
        static void showImage(cv::Mat &image, const std::string &name = "Default", bool scale = false);
};

#endif // MISC_H
