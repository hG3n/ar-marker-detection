#include "file_io.h"

#include <iostream>
#include <dirent.h>

int FileIo::getDirectoryContent(const std::string &dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;

    //Unable to open dir
    if((dp  = opendir(dir.c_str())) == nullptr) {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }

    //read files and push them to vector
    while ((dirp = readdir(dp)) != nullptr) {
        std::string name = std::string(dirp->d_name);

        //discard . and .. from list .. .DS_Store get the fuck outta here
        if(name != "." && name != ".." && name != ".DS_Store") {
            files.push_back(std::string(dirp->d_name));
        }
    }

    closedir(dp);
    std::sort(files.begin(), files.end());

    return 0;

}
