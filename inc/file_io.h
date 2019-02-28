#ifndef FILE_IO_H
#define FILE_IO_H

#include <vector>
#include <string>

class FileIo {

    public:
        static int getDirectoryContent(const std::string & dir, std::vector<std::string> &files);

};

#endif // FILE_IO_H
