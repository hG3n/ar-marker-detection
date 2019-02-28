#ifndef PATH_H
#define PATH_H

#include <string>
#include <vector>

class Path
{
    public:
        Path();
        Path(const std::string &path);

        void push(const std::string &folder);
        void pop();

        const std::string& str() const;

    private:
        std::vector<std::string> path_;
};

#endif // PATH_H
