#ifndef occugridmap_h
#define occugridmap_h

#include <string>
#include <opencv2/core/core.hpp>

class occugridmap {
  public:
    occugridmap(int min_x = 0,
                int max_x = 100,
                int min_y = 0,
                int max_y = 100);
    ~occugridmap();
    void set(int x, int y, int p);
    int get(int x, int y);
    void dumpToFolder(std::string foldername);
    cv::Mat data;
  private:
    occugridmap *left;
    occugridmap *right;
    occugridmap *up;
    occugridmap *down;
    int left_range;
    int right_range;
    int up_range;
    int down_range;
    bool visited;
    void clearVisit();
    void dumpVisit(std::string foldername);
};

#endif
