#ifndef gridmap_h
#define gridmap_h

#include <string>
#include <opencv2/core/core.hpp>

class gridmap {
  public:
    gridmap(int min_x = 0,
                int max_x = 100,
                int min_y = 0,
                int max_y = 100);
    ~gridmap();
    void set(int x, int y, int p);
    int get(int x, int y);
    void dumpToFolder(std::string foldername);
    cv::Mat data;
  private:
    gridmap *left;
    gridmap *right;
    gridmap *up;
    gridmap *down;
    int left_range;
    int right_range;
    int up_range;
    int down_range;
    bool visited;
    void clearVisit();
    void dumpVisit(std::string foldername);
};

#endif
