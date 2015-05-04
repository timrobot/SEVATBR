#include <dirent.h>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include "gridmap.h"

using namespace cv;
using namespace std;

gridmap::gridmap(int min_x, int max_x, int min_y, int max_y) {
  this->left_range = min_x;
  this->right_range = max_x;
  this->up_range = max_y;
  this->down_range = min_y;
  this->left = NULL;
  this->right = NULL;
  this->up = NULL;
  this->down = NULL;
  this->visited = false;
  this->data.create(max_y - min_y, max_x - min_x, CV_8UC3);
  for (int y = 0; y < (max_y - min_y); y++) {
    for (int x = 0; x < (max_x - min_x); x++) {
      this->data.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
    }
  }
}

gridmap::~gridmap() {
  gridmap *ptr;
  if ((ptr = this->left)) {
    this->left->right = NULL;
    this->left = NULL;
    delete ptr;
  }
  if ((ptr = this->right)) {
    this->right->left = NULL;
    this->right = NULL;
    delete ptr;
  }
  if ((ptr = this->up)) {
    this->up->down = NULL;
    this->up = NULL;
    delete ptr;
  }
  if ((ptr = this->down)) {
    this->down->up = NULL;
    this->down = NULL;
    delete ptr;
  }
}

void gridmap::set(int x, int y, int p) {
  int width = right_range - left_range;
  int height = up_range - down_range;
  if (x < this->left_range) {
    if (y < this->down_range && this->down) {
      this->down->set(x, y, p);
    } else if (y >= this->up_range && this->up) {
      this->up->set(x, y, p);
    } else {
      if (!this->left) {
        this->left = new gridmap(
            this->left_range - width,
            this->right_range - width,
            this->down_range,
            this->up_range);
        this->left->right = this;
      }
      this->left->set(x, y, p);
    }
  } else if (x >= this->right_range) {
    if (y < this->down_range && this->down) {
      this->down->set(x, y, p);
    } else if (y >= this->up_range && this->up) {
      this->up->set(x, y, p);
    } else {
      if (!this->right) {
        this->right = new gridmap(
            this->left_range + width,
            this->right_range + width,
            this->down_range,
            this->up_range);
        this->right->left = this;
      }
      this->right->set(x, y, p);
    }
  } else if (y >= this->up_range) {
    if (!this->up) {
      this->up = new gridmap(
          this->left_range,
          this->right_range,
          this->down_range + height,
          this->up_range + height);
      this->up->down = this;
    }
    this->up->set(x, y, p);
  } else if (y < this->down_range) {
    if (!this->down) {
      this->down = new gridmap(
          this->left_range,
          this->right_range,
          this->down_range - height,
          this->up_range - height);
      this->down->up = this;
    }
    this->down->set(x, y, p);
  } else {
    int x_coord = x - this->left_range;
    int y_coord = height - (y - this->down_range) - 1;
    this->data.at<Vec3b>(y_coord, x_coord) = Vec3b(p, p, p);
  }
}

int gridmap::get(int x, int y) {
  if (x < this->left_range)
    return this->left ? this->left->get(x, y) : -1;
  else if (x >= this->right_range)
    return this->right ? this->right->get(x, y) : -1;
  else if (y >= this->up_range)
    return this->up ? this->up->get(x, y) : -1;
  else if (y < this->down_range)
    return this->down ? this->down->get(x, y) : -1;
  else
    return this->data.at<Vec3b>(y, x)[0];
}

void gridmap::dumpToFolder(string foldername) {
  DIR *dp;
  struct dirent *entry;
  char command[256];
  if ((dp = opendir(foldername.c_str())) != NULL) {
    while ((entry = readdir(dp)) != NULL)
      if (foldername.compare(string(entry->d_name)) == 0) {
        sprintf(command, "rm -rf %s", foldername.c_str());
        system(command);
        break;
      }
    closedir(dp);
  }
  sprintf(command, "mkdir %s", foldername.c_str());
  system(command);
  clearVisit();
  dumpVisit(foldername);
}

void gridmap::clearVisit() {
  if (this->visited) {
    this->visited = false;
    this->left->clearVisit();
    this->right->clearVisit();
    this->up->clearVisit();
    this->down->clearVisit();
  }
}

void gridmap::dumpVisit(string foldername) {
  char filename[256];
  if (!this->visited) {
    this->visited = true;
    sprintf(filename, "%s/L%dR%dU%dD%d.bmp",
        foldername.c_str(),
        this->left_range,
        this->right_range,
        this->up_range,
        this->down_range);
    imwrite(filename, this->data);
    if (this->left) {
      this->left->dumpVisit(foldername);
    }
    if (this->right) {
      this->right->dumpVisit(foldername);
    }
    if (this->up) {
      this->up->dumpVisit(foldername);
    }
    if (this->down) {
      this->down->dumpVisit(foldername);
    }
  }
}
