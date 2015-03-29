#ifndef coord_h
#define coord_h

typedef struct pose3d {
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
} pose3d_t;

typedef struct pose2d {
  double x;
  double y;
  double theta;
} pose2d_t;

typedef struct point3d {
  double x;
  double y;
  double z;
} point3d_t;

typedef struct point2d {
  double x;
  double y;
} point2d_t;

typedef point2d_t point_t;
typedef pose2d_t pose_t;

#endif
