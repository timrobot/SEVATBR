typedef struct polar {
  double radius;
  double theta;
} polar_t;

typedef struct pose {
  double x;
  double y;
  double theta;
} pose_t;

typedef struct odoup {
  double drot1;
  double dtrans;
  double drot2;
} odoup_t;

void odo_update(odoup_t *ou, pose_t s0, pose_t s1) {
  double dy = s1.y - s0.y;
  double dx = s1.x - s0.x;
  ou->drot1 = atan2(dy, dx) - s0.theta;
  ou->trans = sqrt(dy * dy + dx * dx);
  ou->drot2 = s1.theta - (ou->trans + s0.theta);
}
