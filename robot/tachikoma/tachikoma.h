#ifndef tachikoma_h
#define tachikoma_h

#include <armadillo>
#include "coord.h"
#include "serial.h"

class tachikoma {
  private:
    serial_t *connections;
    int *ids;
    char **possible_ports;
    int num_possible;
    int num_connected;

    arma::vec leg_nw;
    arma::vec leg_ne;
    arma::vec leg_sw;
    arma::vec leg_se;

  public:
    pose3d_t *arm;
    pose3d_t *base;
    int x;
    int y;
    int z;
    int w;

    tachikoma(void);
    ~tachikoma(void);

    bool connect(void);
    bool connected(void);
    void send(void);
    void recv(void);
    void reset(void);
    void disconnect(void);
};

#endif
