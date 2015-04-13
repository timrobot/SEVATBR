#ifndef tachikoma_h
#define tachikoma_h

#include <armadillo>
#include "coord.h"
#include "serial.h"

#define TACHI_NW_DEVID    1
#define TACHI_NE_DEVID    2
#define TACHI_SW_DEVID    3
#define TACHI_SE_DEVID    4

class tachikoma {
  private:
    serial_t *connections;
    int *ids;
    char **possible_ports;
    int num_possible;
    int num_connected;

    arma::vec leg[4];
    arma::vec encoder[4];
    arma::vec outval[4];
    arma::vec prevval[4];
    
    void leg_fk_solve(int legid);
    arma::vec leg_ik_solve(int legid, const arma::vec &target);

  public:
    pose3d_t base;
    pose3d_t arm[2];

    tachikoma(void);
    ~tachikoma(void);

    bool connect(void);
    void disconnect(void);
    bool connected(void);
    void send(void);
    void recv(void);
    void reset(void);

    // WARNING: Only use the following
    // if you know what you are doing.
    void write_manual(int legid, char *msg);
    char *read_manual(int legid);
};

#endif
