#ifndef tachikoma_h
#define tachikoma_h

#include <armadillo>
#include <sys/time.h>
#include "coord.h"
#include "serial.h"
#include "ActionState.h"

#define TACHI_NUM_DEV       4
#define TACHI_NUM_LEG_DEV   4
#define TACHI_NW_DEVID      1
#define TACHI_NE_DEVID      2
#define TACHI_SW_DEVID      3
#define TACHI_SE_DEVID      4

class tachikoma {
  private:
    serial_t *connections;
    int *ids;
    char **possible_ports;
    int num_possible;
    int num_connected;

    arma::vec curr_pos[TACHI_NUM_LEG_DEV]; // TODO: add probabilities to this thing
    arma::vec curr_enc[TACHI_NUM_LEG_DEV];
    arma::vec target_pos[TACHI_NUM_LEG_DEV];
    arma::vec target_enc[TACHI_NUM_LEG_DEV];
    arma::vec outval[TACHI_NUM_LEG_DEV];
    arma::vec prevval[TACHI_NUM_LEG_DEV];

    // update stuff (inspired from game creation)
    int overall_state;
    int sub_state;
    ActionSequence leg_seq[4];
    
    void init_state_space(void);
    void update(double forward,
                double backward,
                double turn_left,
                double turn_right);
    int getlegid(int devid);
    void leg_fk_solve(int legid);
    void leg_ik_solve(int legid, const arma::vec &target);

  public:
    pose3d_t base[2];
    pose3d_t arm[2];

    tachikoma(void);
    ~tachikoma(void);

    bool connect(void);
    void disconnect(void);
    bool connected(void);
    int numconnected(void);
    void send(void);
    void recv(void);
    void reset(void);

    // WARNING: Only use the following
    // if you know what you are doing.
    void write_manual(int legid, char *msg);
    char *read_manual(int legid);
};

#endif
