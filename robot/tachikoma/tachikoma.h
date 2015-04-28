#ifndef tachikoma_h
#define tachikoma_h

#include <armadillo>
#include <sys/time.h>
#include "coord.h"
#include "serial.h"
#include "actionstate.h"

#define TACHI_NUM_DEV         8
#define TACHI_NUM_LEG_DEV     4
#define TACHI_NUM_WHEEL_DEV   4
#define TACHI_NW_LEG_DEVID    1
#define TACHI_NE_LEG_DEVID    2
#define TACHI_SW_LEG_DEVID    3
#define TACHI_SE_LEG_DEVID    4
#define TACHI_NW_WHEEL_DEVID  5
#define TACHI_NE_WHEEL_DEVID  6
#define TACHI_SW_WHEEL_DEVID  7
#define TACHI_SE_WHEEL_DEVID  8

class tachikoma {
  private:
    serial_t *connections;
    int *ids;
    char **possible_ports;
    int num_possible;
    int num_connected;

    arma::vec curr_pos[TACHI_NUM_LEG_DEV];
    arma::vec curr_enc[TACHI_NUM_LEG_DEV];
    arma::vec target_pos[TACHI_NUM_LEG_DEV];
    arma::vec target_enc[TACHI_NUM_LEG_DEV];
    int legval[TACHI_NUM_LEG_DEV][3];
    int plegval[TACHI_NUM_LEG_DEV][3];
    int wheelval[TACHI_NUM_WHEEL_DEV][1];
    int pwheelval[TACHI_NUM_WHEEL_DEV][1];

    // Action State stuff
    int overall_state;
    int sub_state;
    actionsequence leg_seq[4];
    
    int getlegid(int devid);
    int getwheelid(int devid);
    void init_state_space(void);
    void send(void);
    void recv(void);
    void update_walk(double forward,
                        double backward,
                        double turn_left,
                        double turn_right);
    void update_stand(void);
    void update_drive(void);
    void leg_fk_solve(int legid);
    void leg_ik_solve(int legid, const arma::vec &target);

  public:
    pose3d_t base[2];
    pose3d_t arm[2];

    /** Constructor
     */
    tachikoma(void);

    /** Deconstructor
     */
    ~tachikoma(void);

    /** Initialize the communication layer.
     *  @return whether or not the robot was able to connect with a device
     */
    bool connect(void);

    /** Disconnect everything
     */
    void disconnect(void);

    /** Determine whether or not the robot is connected
     *  @return true if the robot is connected to a device, else false
     */
    bool connected(void);

    /** Return the number of devices that are connected
     *  @return the number of devices that are connected
     */
    int numconnected(void);

    /** Update the robot's incoming and outgoing signals
     *  @param wheelbase
     *    the pose for the wheels
     *  @param legbase
     *    the pose for the legs and waist
     *  @param leftarm
     *    the pose for the left arm
     *  @param rightarm
     *    the pose for the right arm
     */
    int update(pose3d_t wheelbase, pose3d_t legbase,
               pose3d_t leftclaw, pose3d_t rightclaw);

    /** Observe the current world
     *  @return NULL for now
     */
    pose3d_t *observe(void);

    /** Reset the robot values
     */
    void reset(void);

    /** Manually write the values for particular legs
     *  @param devid
     *    the id for which device
     *  @param message
     *    the message to send to the leg
     */
    void write_manual(int devid, char *msg);

    /** Manually read the values for particular legs
     *  @param devid
     *    the id for which device
     *  @return the message, or NULL if there isn't one
     */
    char *read_manual(int devid);
};

#endif
