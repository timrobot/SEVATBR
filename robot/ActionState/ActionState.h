#ifndef ActionState_h
#define ActionState_h

#include <armadillo>

typedef arma::vec (*ActionFcn)(const arma::vec &start, const arma::vec &stop, double t);

class ActionState {
  private:
    arma::vec motion_vector;

  public:
    arma::vec startPos;
    arma::vec stopPos;
    ActionFcn motionFcn;
    double toleranceError;

    ActionState(void);
    ActionState(const arma::vec &start,
                const arma::vec &stop,
                ActionFcn motion,
                double tolerance = 1.0);
    arma::vec get_motion_vector(const arma::vec &currPos);
    bool finished(const arma::vec &currpos);
};

class ActionSequence { // Piecewise functions
  public:
    std::vector<ActionState> sequence;
    int curr_action;
    arma::vec get_motion_vector(const arma::vec &currPos);
    bool finished(const arma::vec &currPos);
    void next_action(void);
    void add_action(const ActionState &action);
};

#endif
