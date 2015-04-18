#include <math.h>
#include "ActionState.h"

static double mag(const arma::vec &v);

/** Constructor
 */
ActionState::ActionSttate(void) {
  this->startPos = arma::vec(4, arma::fill::zeros);
  this->stopPos = arma::vec(4, arma::fill::zeros);
  this->motionFcn = (ActionFcn)NULL;
  this->durationSec = 0.0;
  this->toleranceError = 0.0;
}

/** Constructor
 *  @param start
 *    the starting point
 *  @param stop
 *    the stopping point
 *  @param motion
 *    the motion function
 *  @param duration
 *    the time it should take to complete the motion
 *  @param tolerance
 *    the tolerance for declaring a state to be finished
 */
ActionState::ActionState(const arma::vec &start, const arma::vec &stop,
    ActionFcn motion, double duration, double tolerance) {
  this->startPos = start;
  this->stopPos = stop;
  this->motionFcn = motion;
  this->durationSec = duration;
  this->toleranceError = tolerance;
}

/** Return the intrepolated motion vector from an approxiamation of 1024 samples
 *  @param currPos
 *    the current position of the vector
 *  @return the weighted interpolation of the error and approxiamation derivative
 */
arma::vec ActionState::get_motion_vector(const arma::vec &currPos) {
  // Use binary approxiamation method to determine the closest point to the fcn
  double begTime, midTime, endTime;
  arma::vec begVec, midVec, endVec;
  arma::vec diff1, diff2;
  double weight1, weight2;
  int i;

  begTime = 0.0;
  endTime = this->durationSec;
  begVec = this->startPos;
  endVec = this->stopPos;
  weight1 = 1.0;
  weight2 = 1.5;

  // 10 iterations, 2 ^ 10 samples traversed, 1024 samples traversed
  for (i = 0; i < 10; i++) {
    midTime = (begTime + endTime) / 2;
    midVec = this->motionFcn(this->startPos, this->stopPos, midTime);
    diff1 = currPos - begVec;
    diff2 = currPos - endVec;
    if (mag(diff1) > mag(diff2)) {
      begVec = midVec;
      begTime = midTime;
    } else {
      endVec = midVec;
      endTime = midTime;
    }
  }
  midTime = (begTime + endTime) / 2;
  midVec = this->motionFcn(this->startPos, this->stopPos, midTime);

  // do a weighted calculation for the motion vector
  diff1 = arma::normalise(endVec - currPos);
  diff2 = arma::normalise(endVec - begVec);
  return (diff1 * weight1 + diff2 + weight2) / (weight1 + weight2);
}

/** Returns whether or not the current action state has finished
 *  @param currPos
 *    the current position of the vector
 *  @return true if within tolerance, else false
 */
bool ActionState::finished(const arma::vec &currPos) {
  return mag(currPos - this->stopPos) < this->toleranceError;
}

/** Return the motion vector of the piecewise motion sequence
 *  @param currPos
 *    the current position of the vector
 *  @return piecewise motion vector
 */
arma::vec ActionSequence::get_motion_vector(const arma::vec &currPos) {
  if (this->sequence[this->curr_action].finished(currPos)) {
    this->curr_action = (this->curr_action + 1) % this->sequence.size();
  }
  return this->sequence[this->curr_action].get_motion_vector(currPos);
}

/** Return the magnitude of a vector
 *  @param v
 *    the vector
 *  @return the magnitude
 */
static double mag(const arma::vec &v) {
  return sqrt(arma::dot(v, v));
}
