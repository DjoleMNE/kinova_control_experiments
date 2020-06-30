#include "abag.h"

/* definitions of contained functions and schedules */
void fb_sched(double const * error_sign, double * actuation, double const * gain, double const * bias) {
  /* data block declarations */
  double gainedErrSgn_ge_sgn;

  /* fixed data flow schedule */
  gainedErrSgn_ge_sgn = (*gain) * (*error_sign);
  *actuation = +(*bias) + (gainedErrSgn_ge_sgn);

  /* update output ports */
}

void errSign(double const * input, double * output) {
  *output = (double)( (0 < *input) - (*input < 0) );
}

void errSignFilter(double const * filterFactor, double const * signedErr, double const * lowPassErrPrev, double * lowPassErr) {
  *lowPassErr = (*filterFactor) * (*lowPassErrPrev) + (1 - (*filterFactor)) * (*signedErr);
}

void delayEbar(double const * input, int * rearIndex, double delay[]) {
  delay[(*rearIndex)++] = *input;
  if (*rearIndex == 1) {
    *rearIndex = 0;
  }
}

void biasAdapter_sched(double const * e_bar, double const * bias_threshold, double const * bias_step, double * adapted_bias) {
  /* data block declarations */
  double decision_access;
  double bias_step_access;

  /* fixed data flow schedule */

  /* decision map biasAdaptDecision */
  if      (*e_bar < -*bias_threshold) decision_access = (double) -1.;
  else if (*e_bar > *bias_threshold) decision_access = (double) 1.;
  else decision_access = (double) 0.;

  bias_step_access = (*bias_step) * (decision_access);
  *adapted_bias = +(*adapted_bias) + (bias_step_access);

  /* saturation saturateBias */
  if      (*adapted_bias > 1.) *adapted_bias = 1.;
  else if (*adapted_bias < -1.) *adapted_bias = -1.;


  /* update output ports */
}

void gainAdapater_sched(double const * e_bar, double const * gain_threshold, double const * gain_step, double * adapted_gain) {
  /* data block declarations */
  double decision_access;
  double gain_step_access;

  /* fixed data flow schedule */

  /* decision map gainDecision */
  if      (*e_bar < -*gain_threshold) decision_access = (double) 1.;
  else if (*e_bar > *gain_threshold) decision_access = (double) 1.;
  else decision_access = (double) -1.;

  gain_step_access = (*gain_step) * (decision_access);
  *adapted_gain = +(*adapted_gain) + (gain_step_access);

  /* saturation saturateGain */
  if      (*adapted_gain > 1.) *adapted_gain = 1.;
  else if (*adapted_gain < -1.) *adapted_gain = -1.;


  /* update output ports */
}


/* definitions of root schedules */
void abag_sched(abagState_t * abagState, double const * error, double * actuation, double const * alpha) {
  /* data block declarations */
  const double bias_threshold_xi = 0.000457;
  const double bias_step_delta = 0.000500;
  const double gain_threshold_xi = 0.602492;
  const double gain_step_delta = 0.003552;

  /* fixed data flow schedule */
  errSign(error, &(abagState->signedErr_access));
  errSignFilter(alpha, &(abagState->signedErr_access), &(abagState->eBarDelay[abagState->delayIndex_access]), &(abagState->eBar_access));
  biasAdapter_sched(&(abagState->eBar_access), &bias_threshold_xi, &bias_step_delta, &(abagState->bias_access));
  gainAdapater_sched(&(abagState->eBar_access), &gain_threshold_xi, &gain_step_delta, &(abagState->gain_access));
  fb_sched(&(abagState->signedErr_access), actuation, &(abagState->gain_access), &(abagState->bias_access));
  delayEbar(&(abagState->eBar_access), &(abagState->delayIndex_access), abagState->eBarDelay);

  /* update output ports */
}