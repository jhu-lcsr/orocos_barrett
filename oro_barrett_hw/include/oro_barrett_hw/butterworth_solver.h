#ifndef BUTTER_SOLVER_H
#define BUTTER_SOLVER_H


#define   BUTTER_LOWPASS  0
#define   BUTTER_HIGHPASS 1
#define   BUTTER_BANDPASS 2
#define   BUTTER_BANDSTOP 3

/* butter: design a butterworth filter
 * 
 * Inputs:
 *     type: lowpass, highpass, bandpass, or bandstop
 *    order: filter order (>1)
 *       fs: sampling frequency
 *       fa: smaller cutoff frequency
 *       fb: larger cutoff frequency (unused for low/high)
 * 
 * Outputs:
 *       as: array of coefficients for the output data
 *       bs: array of coefficients for the input data
 * 
 * Note: Output arrays must be of length:
 *       order+1 for lowpass or highpass filters
 *       2*order+1 for bandpass or bandstop filters
 * 
 * Returns:
 *    0 on success
 *   -1 on failure (math overflow)
 *   -2 on internal unexpected zord number
 */

#ifdef __cplusplus
extern "C" {
#endif 

int butterworth_solver(const int type, const int order,
		       const double fs, const double fa, const double fb,
		       double * as, double * bs);

#ifdef __cplusplus
}
#endif 

#endif /* BUTTERWORTH_SOLVER_H */
