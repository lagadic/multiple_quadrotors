#ifndef __c3_test_pos_ctrl_target_h__
#define __c3_test_pos_ctrl_target_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc3_test_pos_ctrl_targetInstanceStruct
#define typedef_SFc3_test_pos_ctrl_targetInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_isStable;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_test_pos_ctrl_target;
  real_T c3_m0;
  real_T c3_g;
  real_T c3_kd[9];
  real_T c3_kp[9];
  real_T c3_ki[9];
  real_T c3_max_roll;
  real_T c3_max_pitch;
} SFc3_test_pos_ctrl_targetInstanceStruct;

#endif                                 /*typedef_SFc3_test_pos_ctrl_targetInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_test_pos_ctrl_target_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c3_test_pos_ctrl_target_get_check_sum(mxArray *plhs[]);
extern void c3_test_pos_ctrl_target_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
