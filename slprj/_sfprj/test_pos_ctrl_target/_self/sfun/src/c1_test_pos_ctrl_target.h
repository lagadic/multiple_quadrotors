#ifndef __c1_test_pos_ctrl_target_h__
#define __c1_test_pos_ctrl_target_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_test_pos_ctrl_targetInstanceStruct
#define typedef_SFc1_test_pos_ctrl_targetInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_test_pos_ctrl_target;
  real_T c1_KDR;
  real_T c1_KPR;
  real_T c1_KDP;
  real_T c1_KPP;
  real_T c1_KDY;
  real_T c1_KPY;
} SFc1_test_pos_ctrl_targetInstanceStruct;

#endif                                 /*typedef_SFc1_test_pos_ctrl_targetInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_test_pos_ctrl_target_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c1_test_pos_ctrl_target_get_check_sum(mxArray *plhs[]);
extern void c1_test_pos_ctrl_target_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
