/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-12-16T16:42:13+0800
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "network.h"
#include "network_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "0x70a124aa3dfbeaeb89c5988e6ee6348d"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2025-12-16T16:42:13+0800"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

static ai_ptr g_network_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_network_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  input_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 5, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  _input_layer_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_Sub_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_Div_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  _Sigmoid_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  _Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_shortcut_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_linear1_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_Sub_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_Div_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_Sigmoid_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_linear2_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#27 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#28 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#29 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_Sub_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#30 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#31 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#32 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#33 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#34 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_Div_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#35 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#36 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_Add_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#37 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_Sigmoid_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#38 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_Mul_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#39 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_shortcut_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#40 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_linear1_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#41 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#42 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#43 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_Sub_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#44 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#45 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#46 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#47 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#48 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_Div_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#49 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#50 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_Sigmoid_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#51 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#52 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_linear2_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#53 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#54 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#55 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_Sub_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#56 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#57 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#58 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#59 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#60 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_Div_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#61 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#62 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_Add_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#63 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_Sigmoid_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#64 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_Mul_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#65 */
AI_ARRAY_OBJ_DECLARE(
  _attention_attention_0_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#66 */
AI_ARRAY_OBJ_DECLARE(
  _attention_attention_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#67 */
AI_ARRAY_OBJ_DECLARE(
  _attention_attention_2_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#68 */
AI_ARRAY_OBJ_DECLARE(
  _attention_attention_3_Sigmoid_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#69 */
AI_ARRAY_OBJ_DECLARE(
  _Mul_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#70 */
AI_ARRAY_OBJ_DECLARE(
  _output_layer_output_layer_0_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#71 */
AI_ARRAY_OBJ_DECLARE(
  _output_layer_output_layer_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#72 */
AI_ARRAY_OBJ_DECLARE(
  output_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 3, AI_STATIC)

/* Array#73 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_Constant_output_0_2D_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#74 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_Constant_output_0_2D_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#75 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_Constant_output_0_2D_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#76 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_Constant_output_0_2D_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#77 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_Constant_output_0_2D_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#78 */
AI_ARRAY_OBJ_DECLARE(
  _input_layer_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 640, AI_STATIC)

/* Array#79 */
AI_ARRAY_OBJ_DECLARE(
  _input_layer_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#80 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_Mul_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#81 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_Mul_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#82 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_Mul_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#83 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_Mul_output_0_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#84 */
AI_ARRAY_OBJ_DECLARE(
  _norm_input_Mul_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#85 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_shortcut_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_LUT4_FLOAT,
  NULL, NULL, 32768, AI_STATIC)

/* Array#86 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_shortcut_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#87 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_linear1_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_LUT4_FLOAT,
  NULL, NULL, 32768, AI_STATIC)

/* Array#88 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_linear1_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#89 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#90 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_Mul_output_0_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#91 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm1_Mul_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#92 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_linear2_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_LUT4_FLOAT,
  NULL, NULL, 65536, AI_STATIC)

/* Array#93 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_linear2_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#94 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_Mul_output_0_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#95 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_0_norm2_Mul_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#96 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_shortcut_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_LUT4_FLOAT,
  NULL, NULL, 32768, AI_STATIC)

/* Array#97 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_shortcut_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#98 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_linear1_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_LUT4_FLOAT,
  NULL, NULL, 32768, AI_STATIC)

/* Array#99 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_linear1_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#100 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_Mul_output_0_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#101 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm1_Mul_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#102 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_linear2_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_LUT4_FLOAT,
  NULL, NULL, 16384, AI_STATIC)

/* Array#103 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_linear2_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#104 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_Mul_output_0_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#105 */
AI_ARRAY_OBJ_DECLARE(
  _residual_blocks_1_norm2_Mul_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#106 */
AI_ARRAY_OBJ_DECLARE(
  _attention_attention_0_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_LUT4_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#107 */
AI_ARRAY_OBJ_DECLARE(
  _attention_attention_0_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#108 */
AI_ARRAY_OBJ_DECLARE(
  _attention_attention_2_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_LUT4_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#109 */
AI_ARRAY_OBJ_DECLARE(
  _attention_attention_2_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#110 */
AI_ARRAY_OBJ_DECLARE(
  _output_layer_output_layer_0_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_LUT4_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#111 */
AI_ARRAY_OBJ_DECLARE(
  _output_layer_output_layer_0_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#112 */
AI_ARRAY_OBJ_DECLARE(
  output_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#113 */
AI_ARRAY_OBJ_DECLARE(
  output_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  _Mul_1_output_0_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_Mul_1_output_0_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  _Mul_output_0_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_Mul_output_0_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  _Sigmoid_output_0_output, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_Sigmoid_output_0_output_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  _attention_attention_0_Gemm_output_0_bias, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_attention_attention_0_Gemm_output_0_bias_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  _attention_attention_0_Gemm_output_0_output, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_attention_attention_0_Gemm_output_0_output_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  _attention_attention_0_Gemm_output_0_weights, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 128, 64, 1, 1), AI_STRIDE_INIT(4, 1, 64, 4096, 4096),
  1, &_attention_attention_0_Gemm_output_0_weights_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  _attention_attention_1_Relu_output_0_output, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_attention_attention_1_Relu_output_0_output_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  _attention_attention_2_Gemm_output_0_bias, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_attention_attention_2_Gemm_output_0_bias_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  _attention_attention_2_Gemm_output_0_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_attention_attention_2_Gemm_output_0_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  _attention_attention_2_Gemm_output_0_weights, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 64, 128, 1, 1), AI_STRIDE_INIT(4, 1, 32, 4096, 4096),
  1, &_attention_attention_2_Gemm_output_0_weights_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  _attention_attention_3_Sigmoid_output_0_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_attention_attention_3_Sigmoid_output_0_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  _input_layer_Gemm_output_0_bias, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_input_layer_Gemm_output_0_bias_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  _input_layer_Gemm_output_0_output, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_input_layer_Gemm_output_0_output_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  _input_layer_Gemm_output_0_weights, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 5, 128, 1, 1), AI_STRIDE_INIT(4, 4, 20, 2560, 2560),
  1, &_input_layer_Gemm_output_0_weights_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_Constant_output_0_2D, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_norm_input_Constant_output_0_2D_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_Div_output_0_output, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_norm_input_Div_output_0_output_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_Mul_output_0_bias, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_norm_input_Mul_output_0_bias_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_Mul_output_0_output, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_norm_input_Mul_output_0_output_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_Mul_output_0_scale, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_norm_input_Mul_output_0_scale_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_Pow_output_0_output, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_norm_input_Pow_output_0_output_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_Mul_bias, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_norm_input_ReduceMean_1_output_0_Mul_bias_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_norm_input_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_output, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_norm_input_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_Mul_bias, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_norm_input_ReduceMean_output_0_Mul_bias_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_Mul_output, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_norm_input_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_Mul_scale, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_norm_input_ReduceMean_output_0_Mul_scale_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_output, AI_STATIC,
  26, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_norm_input_ReduceMean_output_0_output_array, NULL)

/* Tensor #27 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_Sqrt_output_0_output, AI_STATIC,
  27, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_norm_input_Sqrt_output_0_output_array, NULL)

/* Tensor #28 */
AI_TENSOR_OBJ_DECLARE(
  _norm_input_Sub_output_0_output, AI_STATIC,
  28, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_norm_input_Sub_output_0_output_array, NULL)

/* Tensor #29 */
AI_TENSOR_OBJ_DECLARE(
  _output_layer_output_layer_0_Gemm_output_0_bias, AI_STATIC,
  29, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_output_layer_output_layer_0_Gemm_output_0_bias_array, NULL)

/* Tensor #30 */
AI_TENSOR_OBJ_DECLARE(
  _output_layer_output_layer_0_Gemm_output_0_output, AI_STATIC,
  30, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_output_layer_output_layer_0_Gemm_output_0_output_array, NULL)

/* Tensor #31 */
AI_TENSOR_OBJ_DECLARE(
  _output_layer_output_layer_0_Gemm_output_0_weights, AI_STATIC,
  31, 0x0,
  AI_SHAPE_INIT(4, 128, 64, 1, 1), AI_STRIDE_INIT(4, 1, 64, 4096, 4096),
  1, &_output_layer_output_layer_0_Gemm_output_0_weights_array, NULL)

/* Tensor #32 */
AI_TENSOR_OBJ_DECLARE(
  _output_layer_output_layer_1_Relu_output_0_output, AI_STATIC,
  32, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_output_layer_output_layer_1_Relu_output_0_output_array, NULL)

/* Tensor #33 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_Add_output_0_output, AI_STATIC,
  33, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_Add_output_0_output_array, NULL)

/* Tensor #34 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_Mul_1_output_0_output, AI_STATIC,
  34, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_Mul_1_output_0_output_array, NULL)

/* Tensor #35 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_Mul_output_0_output, AI_STATIC,
  35, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_Mul_output_0_output_array, NULL)

/* Tensor #36 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_Sigmoid_1_output_0_output, AI_STATIC,
  36, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_Sigmoid_1_output_0_output_array, NULL)

/* Tensor #37 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_Sigmoid_output_0_output, AI_STATIC,
  37, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_Sigmoid_output_0_output_array, NULL)

/* Tensor #38 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_linear1_Gemm_output_0_bias, AI_STATIC,
  38, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_linear1_Gemm_output_0_bias_array, NULL)

/* Tensor #39 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_linear1_Gemm_output_0_output, AI_STATIC,
  39, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_linear1_Gemm_output_0_output_array, NULL)

/* Tensor #40 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_linear1_Gemm_output_0_weights, AI_STATIC,
  40, 0x0,
  AI_SHAPE_INIT(4, 128, 256, 1, 1), AI_STRIDE_INIT(4, 1, 64, 16384, 16384),
  1, &_residual_blocks_0_linear1_Gemm_output_0_weights_array, NULL)

/* Tensor #41 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_linear2_Gemm_output_0_bias, AI_STATIC,
  41, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_linear2_Gemm_output_0_bias_array, NULL)

/* Tensor #42 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_linear2_Gemm_output_0_output, AI_STATIC,
  42, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_linear2_Gemm_output_0_output_array, NULL)

/* Tensor #43 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_linear2_Gemm_output_0_weights, AI_STATIC,
  43, 0x0,
  AI_SHAPE_INIT(4, 256, 256, 1, 1), AI_STRIDE_INIT(4, 1, 128, 32768, 32768),
  1, &_residual_blocks_0_linear2_Gemm_output_0_weights_array, NULL)

/* Tensor #44 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_Constant_output_0_2D, AI_STATIC,
  44, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm1_Constant_output_0_2D_array, NULL)

/* Tensor #45 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_Div_output_0_output, AI_STATIC,
  45, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm1_Div_output_0_output_array, NULL)

/* Tensor #46 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_Mul_output_0_bias, AI_STATIC,
  46, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm1_Mul_output_0_bias_array, NULL)

/* Tensor #47 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_Mul_output_0_output, AI_STATIC,
  47, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm1_Mul_output_0_output_array, NULL)

/* Tensor #48 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_Mul_output_0_scale, AI_STATIC,
  48, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm1_Mul_output_0_scale_array, NULL)

/* Tensor #49 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_Pow_output_0_output, AI_STATIC,
  49, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm1_Pow_output_0_output_array, NULL)

/* Tensor #50 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  50, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #51 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_1_output_0_output, AI_STATIC,
  51, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm1_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #52 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_Mul_output, AI_STATIC,
  52, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #53 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale, AI_STATIC,
  53, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale_array, NULL)

/* Tensor #54 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_output, AI_STATIC,
  54, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm1_ReduceMean_output_0_output_array, NULL)

/* Tensor #55 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_Sqrt_output_0_output, AI_STATIC,
  55, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm1_Sqrt_output_0_output_array, NULL)

/* Tensor #56 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm1_Sub_output_0_output, AI_STATIC,
  56, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm1_Sub_output_0_output_array, NULL)

/* Tensor #57 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_Constant_output_0_2D, AI_STATIC,
  57, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm2_Constant_output_0_2D_array, NULL)

/* Tensor #58 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_Div_output_0_output, AI_STATIC,
  58, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm2_Div_output_0_output_array, NULL)

/* Tensor #59 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_Mul_output_0_bias, AI_STATIC,
  59, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm2_Mul_output_0_bias_array, NULL)

/* Tensor #60 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_Mul_output_0_output, AI_STATIC,
  60, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm2_Mul_output_0_output_array, NULL)

/* Tensor #61 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_Mul_output_0_scale, AI_STATIC,
  61, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm2_Mul_output_0_scale_array, NULL)

/* Tensor #62 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_Pow_output_0_output, AI_STATIC,
  62, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm2_Pow_output_0_output_array, NULL)

/* Tensor #63 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  63, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #64 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_1_output_0_output, AI_STATIC,
  64, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm2_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #65 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_output_0_Mul_output, AI_STATIC,
  65, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm2_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #66 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_output_0_output, AI_STATIC,
  66, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm2_ReduceMean_output_0_output_array, NULL)

/* Tensor #67 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_Sqrt_output_0_output, AI_STATIC,
  67, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_0_norm2_Sqrt_output_0_output_array, NULL)

/* Tensor #68 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_norm2_Sub_output_0_output, AI_STATIC,
  68, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_norm2_Sub_output_0_output_array, NULL)

/* Tensor #69 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_shortcut_Gemm_output_0_bias, AI_STATIC,
  69, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_shortcut_Gemm_output_0_bias_array, NULL)

/* Tensor #70 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_shortcut_Gemm_output_0_output, AI_STATIC,
  70, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_residual_blocks_0_shortcut_Gemm_output_0_output_array, NULL)

/* Tensor #71 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_0_shortcut_Gemm_output_0_weights, AI_STATIC,
  71, 0x0,
  AI_SHAPE_INIT(4, 128, 256, 1, 1), AI_STRIDE_INIT(4, 1, 64, 16384, 16384),
  1, &_residual_blocks_0_shortcut_Gemm_output_0_weights_array, NULL)

/* Tensor #72 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_Add_output_0_output, AI_STATIC,
  72, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_Add_output_0_output_array, NULL)

/* Tensor #73 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_Mul_1_output_0_output, AI_STATIC,
  73, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_Mul_1_output_0_output_array, NULL)

/* Tensor #74 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_Mul_output_0_output, AI_STATIC,
  74, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_Mul_output_0_output_array, NULL)

/* Tensor #75 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_Sigmoid_1_output_0_output, AI_STATIC,
  75, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_Sigmoid_1_output_0_output_array, NULL)

/* Tensor #76 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_Sigmoid_output_0_output, AI_STATIC,
  76, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_Sigmoid_output_0_output_array, NULL)

/* Tensor #77 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_linear1_Gemm_output_0_bias, AI_STATIC,
  77, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_linear1_Gemm_output_0_bias_array, NULL)

/* Tensor #78 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_linear1_Gemm_output_0_output, AI_STATIC,
  78, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_linear1_Gemm_output_0_output_array, NULL)

/* Tensor #79 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_linear1_Gemm_output_0_weights, AI_STATIC,
  79, 0x0,
  AI_SHAPE_INIT(4, 256, 128, 1, 1), AI_STRIDE_INIT(4, 1, 128, 16384, 16384),
  1, &_residual_blocks_1_linear1_Gemm_output_0_weights_array, NULL)

/* Tensor #80 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_linear2_Gemm_output_0_bias, AI_STATIC,
  80, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_linear2_Gemm_output_0_bias_array, NULL)

/* Tensor #81 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_linear2_Gemm_output_0_output, AI_STATIC,
  81, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_linear2_Gemm_output_0_output_array, NULL)

/* Tensor #82 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_linear2_Gemm_output_0_weights, AI_STATIC,
  82, 0x0,
  AI_SHAPE_INIT(4, 128, 128, 1, 1), AI_STRIDE_INIT(4, 1, 64, 8192, 8192),
  1, &_residual_blocks_1_linear2_Gemm_output_0_weights_array, NULL)

/* Tensor #83 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_Constant_output_0_2D, AI_STATIC,
  83, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm1_Constant_output_0_2D_array, NULL)

/* Tensor #84 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_Div_output_0_output, AI_STATIC,
  84, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm1_Div_output_0_output_array, NULL)

/* Tensor #85 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_Mul_output_0_bias, AI_STATIC,
  85, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm1_Mul_output_0_bias_array, NULL)

/* Tensor #86 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_Mul_output_0_output, AI_STATIC,
  86, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm1_Mul_output_0_output_array, NULL)

/* Tensor #87 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_Mul_output_0_scale, AI_STATIC,
  87, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm1_Mul_output_0_scale_array, NULL)

/* Tensor #88 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_Pow_output_0_output, AI_STATIC,
  88, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm1_Pow_output_0_output_array, NULL)

/* Tensor #89 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  89, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #90 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_1_output_0_output, AI_STATIC,
  90, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm1_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #91 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_output_0_Mul_output, AI_STATIC,
  91, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm1_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #92 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_output_0_output, AI_STATIC,
  92, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm1_ReduceMean_output_0_output_array, NULL)

/* Tensor #93 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_Sqrt_output_0_output, AI_STATIC,
  93, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm1_Sqrt_output_0_output_array, NULL)

/* Tensor #94 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm1_Sub_output_0_output, AI_STATIC,
  94, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm1_Sub_output_0_output_array, NULL)

/* Tensor #95 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_Constant_output_0_2D, AI_STATIC,
  95, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm2_Constant_output_0_2D_array, NULL)

/* Tensor #96 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_Div_output_0_output, AI_STATIC,
  96, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm2_Div_output_0_output_array, NULL)

/* Tensor #97 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_Mul_output_0_bias, AI_STATIC,
  97, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm2_Mul_output_0_bias_array, NULL)

/* Tensor #98 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_Mul_output_0_output, AI_STATIC,
  98, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm2_Mul_output_0_output_array, NULL)

/* Tensor #99 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_Mul_output_0_scale, AI_STATIC,
  99, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm2_Mul_output_0_scale_array, NULL)

/* Tensor #100 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_Pow_output_0_output, AI_STATIC,
  100, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm2_Pow_output_0_output_array, NULL)

/* Tensor #101 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  101, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #102 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_1_output_0_output, AI_STATIC,
  102, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm2_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #103 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_output_0_Mul_output, AI_STATIC,
  103, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm2_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #104 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_output_0_output, AI_STATIC,
  104, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm2_ReduceMean_output_0_output_array, NULL)

/* Tensor #105 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_Sqrt_output_0_output, AI_STATIC,
  105, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_residual_blocks_1_norm2_Sqrt_output_0_output_array, NULL)

/* Tensor #106 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_norm2_Sub_output_0_output, AI_STATIC,
  106, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_norm2_Sub_output_0_output_array, NULL)

/* Tensor #107 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_shortcut_Gemm_output_0_bias, AI_STATIC,
  107, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_shortcut_Gemm_output_0_bias_array, NULL)

/* Tensor #108 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_shortcut_Gemm_output_0_output, AI_STATIC,
  108, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_residual_blocks_1_shortcut_Gemm_output_0_output_array, NULL)

/* Tensor #109 */
AI_TENSOR_OBJ_DECLARE(
  _residual_blocks_1_shortcut_Gemm_output_0_weights, AI_STATIC,
  109, 0x0,
  AI_SHAPE_INIT(4, 256, 128, 1, 1), AI_STRIDE_INIT(4, 1, 128, 16384, 16384),
  1, &_residual_blocks_1_shortcut_Gemm_output_0_weights_array, NULL)

/* Tensor #110 */
AI_TENSOR_OBJ_DECLARE(
  input_output, AI_STATIC,
  110, 0x0,
  AI_SHAPE_INIT(4, 1, 5, 1, 1), AI_STRIDE_INIT(4, 4, 4, 20, 20),
  1, &input_output_array, NULL)

/* Tensor #111 */
AI_TENSOR_OBJ_DECLARE(
  output_bias, AI_STATIC,
  111, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 1, 1), AI_STRIDE_INIT(4, 4, 4, 12, 12),
  1, &output_bias_array, NULL)

/* Tensor #112 */
AI_TENSOR_OBJ_DECLARE(
  output_output, AI_STATIC,
  112, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 1, 1), AI_STRIDE_INIT(4, 4, 4, 12, 12),
  1, &output_output_array, NULL)

/* Tensor #113 */
AI_TENSOR_OBJ_DECLARE(
  output_weights, AI_STATIC,
  113, 0x0,
  AI_SHAPE_INIT(4, 64, 3, 1, 1), AI_STRIDE_INIT(4, 4, 256, 768, 768),
  1, &output_weights_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  output_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_output_layer_output_layer_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &output_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &output_weights, &output_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  output_layer, 82,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &output_chain,
  NULL, &output_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _output_layer_output_layer_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_output_layer_output_layer_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_output_layer_output_layer_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _output_layer_output_layer_1_Relu_output_0_layer, 81,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_output_layer_output_layer_1_Relu_output_0_chain,
  NULL, &output_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _output_layer_output_layer_0_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Mul_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_output_layer_output_layer_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_output_layer_output_layer_0_Gemm_output_0_weights, &_output_layer_output_layer_0_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _output_layer_output_layer_0_Gemm_output_0_layer, 80,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_output_layer_output_layer_0_Gemm_output_0_chain,
  NULL, &_output_layer_output_layer_1_Relu_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Mul_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_Mul_1_output_0_output, &_attention_attention_3_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Mul_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Mul_1_output_0_layer, 79,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Mul_1_output_0_chain,
  NULL, &_output_layer_output_layer_0_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _attention_attention_3_Sigmoid_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_attention_attention_2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_attention_attention_3_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _attention_attention_3_Sigmoid_output_0_layer, 78,
  NL_TYPE, 0x0, NULL,
  nl, forward_sigmoid,
  &_attention_attention_3_Sigmoid_output_0_chain,
  NULL, &_Mul_1_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _attention_attention_2_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_attention_attention_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_attention_attention_2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_attention_attention_2_Gemm_output_0_weights, &_attention_attention_2_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _attention_attention_2_Gemm_output_0_layer, 77,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_attention_attention_2_Gemm_output_0_chain,
  NULL, &_attention_attention_3_Sigmoid_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _attention_attention_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_attention_attention_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_attention_attention_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _attention_attention_1_Relu_output_0_layer, 76,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_attention_attention_1_Relu_output_0_chain,
  NULL, &_attention_attention_2_Gemm_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _attention_attention_0_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_Mul_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_attention_attention_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_attention_attention_0_Gemm_output_0_weights, &_attention_attention_0_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _attention_attention_0_Gemm_output_0_layer, 75,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_attention_attention_0_Gemm_output_0_chain,
  NULL, &_attention_attention_1_Relu_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_Mul_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_Add_output_0_output, &_residual_blocks_1_Sigmoid_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_Mul_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_Mul_1_output_0_layer, 74,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_1_Mul_1_output_0_chain,
  NULL, &_attention_attention_0_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_Sigmoid_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_Sigmoid_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_Sigmoid_1_output_0_layer, 73,
  NL_TYPE, 0x0, NULL,
  nl, forward_sigmoid,
  &_residual_blocks_1_Sigmoid_1_output_0_chain,
  NULL, &_residual_blocks_1_Mul_1_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_norm2_Mul_output_0_output, &_residual_blocks_1_shortcut_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_Add_output_0_layer, 72,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_1_Add_output_0_chain,
  NULL, &_residual_blocks_1_Sigmoid_1_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm2_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_norm2_Mul_output_0_scale, &_residual_blocks_1_norm2_Mul_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm2_Mul_output_0_layer, 71,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_1_norm2_Mul_output_0_chain,
  NULL, &_residual_blocks_1_Add_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm2_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_norm2_Sub_output_0_output, &_residual_blocks_1_norm2_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm2_Div_output_0_layer, 69,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_1_norm2_Div_output_0_chain,
  NULL, &_residual_blocks_1_norm2_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm2_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm2_Sqrt_output_0_layer, 68,
  NL_TYPE, 0x0, NULL,
  nl, forward_sqrt,
  &_residual_blocks_1_norm2_Sqrt_output_0_chain,
  NULL, &_residual_blocks_1_norm2_Div_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_1_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_layer, 67,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_chain,
  NULL, &_residual_blocks_1_norm2_Sqrt_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _residual_blocks_1_norm2_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _residual_blocks_1_norm2_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _residual_blocks_1_norm2_ReduceMean_1_output_0_neutral_value_data, _residual_blocks_1_norm2_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_1_output_0_layer, 67,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_residual_blocks_1_norm2_ReduceMean_1_output_0_chain,
  NULL, &_residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_residual_blocks_1_norm2_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm2_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_norm2_Sub_output_0_output, &_residual_blocks_1_norm2_Constant_output_0_2D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm2_Pow_output_0_layer, 64,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_1_norm2_Pow_output_0_chain,
  NULL, &_residual_blocks_1_norm2_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm2_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_linear2_Gemm_output_0_output, &_residual_blocks_1_norm2_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm2_Sub_output_0_layer, 62,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_1_norm2_Sub_output_0_chain,
  NULL, &_residual_blocks_1_norm2_Pow_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_output_0_Mul_layer, 61,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_1_norm2_ReduceMean_output_0_Mul_chain,
  NULL, &_residual_blocks_1_norm2_Sub_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _residual_blocks_1_norm2_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _residual_blocks_1_norm2_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _residual_blocks_1_norm2_ReduceMean_output_0_neutral_value_data, _residual_blocks_1_norm2_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_linear2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm2_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm2_ReduceMean_output_0_layer, 61,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_residual_blocks_1_norm2_ReduceMean_output_0_chain,
  NULL, &_residual_blocks_1_norm2_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_residual_blocks_1_norm2_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_linear2_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_linear2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_linear2_Gemm_output_0_weights, &_residual_blocks_1_linear2_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_linear2_Gemm_output_0_layer, 60,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_residual_blocks_1_linear2_Gemm_output_0_chain,
  NULL, &_residual_blocks_1_norm2_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_norm1_Mul_output_0_output, &_residual_blocks_1_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_Mul_output_0_layer, 59,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_1_Mul_output_0_chain,
  NULL, &_residual_blocks_1_linear2_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_Sigmoid_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_Sigmoid_output_0_layer, 58,
  NL_TYPE, 0x0, NULL,
  nl, forward_sigmoid,
  &_residual_blocks_1_Sigmoid_output_0_chain,
  NULL, &_residual_blocks_1_Mul_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm1_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_norm1_Mul_output_0_scale, &_residual_blocks_1_norm1_Mul_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm1_Mul_output_0_layer, 57,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_1_norm1_Mul_output_0_chain,
  NULL, &_residual_blocks_1_Sigmoid_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm1_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_norm1_Sub_output_0_output, &_residual_blocks_1_norm1_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm1_Div_output_0_layer, 55,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_1_norm1_Div_output_0_chain,
  NULL, &_residual_blocks_1_norm1_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm1_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm1_Sqrt_output_0_layer, 54,
  NL_TYPE, 0x0, NULL,
  nl, forward_sqrt,
  &_residual_blocks_1_norm1_Sqrt_output_0_chain,
  NULL, &_residual_blocks_1_norm1_Div_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_1_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_layer, 53,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_chain,
  NULL, &_residual_blocks_1_norm1_Sqrt_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _residual_blocks_1_norm1_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _residual_blocks_1_norm1_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _residual_blocks_1_norm1_ReduceMean_1_output_0_neutral_value_data, _residual_blocks_1_norm1_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_1_output_0_layer, 53,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_residual_blocks_1_norm1_ReduceMean_1_output_0_chain,
  NULL, &_residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_residual_blocks_1_norm1_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm1_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_norm1_Sub_output_0_output, &_residual_blocks_1_norm1_Constant_output_0_2D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm1_Pow_output_0_layer, 50,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_1_norm1_Pow_output_0_chain,
  NULL, &_residual_blocks_1_norm1_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm1_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_linear1_Gemm_output_0_output, &_residual_blocks_1_norm1_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm1_Sub_output_0_layer, 48,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_1_norm1_Sub_output_0_chain,
  NULL, &_residual_blocks_1_norm1_Pow_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_output_0_Mul_layer, 47,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_1_norm1_ReduceMean_output_0_Mul_chain,
  NULL, &_residual_blocks_1_norm1_Sub_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _residual_blocks_1_norm1_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _residual_blocks_1_norm1_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _residual_blocks_1_norm1_ReduceMean_output_0_neutral_value_data, _residual_blocks_1_norm1_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_linear1_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_norm1_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_norm1_ReduceMean_output_0_layer, 47,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_residual_blocks_1_norm1_ReduceMean_output_0_chain,
  NULL, &_residual_blocks_1_norm1_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_residual_blocks_1_norm1_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_linear1_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_Mul_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_linear1_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_linear1_Gemm_output_0_weights, &_residual_blocks_1_linear1_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_linear1_Gemm_output_0_layer, 46,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_residual_blocks_1_linear1_Gemm_output_0_chain,
  NULL, &_residual_blocks_1_norm1_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_1_shortcut_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_Mul_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_1_shortcut_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_1_shortcut_Gemm_output_0_weights, &_residual_blocks_1_shortcut_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_1_shortcut_Gemm_output_0_layer, 45,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_residual_blocks_1_shortcut_Gemm_output_0_chain,
  NULL, &_residual_blocks_1_linear1_Gemm_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_Mul_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_Add_output_0_output, &_residual_blocks_0_Sigmoid_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_Mul_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_Mul_1_output_0_layer, 44,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_0_Mul_1_output_0_chain,
  NULL, &_residual_blocks_1_shortcut_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_Sigmoid_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_Sigmoid_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_Sigmoid_1_output_0_layer, 43,
  NL_TYPE, 0x0, NULL,
  nl, forward_sigmoid,
  &_residual_blocks_0_Sigmoid_1_output_0_chain,
  NULL, &_residual_blocks_0_Mul_1_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm2_Mul_output_0_output, &_residual_blocks_0_shortcut_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_Add_output_0_layer, 42,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_0_Add_output_0_chain,
  NULL, &_residual_blocks_0_Sigmoid_1_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm2_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm2_Mul_output_0_scale, &_residual_blocks_0_norm2_Mul_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm2_Mul_output_0_layer, 41,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_0_norm2_Mul_output_0_chain,
  NULL, &_residual_blocks_0_Add_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm2_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm2_Sub_output_0_output, &_residual_blocks_0_norm2_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm2_Div_output_0_layer, 39,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_0_norm2_Div_output_0_chain,
  NULL, &_residual_blocks_0_norm2_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm2_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm2_Sqrt_output_0_layer, 38,
  NL_TYPE, 0x0, NULL,
  nl, forward_sqrt,
  &_residual_blocks_0_norm2_Sqrt_output_0_chain,
  NULL, &_residual_blocks_0_norm2_Div_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_1_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_layer, 37,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_chain,
  NULL, &_residual_blocks_0_norm2_Sqrt_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _residual_blocks_0_norm2_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _residual_blocks_0_norm2_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _residual_blocks_0_norm2_ReduceMean_1_output_0_neutral_value_data, _residual_blocks_0_norm2_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_1_output_0_layer, 37,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_residual_blocks_0_norm2_ReduceMean_1_output_0_chain,
  NULL, &_residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_residual_blocks_0_norm2_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm2_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm2_Sub_output_0_output, &_residual_blocks_0_norm2_Constant_output_0_2D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm2_Pow_output_0_layer, 34,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_0_norm2_Pow_output_0_chain,
  NULL, &_residual_blocks_0_norm2_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm2_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_linear2_Gemm_output_0_output, &_residual_blocks_0_norm2_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm2_Sub_output_0_layer, 32,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_0_norm2_Sub_output_0_chain,
  NULL, &_residual_blocks_0_norm2_Pow_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_output_0_Mul_layer, 31,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_0_norm2_ReduceMean_output_0_Mul_chain,
  NULL, &_residual_blocks_0_norm2_Sub_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _residual_blocks_0_norm2_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _residual_blocks_0_norm2_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _residual_blocks_0_norm2_ReduceMean_output_0_neutral_value_data, _residual_blocks_0_norm2_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_linear2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm2_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm2_ReduceMean_output_0_layer, 31,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_residual_blocks_0_norm2_ReduceMean_output_0_chain,
  NULL, &_residual_blocks_0_norm2_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_residual_blocks_0_norm2_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_linear2_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_linear2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_linear2_Gemm_output_0_weights, &_residual_blocks_0_linear2_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_linear2_Gemm_output_0_layer, 30,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_residual_blocks_0_linear2_Gemm_output_0_chain,
  NULL, &_residual_blocks_0_norm2_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm1_Mul_output_0_output, &_residual_blocks_0_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_Mul_output_0_layer, 29,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_0_Mul_output_0_chain,
  NULL, &_residual_blocks_0_linear2_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_Sigmoid_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_Sigmoid_output_0_layer, 28,
  NL_TYPE, 0x0, NULL,
  nl, forward_sigmoid,
  &_residual_blocks_0_Sigmoid_output_0_chain,
  NULL, &_residual_blocks_0_Mul_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm1_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm1_Mul_output_0_scale, &_residual_blocks_0_norm1_Mul_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm1_Mul_output_0_layer, 27,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_0_norm1_Mul_output_0_chain,
  NULL, &_residual_blocks_0_Sigmoid_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm1_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm1_Sub_output_0_output, &_residual_blocks_0_norm1_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm1_Div_output_0_layer, 25,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_0_norm1_Div_output_0_chain,
  NULL, &_residual_blocks_0_norm1_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm1_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm1_Sqrt_output_0_layer, 24,
  NL_TYPE, 0x0, NULL,
  nl, forward_sqrt,
  &_residual_blocks_0_norm1_Sqrt_output_0_chain,
  NULL, &_residual_blocks_0_norm1_Div_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_1_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_layer, 23,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_chain,
  NULL, &_residual_blocks_0_norm1_Sqrt_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _residual_blocks_0_norm1_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _residual_blocks_0_norm1_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _residual_blocks_0_norm1_ReduceMean_1_output_0_neutral_value_data, _residual_blocks_0_norm1_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_1_output_0_layer, 23,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_residual_blocks_0_norm1_ReduceMean_1_output_0_chain,
  NULL, &_residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_residual_blocks_0_norm1_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm1_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm1_Sub_output_0_output, &_residual_blocks_0_norm1_Constant_output_0_2D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm1_Pow_output_0_layer, 20,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_0_norm1_Pow_output_0_chain,
  NULL, &_residual_blocks_0_norm1_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm1_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_linear1_Gemm_output_0_output, &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm1_Sub_output_0_layer, 18,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_residual_blocks_0_norm1_Sub_output_0_chain,
  NULL, &_residual_blocks_0_norm1_Pow_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_Mul_layer, 17,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_chain,
  NULL, &_residual_blocks_0_norm1_Sub_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _residual_blocks_0_norm1_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _residual_blocks_0_norm1_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _residual_blocks_0_norm1_ReduceMean_output_0_neutral_value_data, _residual_blocks_0_norm1_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_linear1_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_norm1_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_norm1_ReduceMean_output_0_layer, 17,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_residual_blocks_0_norm1_ReduceMean_output_0_chain,
  NULL, &_residual_blocks_0_norm1_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_residual_blocks_0_norm1_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_linear1_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_linear1_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_linear1_Gemm_output_0_weights, &_residual_blocks_0_linear1_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_linear1_Gemm_output_0_layer, 16,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_residual_blocks_0_linear1_Gemm_output_0_chain,
  NULL, &_residual_blocks_0_norm1_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _residual_blocks_0_shortcut_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_residual_blocks_0_shortcut_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_residual_blocks_0_shortcut_Gemm_output_0_weights, &_residual_blocks_0_shortcut_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _residual_blocks_0_shortcut_Gemm_output_0_layer, 15,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_residual_blocks_0_shortcut_Gemm_output_0_chain,
  NULL, &_residual_blocks_0_linear1_Gemm_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_Mul_output_0_output, &_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Mul_output_0_layer, 14,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Mul_output_0_chain,
  NULL, &_residual_blocks_0_shortcut_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sigmoid_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sigmoid_output_0_layer, 13,
  NL_TYPE, 0x0, NULL,
  nl, forward_sigmoid,
  &_Sigmoid_output_0_chain,
  NULL, &_Mul_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _norm_input_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_Mul_output_0_scale, &_norm_input_Mul_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _norm_input_Mul_output_0_layer, 12,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_norm_input_Mul_output_0_chain,
  NULL, &_Sigmoid_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _norm_input_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_Sub_output_0_output, &_norm_input_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _norm_input_Div_output_0_layer, 10,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_norm_input_Div_output_0_chain,
  NULL, &_norm_input_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _norm_input_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _norm_input_Sqrt_output_0_layer, 9,
  NL_TYPE, 0x0, NULL,
  nl, forward_sqrt,
  &_norm_input_Sqrt_output_0_chain,
  NULL, &_norm_input_Div_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_1_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_Mul_layer, 8,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_norm_input_ReduceMean_1_output_0_Mul_chain,
  NULL, &_norm_input_Sqrt_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _norm_input_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _norm_input_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _norm_input_ReduceMean_1_output_0_neutral_value_data, _norm_input_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _norm_input_ReduceMean_1_output_0_layer, 8,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_norm_input_ReduceMean_1_output_0_chain,
  NULL, &_norm_input_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_norm_input_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _norm_input_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_Sub_output_0_output, &_norm_input_Constant_output_0_2D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _norm_input_Pow_output_0_layer, 5,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_norm_input_Pow_output_0_chain,
  NULL, &_norm_input_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _norm_input_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_input_layer_Gemm_output_0_output, &_norm_input_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _norm_input_Sub_output_0_layer, 3,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_norm_input_Sub_output_0_chain,
  NULL, &_norm_input_Pow_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_norm_input_ReduceMean_output_0_Mul_scale, &_norm_input_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_Mul_layer, 2,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_norm_input_ReduceMean_output_0_Mul_chain,
  NULL, &_norm_input_Sub_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _norm_input_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _norm_input_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _norm_input_ReduceMean_output_0_neutral_value_data, _norm_input_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_input_layer_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_norm_input_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _norm_input_ReduceMean_output_0_layer, 2,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_norm_input_ReduceMean_output_0_chain,
  NULL, &_norm_input_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_norm_input_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _input_layer_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_input_layer_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_input_layer_Gemm_output_0_weights, &_input_layer_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _input_layer_Gemm_output_0_layer, 1,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_input_layer_Gemm_output_0_chain,
  NULL, &_norm_input_ReduceMean_output_0_layer, AI_STATIC, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 136048, 1, 1),
    136048, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 3076, 1, 1),
    3076, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &output_output),
  &_input_layer_Gemm_output_0_layer, 0xa9425283, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 136048, 1, 1),
      136048, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 3076, 1, 1),
      3076, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &output_output),
  &_input_layer_Gemm_output_0_layer, 0xa9425283, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_network_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    input_output_array.data = AI_PTR(g_network_activations_map[0] + 1004);
    input_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1004);
    _input_layer_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _input_layer_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _norm_input_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1004);
    _norm_input_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1004);
    _norm_input_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 1008);
    _norm_input_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1008);
    _norm_input_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _norm_input_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _norm_input_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _norm_input_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _norm_input_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1536);
    _norm_input_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1536);
    _norm_input_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _norm_input_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _norm_input_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 516);
    _norm_input_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 516);
    _norm_input_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _norm_input_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _norm_input_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _norm_input_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _Sigmoid_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _Sigmoid_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_0_shortcut_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_0_shortcut_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_0_linear1_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_linear1_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm1_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_0_norm1_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_0_norm1_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 516);
    _residual_blocks_0_norm1_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 516);
    _residual_blocks_0_norm1_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm1_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm1_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm1_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm1_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 3072);
    _residual_blocks_0_norm1_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 3072);
    _residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm1_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm1_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 4);
    _residual_blocks_0_norm1_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 4);
    _residual_blocks_0_norm1_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm1_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm1_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm1_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_Sigmoid_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_Sigmoid_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_linear2_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_linear2_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm2_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm2_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm2_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 4);
    _residual_blocks_0_norm2_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 4);
    _residual_blocks_0_norm2_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm2_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm2_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm2_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm2_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 3072);
    _residual_blocks_0_norm2_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 3072);
    _residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm2_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm2_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 4);
    _residual_blocks_0_norm2_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 4);
    _residual_blocks_0_norm2_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm2_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_norm2_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_norm2_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_Add_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_Add_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 2048);
    _residual_blocks_0_Sigmoid_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_Sigmoid_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_0_Mul_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_0_Mul_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_shortcut_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_1_shortcut_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_1_linear1_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_linear1_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm1_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_norm1_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_norm1_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 1028);
    _residual_blocks_1_norm1_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1028);
    _residual_blocks_1_norm1_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1032);
    _residual_blocks_1_norm1_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1032);
    _residual_blocks_1_norm1_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm1_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm1_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_norm1_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm1_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm1_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 516);
    _residual_blocks_1_norm1_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 516);
    _residual_blocks_1_norm1_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _residual_blocks_1_norm1_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _residual_blocks_1_norm1_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1032);
    _residual_blocks_1_norm1_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1032);
    _residual_blocks_1_Sigmoid_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_Sigmoid_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1544);
    _residual_blocks_1_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1544);
    _residual_blocks_1_linear2_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_linear2_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm2_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_norm2_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_norm2_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 1028);
    _residual_blocks_1_norm2_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1028);
    _residual_blocks_1_norm2_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1032);
    _residual_blocks_1_norm2_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1032);
    _residual_blocks_1_norm2_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm2_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm2_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_norm2_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm2_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_norm2_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 516);
    _residual_blocks_1_norm2_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 516);
    _residual_blocks_1_norm2_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _residual_blocks_1_norm2_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _residual_blocks_1_norm2_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1032);
    _residual_blocks_1_norm2_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1032);
    _residual_blocks_1_Add_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_Add_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _residual_blocks_1_Sigmoid_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_1_Sigmoid_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _residual_blocks_1_Mul_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _residual_blocks_1_Mul_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _attention_attention_0_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _attention_attention_0_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _attention_attention_1_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 256);
    _attention_attention_1_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 256);
    _attention_attention_2_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _attention_attention_2_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _attention_attention_3_Sigmoid_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _attention_attention_3_Sigmoid_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _Mul_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _Mul_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _output_layer_output_layer_0_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _output_layer_output_layer_0_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _output_layer_output_layer_1_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 256);
    _output_layer_output_layer_1_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 256);
    output_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    output_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_network_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    _norm_input_Constant_output_0_2D_array.format |= AI_FMT_FLAG_CONST;
    _norm_input_Constant_output_0_2D_array.data = AI_PTR(g_network_weights_map[0] + 0);
    _norm_input_Constant_output_0_2D_array.data_start = AI_PTR(g_network_weights_map[0] + 0);
    _residual_blocks_0_norm1_Constant_output_0_2D_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_norm1_Constant_output_0_2D_array.data = AI_PTR(g_network_weights_map[0] + 4);
    _residual_blocks_0_norm1_Constant_output_0_2D_array.data_start = AI_PTR(g_network_weights_map[0] + 4);
    _residual_blocks_0_norm2_Constant_output_0_2D_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_norm2_Constant_output_0_2D_array.data = AI_PTR(g_network_weights_map[0] + 8);
    _residual_blocks_0_norm2_Constant_output_0_2D_array.data_start = AI_PTR(g_network_weights_map[0] + 8);
    _residual_blocks_1_norm1_Constant_output_0_2D_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_norm1_Constant_output_0_2D_array.data = AI_PTR(g_network_weights_map[0] + 12);
    _residual_blocks_1_norm1_Constant_output_0_2D_array.data_start = AI_PTR(g_network_weights_map[0] + 12);
    _residual_blocks_1_norm2_Constant_output_0_2D_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_norm2_Constant_output_0_2D_array.data = AI_PTR(g_network_weights_map[0] + 16);
    _residual_blocks_1_norm2_Constant_output_0_2D_array.data_start = AI_PTR(g_network_weights_map[0] + 16);
    _input_layer_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _input_layer_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 20);
    _input_layer_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 20);
    _input_layer_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _input_layer_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 2580);
    _input_layer_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 2580);
    _norm_input_ReduceMean_output_0_Mul_scale_array.format |= AI_FMT_FLAG_CONST;
    _norm_input_ReduceMean_output_0_Mul_scale_array.data = AI_PTR(g_network_weights_map[0] + 3092);
    _norm_input_ReduceMean_output_0_Mul_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 3092);
    _norm_input_ReduceMean_output_0_Mul_bias_array.format |= AI_FMT_FLAG_CONST;
    _norm_input_ReduceMean_output_0_Mul_bias_array.data = AI_PTR(g_network_weights_map[0] + 3096);
    _norm_input_ReduceMean_output_0_Mul_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 3096);
    _norm_input_ReduceMean_1_output_0_Mul_bias_array.format |= AI_FMT_FLAG_CONST;
    _norm_input_ReduceMean_1_output_0_Mul_bias_array.data = AI_PTR(g_network_weights_map[0] + 3100);
    _norm_input_ReduceMean_1_output_0_Mul_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 3100);
    _norm_input_Mul_output_0_scale_array.format |= AI_FMT_FLAG_CONST;
    _norm_input_Mul_output_0_scale_array.data = AI_PTR(g_network_weights_map[0] + 3104);
    _norm_input_Mul_output_0_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 3104);
    _norm_input_Mul_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _norm_input_Mul_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 3616);
    _norm_input_Mul_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 3616);
    _residual_blocks_0_shortcut_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_shortcut_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 4192);
    _residual_blocks_0_shortcut_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 4128);
    _residual_blocks_0_shortcut_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_shortcut_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 20576);
    _residual_blocks_0_shortcut_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 20576);
    _residual_blocks_0_linear1_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_linear1_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 21664);
    _residual_blocks_0_linear1_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 21600);
    _residual_blocks_0_linear1_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_linear1_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 38048);
    _residual_blocks_0_linear1_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 38048);
    _residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale_array.data = AI_PTR(g_network_weights_map[0] + 39072);
    _residual_blocks_0_norm1_ReduceMean_output_0_Mul_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 39072);
    _residual_blocks_0_norm1_Mul_output_0_scale_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_norm1_Mul_output_0_scale_array.data = AI_PTR(g_network_weights_map[0] + 39076);
    _residual_blocks_0_norm1_Mul_output_0_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 39076);
    _residual_blocks_0_norm1_Mul_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_norm1_Mul_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 40100);
    _residual_blocks_0_norm1_Mul_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 40100);
    _residual_blocks_0_linear2_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_linear2_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 41188);
    _residual_blocks_0_linear2_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 41124);
    _residual_blocks_0_linear2_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_linear2_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 73956);
    _residual_blocks_0_linear2_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 73956);
    _residual_blocks_0_norm2_Mul_output_0_scale_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_norm2_Mul_output_0_scale_array.data = AI_PTR(g_network_weights_map[0] + 74980);
    _residual_blocks_0_norm2_Mul_output_0_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 74980);
    _residual_blocks_0_norm2_Mul_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_0_norm2_Mul_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 76004);
    _residual_blocks_0_norm2_Mul_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 76004);
    _residual_blocks_1_shortcut_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_shortcut_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 77092);
    _residual_blocks_1_shortcut_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 77028);
    _residual_blocks_1_shortcut_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_shortcut_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 93476);
    _residual_blocks_1_shortcut_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 93476);
    _residual_blocks_1_linear1_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_linear1_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 94052);
    _residual_blocks_1_linear1_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 93988);
    _residual_blocks_1_linear1_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_linear1_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 110436);
    _residual_blocks_1_linear1_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 110436);
    _residual_blocks_1_norm1_Mul_output_0_scale_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_norm1_Mul_output_0_scale_array.data = AI_PTR(g_network_weights_map[0] + 110948);
    _residual_blocks_1_norm1_Mul_output_0_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 110948);
    _residual_blocks_1_norm1_Mul_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_norm1_Mul_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 111460);
    _residual_blocks_1_norm1_Mul_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 111460);
    _residual_blocks_1_linear2_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_linear2_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 112036);
    _residual_blocks_1_linear2_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 111972);
    _residual_blocks_1_linear2_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_linear2_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 120228);
    _residual_blocks_1_linear2_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 120228);
    _residual_blocks_1_norm2_Mul_output_0_scale_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_norm2_Mul_output_0_scale_array.data = AI_PTR(g_network_weights_map[0] + 120740);
    _residual_blocks_1_norm2_Mul_output_0_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 120740);
    _residual_blocks_1_norm2_Mul_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _residual_blocks_1_norm2_Mul_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 121252);
    _residual_blocks_1_norm2_Mul_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 121252);
    _attention_attention_0_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _attention_attention_0_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 121828);
    _attention_attention_0_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 121764);
    _attention_attention_0_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _attention_attention_0_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 125924);
    _attention_attention_0_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 125924);
    _attention_attention_2_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _attention_attention_2_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 126244);
    _attention_attention_2_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 126180);
    _attention_attention_2_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _attention_attention_2_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 130340);
    _attention_attention_2_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 130340);
    _output_layer_output_layer_0_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _output_layer_output_layer_0_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 130916);
    _output_layer_output_layer_0_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 130852);
    _output_layer_output_layer_0_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _output_layer_output_layer_0_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 135012);
    _output_layer_output_layer_0_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 135012);
    output_weights_array.format |= AI_FMT_FLAG_CONST;
    output_weights_array.data = AI_PTR(g_network_weights_map[0] + 135268);
    output_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 135268);
    output_bias_array.format |= AI_FMT_FLAG_CONST;
    output_bias_array.data = AI_PTR(g_network_weights_map[0] + 136036);
    output_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 136036);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 269715,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xa9425283,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_network_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 269715,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xa9425283,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_network_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_network_create(network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_network_data_params_get(&params) != true) {
    err = ai_network_get_error(*network);
    return err;
  }
#if defined(AI_NETWORK_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_NETWORK_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_network_init(*network, &params) != true) {
    err = ai_network_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_network_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_network_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= network_configure_weights(net_ctx, params);
  ok &= network_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

