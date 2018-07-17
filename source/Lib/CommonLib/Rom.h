/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     Rom.h
    \brief    global variables & functions (header)
*/

#ifndef __ROM__
#define __ROM__

#include "CommonDef.h"
#include "Common.h"

#include "BinaryDecisionTree.h"

#include <stdio.h>
#include <iostream>


//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

Void         initROM();
Void         destroyROM();

void         generateTrafoBlockSizeScaling( SizeIndexInfo& sizeIdxInfo );

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

// flexible conversion from relative to absolute index
extern       UInt*  g_scanOrder     [SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
extern       UInt*  g_scanOrderPosXY[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1][2];

extern const Int g_quantScales   [SCALING_LIST_REM_NUM];          // Q(QP%6)
extern const Int g_invQuantScales[SCALING_LIST_REM_NUM];          // IQ(QP%6)

#if RExt__HIGH_PRECISION_FORWARD_TRANSFORM
static const Int g_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = { 14, 6 };
#else
static const Int g_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = {  6, 6 };
#endif

extern const TMatrixCoeff g_aiT2  [TRANSFORM_NUMBER_OF_DIRECTIONS][  2][  2];
extern const TMatrixCoeff g_aiT4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_aiT8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_aiT16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_aiT32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];
extern const TMatrixCoeff g_aiT64 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 64][ 64];
extern const TMatrixCoeff g_aiT128[TRANSFORM_NUMBER_OF_DIRECTIONS][128][128];

// ====================================================================================================================
// Luma QP to Chroma QP mapping
// ====================================================================================================================

static const Int chromaQPMappingTableSize = 58;

extern const UChar  g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize];


// ====================================================================================================================
// Scanning order & context mapping table
// ====================================================================================================================

extern const UInt   ctxIndMap4x4[4*4];

extern const UInt   g_uiGroupIdx[ MAX_TU_SIZE ];
extern const UInt   g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ];
extern const UInt   g_auiGoRiceRange[ MAX_GR_ORDER_RESIDUAL ];                  //!< maximum value coded with Rice codes

// ====================================================================================================================
// Intra prediction table
// ====================================================================================================================

extern const UChar  g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1];
extern const UChar  g_aucIntraModeNumFast_UseMPM   [MAX_CU_DEPTH];
extern const UChar  g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH];

extern const UChar  g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE];
extern const UChar  g_intraMode65to33AngMapping[NUM_INTRA_MODE];

extern const UChar  g_intraMode33to65AngMapping[36];

static const unsigned mpmCtx[NUM_INTRA_MODE] =
{ 1, 1,                                                                                              // PLANAR, DC
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, // HOR domain
  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3     // VER domain
};


// ====================================================================================================================
// Mode-Dependent DST Matrices
// ====================================================================================================================

#if HEVC_USE_4x4_DSTVII
extern const TMatrixCoeff g_as_DST_MAT_4 [TRANSFORM_NUMBER_OF_DIRECTIONS][4][4];

#endif
extern TMatrixCoeff g_aiTr2   [NUM_TRANS_TYPE][  2][  2];
extern TMatrixCoeff g_aiTr4   [NUM_TRANS_TYPE][  4][  4];
extern TMatrixCoeff g_aiTr8   [NUM_TRANS_TYPE][  8][  8];
extern TMatrixCoeff g_aiTr16  [NUM_TRANS_TYPE][ 16][ 16];
extern TMatrixCoeff g_aiTr32  [NUM_TRANS_TYPE][ 32][ 32];
extern TMatrixCoeff g_aiTr64  [NUM_TRANS_TYPE][ 64][ 64];
extern TMatrixCoeff g_aiTr128 [NUM_TRANS_TYPE][128][128];

#if INTRA_KLT_MATRIX
extern const TMatrixCoeff g_aiKLT8x8[64][64];
#endif

// ====================================================================================================================
// Decision tree templates
// ====================================================================================================================

enum SplitDecisionTree
{
  DTT_SPLIT_DO_SPLIT_DECISION = 0, // decision node
  DTT_SPLIT_NO_SPLIT          = 1, // end-node
  DTT_SPLIT_BT_HORZ           = 2, // end-node - id same as CU_HORZ_SPLIT
  DTT_SPLIT_BT_VERT           = 3, // end-node - id same as CU_VERT_SPLIT
  DTT_SPLIT_TT_HORZ           = 4, // end-node - id same as CU_TRIH_SPLIT
  DTT_SPLIT_TT_VERT           = 5, // end-node - id same as CU_TRIV_SPLIT
  DTT_SPLIT_HV_DECISION,           // decision node
  DTT_SPLIT_H_IS_BT_12_DECISION,   // decision node
  DTT_SPLIT_V_IS_BT_12_DECISION,   // decision node
};

// decision tree for multi-type tree split decision
extern const DecisionTreeTemplate g_mtSplitDTT;

// decision tree for QTBT split
extern const DecisionTreeTemplate g_qtbtSplitDTT;

#if HEVC_USE_PART_SIZE
enum PartSizeDecisionTree
{
  DTT_PS_2Nx2N  = 0,           // end-node - id same as in 'enum PartSize'
  DTT_PS_2NxN   = 1,           // end-node - id same as in 'enum PartSize'
  DTT_PS_Nx2N   = 2,           // end-node - id same as in 'enum PartSize'
  DTT_PS_NxN    = 3,           // end-node - id same as in 'enum PartSize'
  DTT_PS_2NxnU  = 4,           // end-node - id same as in 'enum PartSize'
  DTT_PS_2NxnD  = 5,           // end-node - id same as in 'enum PartSize'
  DTT_PS_nLx2N  = 6,           // end-node - id same as in 'enum PartSize'
  DTT_PS_nRx2N  = 7,           // end-node - id same as in 'enum PartSize'
  DTT_PS_IS_2Nx2N,             // decision node
  DTT_PS_IS_2Nx,               // decision node
  DTT_PS_IS_NOT_NxN,           // decision node
  DTT_PS_IS_2NxN,              // decision node
  DTT_PS_IS_Nx2N,              // decision node
  DTT_PS_IS_nRx2N,             // decision node
  DTT_PS_IS_2NxnD              // decision node
};

extern const DecisionTreeTemplate g_partSizeDTT;

#endif

// ====================================================================================================================
// Misc.
// ====================================================================================================================
extern SizeIndexInfo* gp_sizeIdxInfo;
extern int            g_BlockSizeTrafoScale           [MAX_CU_SIZE + 1][MAX_CU_SIZE + 1][2];
extern SChar          g_aucLog2                       [MAX_CU_SIZE + 1];
extern SChar          g_aucNextLog2        [MAX_CU_SIZE + 1];
extern SChar          g_aucPrevLog2        [MAX_CU_SIZE + 1];
extern const SChar    i2Log2Tab[257];

inline bool is34( const SizeType& size )
{
  return ( size & ( ( Int64 ) 1 << ( g_aucLog2[size] - 1 ) ) );
}

inline bool is58( const SizeType& size )
{
  return ( size & ( ( Int64 ) 1 << ( g_aucLog2[size] - 2 ) ) );
}

inline bool isNonLog2BlockSize( const Size& size )
{
  return ( ( 1 << g_aucLog2[size.width] ) != size.width ) || ( ( 1 << g_aucLog2[size.height] ) != size.height );
}

inline bool isNonLog2Size( const SizeType& size )
{
  return ( ( 1 << g_aucLog2[size] ) != size );
}

extern UnitScale     g_miScaling; // scaling object for motion scaling

/*! Sophisticated Trace-logging */
#if ENABLE_TRACING
#include "dtrace.h"
extern CDTrace* g_trace_ctx;
#endif

const TChar* nalUnitTypeToString(NalUnitType type);

#if HEVC_USE_SCALING_LISTS
extern const TChar *MatrixType   [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];
extern const TChar *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];

extern const Int g_quantTSDefault4x4   [4*4];
extern const Int g_quantIntraDefault8x8[8*8];
extern const Int g_quantInterDefault8x8[8*8];

extern const UInt g_scalingListSize [SCALING_LIST_SIZE_NUM];
extern const UInt g_scalingListSizeX[SCALING_LIST_SIZE_NUM];
#endif

extern MsgLevel g_verbosity;



//! \}

#endif  //__TCOMROM__

