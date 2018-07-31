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

/** \file     Rom.cpp
    \brief    global variables & functions
*/

#include "Rom.h"
#include "UnitTools.h"

#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iomanip>

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

#if ENABLE_TRACING
CDTrace *g_trace_ctx = NULL;
#endif


//! \ingroup CommonLib
//! \{

MsgLevel g_verbosity = VERBOSE;

const TChar* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
  case NAL_UNIT_CODED_SLICE_TRAIL_R:    return "TRAIL_R";
  case NAL_UNIT_CODED_SLICE_TRAIL_N:    return "TRAIL_N";
  case NAL_UNIT_CODED_SLICE_TSA_R:      return "TSA_R";
  case NAL_UNIT_CODED_SLICE_TSA_N:      return "TSA_N";
  case NAL_UNIT_CODED_SLICE_STSA_R:     return "STSA_R";
  case NAL_UNIT_CODED_SLICE_STSA_N:     return "STSA_N";
  case NAL_UNIT_CODED_SLICE_BLA_W_LP:   return "BLA_W_LP";
  case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_W_RADL";
  case NAL_UNIT_CODED_SLICE_BLA_N_LP:   return "BLA_N_LP";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_RADL_R:     return "RADL_R";
  case NAL_UNIT_CODED_SLICE_RADL_N:     return "RADL_N";
  case NAL_UNIT_CODED_SLICE_RASL_R:     return "RASL_R";
  case NAL_UNIT_CODED_SLICE_RASL_N:     return "RASL_N";
#if HEVC_VPS
  case NAL_UNIT_VPS:                    return "VPS";
#endif
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_FILLER_DATA:            return "FILLER";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  default:                              return "UNK";
  }
}

class ScanGenerator
{
private:
  UInt m_line, m_column;
  const UInt m_blockWidth, m_blockHeight;
  const UInt m_stride;
  const CoeffScanType m_scanType;

public:
  ScanGenerator(UInt blockWidth, UInt blockHeight, UInt stride, CoeffScanType scanType)
    : m_line(0), m_column(0), m_blockWidth(blockWidth), m_blockHeight(blockHeight), m_stride(stride), m_scanType(scanType)
  { }

  UInt GetCurrentX() const { return m_column; }
  UInt GetCurrentY() const { return m_line; }

  UInt GetNextIndex(UInt blockOffsetX, UInt blockOffsetY)
  {
    const UInt rtn = ((m_line + blockOffsetY) * m_stride) + m_column + blockOffsetX;

    //advance line and column to the next position
    switch (m_scanType)
    {
      //------------------------------------------------

      case SCAN_DIAG:

        if ((m_column == m_blockWidth - 1) || (m_line == 0)) //if we reach the end of a rank, go diagonally down to the next one
        {
          m_line += m_column + 1;
          m_column = 0;

          if (m_line >= m_blockHeight) //if that takes us outside the block, adjust so that we are back on the bottom row
          {
            m_column += m_line - (m_blockHeight - 1);
            m_line = m_blockHeight - 1;
          }
        }
        else
        {
          m_column++;
          m_line--;
        }
        break;

#if HEVC_USE_MDCS
      //------------------------------------------------
      case SCAN_HOR:

        if (m_column == m_blockWidth - 1)
        {
          m_line++;
          m_column = 0;
        }
        else
        {
          m_column++;
        }
        break;

      //------------------------------------------------

      case SCAN_VER:

        if (m_line == m_blockHeight - 1)
        {
          m_column++;
          m_line = 0;
        }
        else
        {
          m_line++;
        }
        break;

#endif
      //------------------------------------------------

      default:

        THROW("ERROR: Unknown scan type \"" << m_scanType << "\"in ScanGenerator::GetNextIndex");
        break;
    }

    return rtn;
  }
};

// initialize ROM variables
Void initROM()
{
  Int i, c;

#if RExt__HIGH_BIT_DEPTH_SUPPORT
  {
    c = 64;
    const Double s = sqrt((Double)c) * (64 << COM16_C806_TRANS_PREC);


    for (Int k = 0; k < c; k++)
    {
      for (Int n = 0; n < c; n++)
      {
        Double w0, v;
        const Double PI = 3.14159265358979323846;

        // DCT-II
        w0 = k == 0 ? sqrt(0.5) : 1;
        v = cos(PI*(n + 0.5)*k / c) * w0 * sqrt(2.0 / c);
        short sv = (short)(s * v + (v > 0 ? 0.5 : -0.5));
        if (g_aiT64[0][0][c*c + k*c + n] != sv)
        {
          msg(WARNING, "trap");
        }
      }
    }
  }
#endif





  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
  ::memset(g_aucLog2, 0, sizeof(g_aucLog2));
  c = 0;
  for( int i = 0, n = 0; i <= MAX_CU_SIZE; i++ )
  {
    g_aucNextLog2[i] = i <= 1 ? 0 : c + 1;

    if( i == ( 1 << n ) )
    {
      c = n;
      n++;
    }

    g_aucPrevLog2[i] = c;
    g_aucLog2    [i] = c;
  }

  c = 2; //for the 2x2 transforms if QTBT is on

  const Double PI = 3.14159265358979323846;

  for (i = 0; i < 7; i++)
  {
    TMatrixCoeff *iT = NULL;
    const Double s = sqrt((Double)c) * (64 << COM16_C806_TRANS_PREC);

    switch (i)
    {
    case 0: iT = g_aiTr2[0][0]; break;
    case 1: iT = g_aiTr4[0][0]; break;
    case 2: iT = g_aiTr8[0][0]; break;
    case 3: iT = g_aiTr16[0][0]; break;
    case 4: iT = g_aiTr32[0][0]; break;
    case 5: iT = g_aiTr64[0][0]; break;
    case 6: iT = g_aiTr128[0][0]; break;
    case 7: exit(0); break;
    }

    for (Int k = 0; k < c; k++)
    {
      for (Int n = 0; n < c; n++)
      {
        Double w0, w1, v;

        // DCT-II
        w0 = k == 0 ? sqrt(0.5) : 1;
        v = cos(PI*(n + 0.5)*k / c) * w0 * sqrt(2.0 / c);
        iT[DCT2*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DCT-V
        w0 = (k == 0) ? sqrt(0.5) : 1.0;
        w1 = (n == 0) ? sqrt(0.5) : 1.0;
        v = cos(PI*n*k / (c - 0.5)) * w0 * w1 * sqrt(2.0 / (c - 0.5));
        iT[DCT5*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DCT-VIII
        v = cos(PI*(k + 0.5)*(n + 0.5) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
        iT[DCT8*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DST-I
        v = sin(PI*(n + 1)*(k + 1) / (c + 1)) * sqrt(2.0 / (c + 1));
        iT[DST1*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DST-VII
        v = sin(PI*(k + 0.5)*(n + 1) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
        iT[DST7*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));
      }
    }
    c <<= 1;
  }
  gp_sizeIdxInfo = new SizeIndexInfoLog2();
  gp_sizeIdxInfo->init(MAX_CU_SIZE);


  generateTrafoBlockSizeScaling(*gp_sizeIdxInfo);

  SizeIndexInfoLog2 sizeInfo;
  sizeInfo.init(MAX_CU_SIZE);

  // initialize scan orders
  for (UInt blockHeightIdx = 0; blockHeightIdx < sizeInfo.numAllHeights(); blockHeightIdx++)
  {
    for (UInt blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++)
    {
      const UInt blockWidth  = sizeInfo.sizeFrom(blockWidthIdx);
      const UInt blockHeight = sizeInfo.sizeFrom(blockHeightIdx);
      const UInt totalValues = blockWidth * blockHeight;

      //--------------------------------------------------------------------------------------------------

      //non-grouped scan orders

      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        g_scanOrder     [SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx]    = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][0] = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][1] = new UInt[totalValues];

        ScanGenerator fullBlockScan(blockWidth, blockHeight, blockWidth, scanType);

        for (UInt scanPosition = 0; scanPosition < totalValues; scanPosition++)
        {
          const int rasterPos = fullBlockScan.GetNextIndex( 0, 0 );
          const int posY      = rasterPos / blockWidth;
          const int posX      = rasterPos - ( posY * blockWidth );
          g_scanOrder     [SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx]   [scanPosition] = rasterPos;
          g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][0][scanPosition] = posX;
          g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][1][scanPosition] = posY;
        }
      }

      if( blockWidthIdx >= sizeInfo.numWidths() || blockHeightIdx >= sizeInfo.numHeights() )
      {
        // size indizes greater than numIdxs are sizes than are only used when grouping - they will never come up as a block size - thus they can be skipped at this point
        for( UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++ )
        {
          g_scanOrder     [SCAN_GROUPED_4x4][scanTypeIndex][blockWidthIdx][blockHeightIdx]    = nullptr;
          g_scanOrderPosXY[SCAN_GROUPED_4x4][scanTypeIndex][blockWidthIdx][blockHeightIdx][0] = nullptr;
          g_scanOrderPosXY[SCAN_GROUPED_4x4][scanTypeIndex][blockWidthIdx][blockHeightIdx][1] = nullptr;

        }

        continue;
      }

      //--------------------------------------------------------------------------------------------------

      //grouped scan orders
      const UInt  log2CGWidth    = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;
      const UInt  log2CGHeight   = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;

      const UInt  groupWidth     = 1 << log2CGWidth;
      const UInt  groupHeight    = 1 << log2CGHeight;
      const UInt  widthInGroups  = blockWidth >> log2CGWidth;
      const UInt  heightInGroups = blockHeight >> log2CGHeight;

      const UInt  groupSize      = groupWidth    * groupHeight;
      const UInt  totalGroups    = widthInGroups * heightInGroups;

      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        g_scanOrder     [SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx]    = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][0] = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][1] = new UInt[totalValues];


        ScanGenerator fullBlockScan(widthInGroups, heightInGroups, groupWidth, scanType);

        for (UInt groupIndex = 0; groupIndex < totalGroups; groupIndex++)
        {
          const UInt groupPositionY  = fullBlockScan.GetCurrentY();
          const UInt groupPositionX  = fullBlockScan.GetCurrentX();
          const UInt groupOffsetX    = groupPositionX * groupWidth;
          const UInt groupOffsetY    = groupPositionY * groupHeight;
          const UInt groupOffsetScan = groupIndex     * groupSize;

          ScanGenerator groupScan(groupWidth, groupHeight, blockWidth, scanType);

          for (UInt scanPosition = 0; scanPosition < groupSize; scanPosition++)
          {
            const int rasterPos = groupScan.GetNextIndex( groupOffsetX, groupOffsetY );
            const int posY      = rasterPos / blockWidth;
            const int posX      = rasterPos - ( posY * blockWidth );

            g_scanOrder     [SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx]   [groupOffsetScan + scanPosition] = rasterPos;
            g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][0][groupOffsetScan + scanPosition] = posX;
            g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][1][groupOffsetScan + scanPosition] = posY;
          }

          fullBlockScan.GetNextIndex(0, 0);
        }
      }

      //--------------------------------------------------------------------------------------------------
    }
  }
}

Void destroyROM()
{
  unsigned numWidths = gp_sizeIdxInfo->numAllWidths();
  unsigned numHeights = gp_sizeIdxInfo->numAllHeights();

  for (UInt groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++)
  {
    for (UInt scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++)
    {
      for (UInt blockWidthIdx = 0; blockWidthIdx <= numWidths; blockWidthIdx++)
      {
        for (UInt blockHeightIdx = 0; blockHeightIdx <= numHeights; blockHeightIdx++)
        {
          delete[] g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx];
          g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx] = nullptr;

          delete[] g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][0];
          g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][0] = nullptr;

          delete[] g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][1];
          g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][1] = nullptr;

        }
      }
    }
  }

  delete gp_sizeIdxInfo;
  gp_sizeIdxInfo = nullptr;
}



void generateTrafoBlockSizeScaling(SizeIndexInfo& sizeIdxInfo)
{
  for (SizeType y = 0; y < sizeIdxInfo.numHeights(); y++)
  {
    for (SizeType x = 0; x < sizeIdxInfo.numWidths(); x++)
    {
      SizeType h = sizeIdxInfo.sizeFrom(y);
      SizeType w = sizeIdxInfo.sizeFrom(x);
      double factor = sqrt(h) * sqrt(w) / (double)(1 << ((g_aucLog2[h] + g_aucLog2[w]) / 2));

      g_BlockSizeTrafoScale[h][w][0] = ((int)(factor + 0.9) != 1) ? (int)(factor * (double)(1 << ADJ_QUANT_SHIFT)) : 1;
      g_BlockSizeTrafoScale[h][w][1] = ((int)(factor + 0.9) != 1) ? (int)((double)(1 << ADJ_DEQUANT_SHIFT) / factor + 0.5) : 1;
    }
  }
}

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

const Int g_quantScales[SCALING_LIST_REM_NUM] =
{
  26214,23302,20560,18396,16384,14564
};

const Int g_invQuantScales[SCALING_LIST_REM_NUM] =
{
  40,45,51,57,64,72
};

//--------------------------------------------------------------------------------------------------
//structures
//EMT transform coeficient variable
TMatrixCoeff g_aiTr2  [NUM_TRANS_TYPE][  2][  2];
TMatrixCoeff g_aiTr4  [NUM_TRANS_TYPE][  4][  4];
TMatrixCoeff g_aiTr8  [NUM_TRANS_TYPE][  8][  8];
TMatrixCoeff g_aiTr16 [NUM_TRANS_TYPE][ 16][ 16];
TMatrixCoeff g_aiTr32 [NUM_TRANS_TYPE][ 32][ 32];
TMatrixCoeff g_aiTr64 [NUM_TRANS_TYPE][ 64][ 64];
TMatrixCoeff g_aiTr128[NUM_TRANS_TYPE][128][128];

#if INTRA_KLT_MATRIX
extern TMatrixCoeff g_aiKLT8x8Row[8][8]=
{
{  26,  42,  55,  66,  74,  78,  78,  72,},
{  55,  82,  83,  58,  12, -37, -72, -78,},
{ -75, -77, -13,  63,  89,  45, -34, -77,},
{ -84, -34,  68,  70, -33, -89, -11,  76,},
{ -85,  29,  83, -43, -70,  61,  56, -65,},
{ -73,  75,   6, -80,  69,  15, -85,  56,},
{ -54,  87, -74,  25,  37, -78,  84, -42,},
{ -29,  56, -76,  85, -85,  73, -53,  23,},
};

extern TMatrixCoeff g_aiKLT8x8Col[8][8] =
{
{  26,  41,  54,  65,  74,  78,  79,  74,},
{  58,  82,  82,  57,  14, -34, -70, -80,},
{ -78, -75, -13,  61,  89,  47, -32, -79,},
{ -86, -30,  71,  70, -32, -88, -15,  74,},
{ -83,  33,  81, -45, -71,  60,  59, -65,},
{ -71,  78,   3, -81,  70,  15, -83,  54,},
{ -51,  87, -76,  28,  35, -79,  84, -40,},
{ -27,  55, -75,  85, -84,  74, -54,  23,},
};


extern TMatrixCoeff g_aiKLT16x4Row[16][16]=
{
{  13,  21,  28,  36,  44,  52,  60,  66,  73,  78,  81,  83,  84,  83,  79,  72,},
{ -43, -61, -73, -84, -87, -84, -75, -57, -36, -12,  13,  36,  57,  73,  80,  78,},
{  62,  82,  84,  73,  44,   5, -30, -58, -78, -84, -72, -40,   6,  51,  80,  87,},
{  71,  89,  67,  23, -34, -78, -86, -61, -12,  38,  78,  90,  55,  -4, -60, -83,},
{  76,  79,  19, -47, -88, -73,  -9,  54,  89,  71,  -6, -78, -88, -36,  38,  74,},
{  79,  59, -26, -87, -73,  17,  87,  70,  -7, -80, -76,  12,  82,  69, -16, -73,},
{  76,  37, -55, -78,   1,  82,  51, -44, -87, -21,  79,  73, -41, -99,  -8,  83,},
{  80,  15, -90, -46,  71,  74, -45, -91,  17,  91,   2, -83, -13,  86,  24, -66,},
{ -70,  12,  87, -14, -87,  23,  88, -28, -94,  35,  87, -50, -70,  70,  48, -63,},
{  71, -30, -79,  56,  55, -89,  -7,  97, -43, -70,  82,  17, -88,  39,  61, -55,},
{  71, -62, -44,  92, -29, -68,  92, -14, -81,  84,  -4, -69,  77,  -5, -71,  51,},
{  70, -83,   1,  77, -85,  28,  43, -84,  62,   6, -68,  87, -46, -36,  92, -54,},
{ -76, 102, -57, -12,  68, -83,  65, -14, -45,  74, -72,  38,  18, -70,  91, -49,},
{  58, -89,  89, -65,  21,  26, -64,  81, -74,  50, -11, -32,  67, -86,  87, -44,},
{ -33,  62, -78,  85, -77,  61, -35,   5,  25, -55,  76, -88,  89, -79,  64, -30,},
{  22, -39,  53, -65,  73, -81,  90, -92,  88, -81,  72, -62,  49, -37,  27, -12,},
};

extern TMatrixCoeff g_aiKLT16x4Col[4][4] =
{
{  43,  61,  72,  74,},
{  80,  60, -19, -78,},
{ -77,  48,  68, -60,},
{  46, -82,  79, -35,},
};

extern TMatrixCoeff g_aiKLT4x16Row[4][4]=
{
{  42,  62,  73,  74,},
{  76,  66, -21, -77,},
{ -80,  45,  67, -59,},
{  50, -79,  78, -40,},
};

extern TMatrixCoeff g_aiKLT4x16Col[16][16] =
{
{  13,  21,  29,  37,  45,  52,  60,  67,  73,  79,  82,  83,  83,  81,  78,  73,},
{ -44, -63, -76, -85, -88, -84, -73, -56, -33,  -8,  17,  39,  59,  72,  77,  75,},
{  64,  83,  84,  69,  40,   3, -35, -65, -83, -84, -65, -30,  12,  51,  78,  87,},
{  81,  90,  64,  16, -42, -82, -88, -54,   1,  53,  82,  78,  42,  -9, -58, -81,},
{  83,  72,  10, -56, -88, -61,   5,  67,  88,  50, -21, -79, -84, -35,  38,  87,},
{  86,  50, -34, -84, -59,  23,  85,  64, -20, -82, -66,  18,  85,  73, -13, -87,},
{  85,  26, -72, -73,  15,  87,  46, -57, -88,  -5,  87,  60, -42, -87, -14,  73,},
{  85,  -8, -93, -24,  84,  54, -66, -71,  41,  84, -15, -86, -15,  81,  40, -69,},
{  78, -38, -81,  38,  81, -47, -72,  50,  70, -53, -70,  57,  68, -66, -63,  69,},
{  69, -60, -52,  84,  20, -97,  25,  81, -61, -49,  84,  10, -90,  31,  71, -56,},
{ -67,  81,   9, -93,  71,  28, -95,  53,  50, -86,  18,  69, -71, -11,  75, -46,},
{  57, -86,  36,  44, -89,  66,   7, -75,  84, -22, -62,  94, -43, -45,  88, -46,},
{  50, -89,  74, -22, -42,  85, -83,  40,  26, -78,  87, -49, -18,  75, -85,  39,},
{ -41,  79, -87,  67, -29, -12,  51, -82,  90, -68,  21,  30, -73,  93, -79,  32,},
{ -29,  60, -81,  90, -83,  69, -51,  24,  12, -45,  67, -83,  88, -79,  59, -24,},
{  15, -32,  47, -58,  66, -74,  82, -88,  88, -85,  80, -73,  63, -50,  36, -15,},
};

extern TMatrixCoeff g_aiKLT16x8Row[16][16] =
{
{  16,  25,  32,  40,  48,  55,  62,  68,  74,  77,  80,  81,  81,  80,  77,  71,},
{ -43, -61, -73, -83, -85, -82, -72, -54, -32,  -8,  17,  40,  61,  76,  83,  80,},
{  65,  85,  85,  71,  41,   1, -34, -63, -81, -85, -69, -35,   9,  50,  76,  83,},
{  76,  89,  63,  15, -41, -80, -84, -56,  -6,  45,  81,  87,  50,  -8, -63, -83,},
{  73,  74,  17, -49, -85, -69,  -7,  58,  91,  67, -13, -82, -87, -36,  40,  80,},
{  78,  55, -27, -87, -71,  17,  87,  73,  -6, -82, -75,  12,  81,  71, -16, -74,},
{  76,  35, -55, -80,  -2,  85,  55, -45, -89, -16,  79,  67, -41, -97, -10,  82,},
{  79,  12, -88, -45,  70,  73, -45, -90,  18,  92,  -2, -86, -12,  86,  26, -68,},
{  77, -19, -91,  17,  88, -21, -90,  32,  91, -45, -78,  56,  62, -68, -45,  59,},
{ -73,  34,  75, -60, -45,  89,  -2, -91,  51,  62, -89, -10,  92, -41, -63,  55,},
{ -70,  64,  40, -92,  32,  66, -92,  18,  74, -87,  15,  66, -81,   7,  72, -52,},
{ -66,  82,  -8, -71,  88, -32, -43,  85, -64,  -4,  67, -87,  46,  37, -95,  55,},
{  71,-104,  63,   9, -68,  85, -64,  14,  41, -71,  74, -42, -18,  70, -89,  47,},
{  53, -87,  88, -65,  22,  24, -63,  82, -75,  50, -11, -33,  70, -87,  88, -44,},
{  32, -62,  78, -85,  78, -59,  34,  -4, -26,  55, -77,  89, -90,  79, -63,  29,},
{ -21,  39, -53,  66, -75,  83, -90,  92, -87,  80, -72,  62, -49,  37, -26,  11,},
};

extern TMatrixCoeff g_aiKLT16x8Col[8][8] =
{
{  29,  42,  54,  65,  73,  77,  78,  75,},
{ -67, -84, -78, -52, -10,  33,  68,  81,},
{  84,  68,   1, -65, -85, -44,  30,  81,},
{  86,  17, -79, -64,  36,  88,  17, -74,},
{  77, -43, -77,  53,  71, -63, -59,  62,},
{  64, -81,   1,  82, -76, -12,  83, -51,},
{ -48,  87, -77,  31,  32, -79,  85, -39,},
{  27, -55,  74, -84,  83, -75,  58, -24,},
};

extern TMatrixCoeff g_aiKLT8x16Row[8][8] =
{
{  29,  43,  54,  65,  73,  77,  78,  74,},
{ -62, -84, -79, -55, -12,  34,  69,  81,},
{  78,  75,   6, -64, -87, -47,  33,  78,},
{  81,  28, -74, -69,  32,  92,  14, -74,},
{  79, -29, -87,  47,  74, -66, -51,  59,},
{  70, -77,  -6,  83, -75, -10,  81, -52,},
{ -58,  87, -70,  26,  30, -73,  89, -47,},
{ -31,  58, -74,  83, -82,  72, -58,  27,},
};

extern TMatrixCoeff g_aiKLT8x16Col[16][16] =
{
{  17,  25,  33,  41,  48,  55,  62,  69,  74,  78,  80,  81,  80,  79,  75,  71,},
{ -45, -63, -76, -84, -86, -82, -70, -52, -29,  -4,  20,  43,  62,  74,  80,  78,},
{  69,  86,  84,  67,  37,   0, -39, -68, -85, -83, -63, -29,  12,  50,  76,  84,},
{  83,  88,  59,  10, -46, -83, -87, -51,   5,  56,  83,  77,  40, -11, -59, -82,},
{  86,  70,   6, -60, -89, -59,  10,  71,  87,  45, -26, -80, -82, -33,  38,  84,},
{  85,  45, -38, -84, -54,  28,  85,  60, -25, -85, -63,  23,  86,  72, -13, -89,},
{  88,  20, -77, -72,  19,  90,  43, -62, -85,   3,  88,  54, -46, -84, -12,  71,},
{  84, -15, -95, -16,  88,  45, -71, -65,  48,  82, -22, -87,  -9,  83,  37, -68,},
{  79, -45, -78,  46,  76, -53, -67,  57,  64, -63, -61,  63,  62, -68, -61,  69,},
{  65, -62, -45,  86,  11, -94,  33,  76, -68, -43,  89,   4, -92,  37,  71, -57,},
{ -64,  83,   2, -90,  75,  25, -94,  54,  46, -88,  25,  66, -74,  -7,  75, -46,},
{  54, -87,  40,  42, -88,  68,   3, -74,  87, -26, -57,  93, -48, -40,  87, -46,},
{ -47,  88, -77,  27,  40, -84,  83, -42, -22,  75, -90,  55,  14, -74,  86, -39,},
{ -40,  80, -90,  72, -35,  -8,  49, -79,  89, -70,  28,  22, -67,  91, -80,  33,},
{ -26,  56, -78,  89, -85,  72, -55,  30,   5, -37,  63, -81,  89, -82,  62, -26,},
{ -13,  29, -43,  54, -63,  73, -82,  87, -87,  85, -82,  76, -66,  53, -39,  16,},
};

extern TMatrixCoeff g_aiKLT16x16Row[16][16] =
{
{  17,  25,  32,  40,  48,  54,  60,  66,  72,  76,  79,  81,  82,  81,  78,  73,},
{ -47, -64, -76, -85, -86, -82, -71, -53, -31,  -7,  18,  40,  60,  74,  80,  77,},
{  67,  85,  84,  68,  36,  -4, -38, -66, -82, -84, -68, -35,   9,  51,  76,  83,},
{  78,  90,  61,   9, -46, -82, -82, -52,  -3,  46,  81,  86,  49,  -8, -62, -84,},
{  78,  74,  10, -57, -88, -65,   2,  64,  90,  61, -19, -83, -84, -33,  38,  78,},
{  75,  50, -34, -85, -63,  24,  91,  68, -15, -85, -73,  14,  85,  73, -16, -77,},
{  80,  29, -60, -77,   3,  87,  51, -51, -88, -12,  78,  65, -40, -97, -11,  82,},
{  78,   6, -87, -38,  71,  70, -49, -90,  23,  92,  -2, -86, -14,  89,  27, -69,},
{ -79,  23,  92, -23, -88,  27,  90, -37, -90,  50,  75, -56, -60,  65,  41, -54,},
{ -68,  36,  67, -58, -41,  87,  -4, -92,  57,  61, -95,  -7,  95, -43, -63,  55,},
{ -69,  68,  38, -98,  35,  68, -93,  22,  71, -90,  20,  63, -79,   9,  65, -46,},
{  62, -80,  12,  69, -89,  34,  39, -81,  67,  -4, -64,  89, -51, -35,  98, -57,},
{ -70, 106, -67,  -8,  69, -83,  62, -17, -37,  68, -72,  44,  12, -69,  92, -48,},
{  50, -84,  87, -66,  23,  24, -62,  80, -73,  50, -13, -34,  71, -89,  92, -46,},
{  32, -63,  78, -85,  79, -60,  34,  -4, -25,  55, -79,  90, -88,  76, -62,  28,},
{ -20,  38, -52,  65, -74,  82, -90,  92, -88,  81, -73,  61, -48,  36, -26,  11,},
};

extern TMatrixCoeff g_aiKLT16x16Col[16][16] =
{
{  19,  26,  34,  41,  48,  55,  61,  67,  72,  77,  79,  81,  81,  80,  77,  73,},
{ -50, -67, -79, -86, -87, -81, -68, -50, -28,  -4,  20,  42,  60,  72,  78,  77,},
{  72,  86,  83,  64,  31,  -6, -44, -72, -86, -82, -61, -27,  13,  49,  75,  85,},
{  87,  86,  54,   2, -52, -86, -85, -47,   9,  59,  84,  76,  39, -11, -58, -82,},
{  91,  67,  -2, -68, -91, -52,  19,  77,  85,  37, -31, -80, -78, -30,  37,  82,},
{  85,  37, -46, -83, -44,  38,  86,  52, -35, -87, -58,  29,  89,  71, -13, -91,},
{  86,  12, -80, -66,  27,  90,  35, -69, -82,  10,  89,  50, -49, -85, -12,  73,},
{  85, -23, -96,  -6,  92,  35, -76, -60,  55,  80, -28, -87,  -6,  81,  34, -64,},
{  76, -52, -73,  55,  70, -62, -61,  66,  56, -70, -54,  68,  57, -70, -59,  68,},
{  61, -64, -37,  87,   1, -91,  41,  69, -73, -36,  91,  -1, -95,  40,  73, -58,},
{  63, -88,   5,  90, -85, -16,  94, -63, -36,  87, -33, -58,  74,   2, -70,  44,},
{  48, -81,  43,  32, -83,  74,  -8, -69,  91, -34, -54,  97, -54, -39,  91, -48,},
{ -45,  88, -79,  30,  36, -83,  84, -45, -17,  72, -90,  58,  11, -74,  86, -39,},
{  38, -78,  89, -72,  38,   4, -46,  78, -90,  73, -30, -20,  65, -92,  81, -33,},
{  25, -56,  79, -89,  86, -74,  59, -34,  -1,  34, -61,  79, -87,  82, -62,  26,},
{  12, -28,  41, -52,  61, -70,  80, -86,  87, -87,  84, -78,  68, -55,  41, -17,},
};
#endif

//--------------------------------------------------------------------------------------------------
//coefficients
//--------------------------------------------------------------------------------------------------

const UChar g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize] =
{
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,29,30,31,32,33,33,34,34,35,35,36,36,37,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 }
};

// ====================================================================================================================
// Intra prediction
// ====================================================================================================================

const UChar g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1] =
{
  {3, 3, 3, 3, 2, 2},  //   4x4,   4x8,   4x16,   4x32,   4x64,   4x128,
  {3, 3, 3, 3, 3, 2},  //   8x4,   8x8,   8x16,   8x32,   8x64,   8x128,
  {3, 3, 3, 3, 3, 2},  //  16x4,  16x8,  16x16,  16x32,  16x64,  16x128,
  {3, 3, 3, 3, 3, 2},  //  32x4,  32x8,  32x16,  32x32,  32x64,  32x128,
  {2, 3, 3, 3, 3, 2},  //  64x4,  64x8,  64x16,  64x32,  64x64,  64x128,
  {2, 2, 2, 2, 2, 3},  // 128x4, 128x8, 128x16, 128x32, 128x64, 128x128,
};

const UChar g_aucIntraModeNumFast_UseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  8,  //   4x4
  8,  //   8x8
  3,  //  16x16
  3,  //  32x32
  3,  //  64x64
  3   // 128x128
};
const UChar g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  9,  //   4x4
  9,  //   8x8
  4,  //  16x16   33
  4,  //  32x32   33
  5,  //  64x64   33
  5   // 128x128
};

const UChar g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE] =
//                                                               H                                                               D                                                               V
//0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 2, 2, 2, 2, 2, 2, 2, 3,  4,  6,  8, 10, 12, 13, 14, 16, 18, 20, 22, 23, 24, 26, 28, 30, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 44, 44, 45, 46, 46, 46, 47, 48, 48, 48, 49, 50, 51, 52, 52, 52, 53, 54, 54, 54, 55, 56, 56, 56, 57, 58, 59, 60, DM_CHROMA_IDX };

extern const UChar  g_intraMode65to33AngMapping[NUM_INTRA_MODE] =
//                                                               H                                                               D                                                               V
//0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 2, 2, 3, 3, 4, 4, 5, 5,  6,  6,  7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 33, 34, DM_CHROMA_IDX };

extern const UChar g_intraMode33to65AngMapping[36] =
//                                   H                               D                               V
//0, 1, 2, 3, 4, 5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, DM
{ 0, 1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, DM_CHROMA_IDX };


// ====================================================================================================================
// Decision tree templates
// ====================================================================================================================

const DecisionTreeTemplate g_mtSplitDTT = compile(
  decision( DTT_SPLIT_DO_SPLIT_DECISION,
  /*0*/ DTT_SPLIT_NO_SPLIT,
  /*1*/ decision( DTT_SPLIT_HV_DECISION,
        /*0*/ decision( DTT_SPLIT_H_IS_BT_12_DECISION,
              /*0*/ DTT_SPLIT_TT_HORZ,
              /*1*/ DTT_SPLIT_BT_HORZ ),
        /*1*/ decision( DTT_SPLIT_V_IS_BT_12_DECISION,
              /*0*/ DTT_SPLIT_TT_VERT,
              /*1*/ DTT_SPLIT_BT_VERT ) ) ) );

#if HEVC_USE_PART_SIZE
const DecisionTreeTemplate g_partSizeDTT = compile(
  decision( DTT_PS_IS_2Nx2N,
  /*0*/ decision( DTT_PS_IS_2Nx,
        /*0*/ decision( DTT_PS_IS_NOT_NxN,
              /*0*/ DTT_PS_NxN,
              /*1*/ decision( DTT_PS_IS_Nx2N,
                    /*0*/ decision( DTT_PS_IS_nRx2N,
                          /*0*/ DTT_PS_nLx2N,
                          /*1*/ DTT_PS_nRx2N ),
                    /*1*/ DTT_PS_Nx2N ) ),
        /*1*/ decision( DTT_PS_IS_2NxN,
              /*0*/ decision( DTT_PS_IS_2NxnD,
                    /*0*/ DTT_PS_2NxnU,
                    /*1*/ DTT_PS_2NxnD ),
              /*1*/ DTT_PS_2NxN ) ),
  /*1*/ DTT_PS_2Nx2N ) );

#endif


// ====================================================================================================================
// Misc.
// ====================================================================================================================
SizeIndexInfo*           gp_sizeIdxInfo = NULL;
int                      g_BlockSizeTrafoScale[MAX_CU_SIZE + 1][MAX_CU_SIZE + 1][2];
SChar                    g_aucLog2    [MAX_CU_SIZE + 1];
SChar                    g_aucNextLog2[MAX_CU_SIZE + 1];
SChar                    g_aucPrevLog2[MAX_CU_SIZE + 1];

UnitScale g_miScaling( MIN_CU_LOG2, MIN_CU_LOG2 );


// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
UInt* g_scanOrder     [SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
UInt* g_scanOrderPosXY[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1][2];

const UInt ctxIndMap4x4[4 * 4] =
{
  0, 1, 4, 5,
  2, 3, 4, 5,
  6, 6, 8, 8,
  7, 7, 8, 8
};


const UInt g_uiMinInGroup[LAST_SIGNIFICANT_GROUPS] = { 0,1,2,3,4,6,8,12,16,24,32,48,64,96 };
const UInt g_uiGroupIdx[MAX_TU_SIZE] = { 0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11
,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12
,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13 };
const UInt g_auiGoRiceRange[MAX_GR_ORDER_RESIDUAL] =
{
  6, 5, 6, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION
};

#if HEVC_USE_SCALING_LISTS
const TChar *MatrixType[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
    "INTRA2X2_LUMA",
    "INTRA2X2_CHROMAU",
    "INTRA2X2_CHROMAV",
    "INTER2X2_LUMA",
    "INTER2X2_CHROMAU",
    "INTER2X2_CHROMAV"
  },
  {
    "INTRA4X4_LUMA",
    "INTRA4X4_CHROMAU",
    "INTRA4X4_CHROMAV",
    "INTER4X4_LUMA",
    "INTER4X4_CHROMAU",
    "INTER4X4_CHROMAV"
  },
  {
    "INTRA8X8_LUMA",
    "INTRA8X8_CHROMAU",
    "INTRA8X8_CHROMAV",
    "INTER8X8_LUMA",
    "INTER8X8_CHROMAU",
    "INTER8X8_CHROMAV"
  },
  {
    "INTRA16X16_LUMA",
    "INTRA16X16_CHROMAU",
    "INTRA16X16_CHROMAV",
    "INTER16X16_LUMA",
    "INTER16X16_CHROMAU",
    "INTER16X16_CHROMAV"
  },
  {
   "INTRA32X32_LUMA",
   "INTRA32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTRA32X32_CHROMAV_FROM16x16_CHROMAV",
   "INTER32X32_LUMA",
   "INTER32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTER32X32_CHROMAV_FROM16x16_CHROMAV"
  },
};

const TChar *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
  },
  {
  },
  {
  },
  {
    "INTRA16X16_LUMA_DC",
    "INTRA16X16_CHROMAU_DC",
    "INTRA16X16_CHROMAV_DC",
    "INTER16X16_LUMA_DC",
    "INTER16X16_CHROMAU_DC",
    "INTER16X16_CHROMAV_DC"
  },
  {
    "INTRA32X32_LUMA_DC",
    "INTRA32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTRA32X32_CHROMAV_DC_FROM16x16_CHROMAV",
    "INTER32X32_LUMA_DC",
    "INTER32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTER32X32_CHROMAV_DC_FROM16x16_CHROMAV"
  },
};

const Int g_quantTSDefault4x4[4 * 4] =
{
  16,16,16,16,
  16,16,16,16,
  16,16,16,16,
  16,16,16,16
};

const Int g_quantIntraDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,21,24,
  16,16,16,16,17,19,22,25,
  16,16,17,18,20,22,25,29,
  16,16,18,21,24,27,31,36,
  17,17,20,24,30,35,41,47,
  18,19,22,27,35,44,54,65,
  21,22,25,31,41,54,70,88,
  24,25,29,36,47,65,88,115
};

const Int g_quantInterDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,20,24,
  16,16,16,17,18,20,24,25,
  16,16,17,18,20,24,25,28,
  16,17,18,20,24,25,28,33,
  17,18,20,24,25,28,33,41,
  18,20,24,25,28,33,41,54,
  20,24,25,28,33,41,54,71,
  24,25,28,33,41,54,71,91
};

const UInt g_scalingListSize [SCALING_LIST_SIZE_NUM] = { 4, 16, 64, 256, 1024, 4096, 16384 };
const UInt g_scalingListSizeX[SCALING_LIST_SIZE_NUM] = { 2,  4,  8,  16,   32,   64,   128 };
#endif

const UChar g_NonMPM[257] = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8 };


//! \}
