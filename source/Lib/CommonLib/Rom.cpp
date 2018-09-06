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
const UChar g_intraDirToKLTIdx[35] = {KLTPLANARDC, KLTPLANARDC, KLTRU,  KLTRU,  KLTRU,  KLTRU,  KLTRU, KLTHOR, KLTHOR, KLTHOR, KLTHOR, KLTHOR, KLTHOR, KLTHOR, KLTDIAG, KLTDIAG, KLTDIAG, KLTDIAG, KLTDIAG, KLTDIAG, KLTDIAG, KLTDIAG, KLTDIAG, KLTVER, KLTVER , KLTVER , KLTVER , KLTVER , KLTVER , KLTVER, KLTLD, KLTLD, KLTLD, KLTLD, KLTLD };

extern TMatrixCoeff g_aiKLT4[KLT_NUM][4][4]=
{
{
{  37,  61,  75,  76,},
{  79,  64, -13, -77,},
{ -81,  41,  67, -60,},
{  48, -83,  78, -34,},
},

{
{  39,  60,  74,  77,},
{  81,  62, -14, -76,},
{ -80,  47,  65, -59,},
{  43, -82,  81, -35,},
},

{
{  40,  60,  73,  76,},
{  81,  61, -14, -77,},
{ -79,  44,  68, -60,},
{  46, -84,  78, -33,},
},

{
{  33,  61,  77,  75,},
{  67,  73,  -9, -80,},
{ -86,  25,  72, -57,},
{  58, -82,  72, -33,},
},

{
{  39,  59,  74,  77,},
{  80,  64, -15, -75,},
{ -79,  44,  68, -60,},
{  48, -83,  78, -35,},
},

{
{  38,  60,  74,  76,},
{  81,  63, -15, -75,},
{ -80,  47,  64, -60,},
{  45, -81,  80, -37,},
},

{
{  38,  61,  75,  75,},
{  74,  69, -17, -77,},
{ -82,  39,  69, -59,},
{  53, -80,  76, -37,},
}
};

extern TMatrixCoeff g_aiKLT4HP[KLT_NUM][4][4] =
{
{
{ 147, 243, 300, 303,},
{ 315, 257, -53,-307,},
{-324, 164, 269,-241,},
{ 191,-332, 312,-135,},
},

{
{ 156, 239, 295, 306,},
{ 324, 247, -54,-305,},
{-321, 188, 262,-235,},
{ 173,-330, 322,-141,},
},

{
{ 159, 241, 294, 304,},
{ 322, 246, -58,-308,},
{-315, 177, 273,-239,},
{ 184,-335, 313,-133,},
},

{
{ 132, 244, 309, 300,},
{ 270, 292, -35,-321,},
{-343, 100, 288,-227,},
{ 234,-328, 287,-133,},
},

{
{ 155, 238, 295, 308,},
{ 319, 256, -60,-302,},
{-316, 176, 273,-239,},
{ 191,-330, 312,-140,},
},

{
{ 150, 240, 298, 306,},
{ 324, 253, -60,-299,},
{-321, 187, 258,-240,},
{ 179,-325, 321,-146,},
},

{
{ 151, 245, 299, 300,},
{ 296, 276, -68,-307,},
{-326, 154, 276,-237,},
{ 213,-320, 303,-149,},
}
};

extern TMatrixCoeff g_aiKLT8[KLT_NUM][8][8]=
{
{
{  26,  41,  55,  66,  74,  78,  78,  73,},
{ -66, -84, -79, -53, -10,  35,  68,  79,},
{  85,  70,   2, -65, -84, -42,  33,  82,},
{  86,  18, -76, -63,  36,  89,  17, -77,},
{  79, -42, -78,  52,  71, -61, -58,  62,},
{  64, -81,   3,  82, -76, -15,  83, -50,},
{ -47,  85, -76,  28,  34, -81,  86, -40,},
{  27, -57,  77, -85,  83, -73,  54, -22,},
},

{
{  28,  41,  53,  65,  73,  78,  79,  75,},
{ -69, -87, -79, -51,  -8,  34,  66,  78,},
{  88,  65,  -5, -69, -83, -38,  33,  81,},
{  88,   7, -81, -55,  42,  86,  15, -77,},
{  76, -51, -70,  60,  65, -66, -58,  64,},
{  59, -85,  14,  78, -81,  -9,  81, -49,},
{ -42,  83, -83,  40,  25, -78,  87, -39,},
{  23, -51,  72, -83,  85, -79,  60, -24,},
},

{
{  28,  42,  54,  64,  72,  77,  79,  76,},
{ -65, -84, -79, -54, -12,  33,  68,  80,},
{  83,  69,   3, -64, -86, -47,  29,  82,},
{  89,  17, -76, -63,  31,  87,  21, -76,},
{  79, -44, -77,  50,  71, -58, -62,  63,},
{  62, -81,   3,  84, -74, -20,  85, -49,},
{ -46,  86, -78,  28,  38, -83,  83, -36,},
{  26, -57,  76, -86,  84, -73,  53, -21,},
},

{
{  19,  36,  52,  66,  76,  81,  80,  73,},
{ -40, -72, -85, -68, -25,  28,  72,  86,},
{  61,  86,  40, -41, -87, -59,  23,  82,},
{  77,  60, -47, -83,   7,  89,  26, -75,},
{  88,   1, -91,  20,  83, -46, -63,  58,},
{  86, -63, -26,  85, -58, -28,  83, -48,},
{ -68,  87, -64,  15,  42, -80,  82, -39,},
{  37, -60,  77, -86,  82, -69,  51, -21,},
},

{
{  28,  41,  53,  63,  72,  77,  80,  77,},
{ -63, -83, -80, -56, -14,  31,  68,  81,},
{  80,  73,   7, -64, -88, -47,  31,  77,},
{  88,  22, -78, -66,  35,  87,  15, -72,},
{  79, -38, -79,  52,  68, -65, -56,  64,},
{ -66,  80,  -1, -81,  77,  10, -82,  52,},
{ -50,  86, -77,  33,  30, -76,  86, -42,},
{  27, -54,  72, -83,  84, -76,  59, -26,},
},

{
{  27,  41,  53,  65,  73,  78,  79,  75,},
{ -65, -84, -81, -54, -11,  34,  67,  78,},
{  88,  69,  -1, -67, -84, -40,  32,  78,},
{  90,  11, -80, -57,  43,  85,  10, -75,},
{  76, -50, -71,  61,  62, -68, -56,  65,},
{  62, -86,  15,  75, -80,  -6,  81, -52,},
{ -42,  81, -81,  39,  25, -77,  89, -43,},
{  23, -51,  72, -84,  86, -78,  58, -24,},
},

{
{  26,  42,  55,  66,  74,  78,  79,  73,},
{  56,  82,  82,  58,  13, -35, -71, -79,},
{ -76, -76, -13,  62,  89,  46, -33, -78,},
{ -85, -32,  69,  70, -32, -89, -13,  75,},
{ -84,  31,  82, -44, -70,  61,  57, -65,},
{ -72,  77,   5, -80,  70,  15, -84,  55,},
{ -53,  87, -75,  26,  36, -79,  84, -41,},
{ -28,  56, -75,  85, -85,  73, -53,  23,},
}
};

extern TMatrixCoeff g_aiKLT8HP[KLT_NUM][8][8] =
{
{
{ 105, 165, 220, 265, 296, 312, 311, 293,},
{-263,-337,-318,-212, -41, 139, 272, 318,},
{ 338, 278,   9,-260,-336,-166, 131, 328,},
{ 344,  70,-303,-251, 142, 357,  69,-307,},
{ 315,-168,-313, 206, 284,-244,-232, 249,},
{ 255,-324,  11, 329,-302, -58, 333,-200,},
{-188, 340,-303, 112, 137,-324, 346,-160,},
{ 109,-229, 306,-340, 333,-291, 217, -89,},
},

{
{ 113, 165, 214, 258, 292, 311, 315, 300,},
{-278,-346,-317,-203, -33, 136, 262, 312,},
{ 352, 260, -20,-277,-333,-154, 132, 322,},
{ 354,  29,-323,-221, 166, 344,  59,-308,},
{ 305,-203,-280, 239, 258,-263,-232, 255,},
{ 237,-342,  58, 313,-325, -35, 323,-197,},
{-166, 331,-330, 160,  99,-313, 346,-157,},
{  91,-204, 287,-332, 342,-314, 238, -97,},
},

{
{ 114, 169, 216, 256, 288, 310, 316, 302,},
{-261,-335,-318,-214, -47, 133, 271, 322,},
{ 332, 274,  14,-256,-344,-186, 117, 326,},
{ 355,  70,-304,-252, 124, 349,  85,-305,},
{ 318,-176,-306, 201, 284,-230,-247, 251,},
{ 246,-322,  14, 336,-295, -81, 339,-196,},
{-185, 342,-311, 112, 152,-332, 331,-145,},
{ 106,-226, 305,-343, 338,-291, 211, -85,},
},

{
{  75, 145, 209, 264, 304, 323, 321, 290,},
{-158,-290,-341,-274,-102, 112, 287, 345,},
{ 246, 344, 158,-163,-349,-237,  90, 328,},
{ 309, 239,-188,-331,  26, 354, 104,-300,},
{ 351,   5,-365,  79, 331,-184,-251, 234,},
{ 345,-252,-106, 342,-234,-112, 331,-194,},
{-271, 350,-254,  58, 168,-319, 327,-154,},
{ 147,-241, 308,-343, 327,-278, 202, -85,},
},

{
{ 111, 165, 213, 254, 288, 310, 319, 308,},
{-251,-332,-321,-225, -57, 125, 271, 324,},
{ 320, 293,  29,-254,-352,-190, 122, 309,},
{ 351,  90,-310,-263, 138, 350,  60,-287,},
{ 316,-153,-316, 207, 272,-261,-225, 256,},
{-266, 322,  -5,-324, 306,  41,-327, 206,},
{-202, 345,-309, 133, 118,-303, 343,-167,},
{ 109,-217, 289,-331, 338,-304, 235,-103,},
},

{
{ 110, 163, 214, 259, 293, 311, 316, 300,},
{-260,-337,-323,-218, -45, 136, 270, 314,},
{ 351, 278,  -3,-267,-336,-161, 130, 311,},
{ 361,  44,-321,-226, 173, 342,  40,-298,},
{ 304,-198,-282, 243, 250,-272,-223, 262,},
{ 249,-344,  59, 300,-320, -23, 323,-208,},
{-166, 325,-325, 158,  99,-309, 355,-171,},
{  93,-204, 287,-338, 345,-310, 231, -95,},
},

{
{ 105, 167, 219, 262, 295, 313, 315, 292,},
{ 226, 327, 329, 230,  53,-141,-285,-317,},
{-306,-305, -52, 247, 355, 184,-132,-311,},
{-340,-129, 278, 282,-130,-354, -52, 301,},
{-337, 124, 328,-177,-281, 242, 229,-260,},
{-288, 307,  19,-322, 279,  59,-337, 220,},
{-211, 347,-299, 105, 144,-315, 336,-164,},
{-112, 224,-302, 341,-339, 293,-213,  90,},
}
};

extern TMatrixCoeff g_aiKLT16[KLT_NUM][16][16] =
{
{
{  19,  28,  36,  44,  51,  58,  64,  69,  74,  77,  79,  79,  79,  77,  74,  70,},
{ -53, -69, -81, -86, -85, -78, -65, -45, -22,   1,  25,  45,  62,  73,  78,  76,},
{  73,  85,  81,  61,  28, -11, -48, -75, -86, -80, -58, -23,  17,  53,  77,  84,},
{  83,  83,  51,   0, -51, -84, -82, -46,   8,  58,  87,  81,  42, -14, -62, -84,},
{  88,  67,   2, -64, -90, -56,  12,  72,  86,  45, -28, -81, -81, -30,  39,  82,},
{  84,  40, -41, -86, -52,  35,  90,  57, -29, -89, -62,  26,  87,  68, -15, -83,},
{  87,   9, -76, -69,  24,  91,  37, -63, -81,   5,  84,  54, -46, -89, -15,  80,},
{  86, -21, -90, -16,  82,  46, -64, -68,  43,  83, -17, -88, -11,  89,  38, -75,},
{  83, -55, -78,  51,  75, -52, -72,  57,  70, -58, -65,  56,  62, -62, -56,  63,},
{  64, -64, -38,  80,   7, -89,  33,  78, -69, -47,  91,   9, -94,  32,  75, -59,},
{  57, -82,   2,  86, -68, -36,  97, -45, -58,  92, -21, -69,  75,   8, -76,  46,},
{  47, -83,  40,  45, -91,  60,  18, -83,  81, -12, -67,  91, -38, -48,  91, -47,},
{  44, -90,  80, -16, -54,  88, -76,  28,  33, -77,  82, -44, -23,  79, -87,  38,},
{  35, -77,  92, -73,  29,  20, -59,  81, -81,  56, -14, -36,  76, -93,  81, -34,},
{ -23,  58, -82,  90, -83,  65, -41,  12,  19, -50,  75, -89,  89, -77,  56, -22,},
{  13, -32,  50, -62,  71, -79,  87, -90,  89, -85,  78, -68,  55, -41,  27, -10,},
},

{
{  20,  27,  35,  42,  48,  55,  61,  66,  71,  75,  78,  80,  81,  80,  78,  74,},
{ -55, -71, -82, -88, -88, -80, -66, -46, -24,   0,  23,  43,  59,  70,  75,  73,},
{  78,  88,  81,  58,  23, -17, -53, -77, -86, -79, -56, -22,  16,  51,  74,  82,},
{  90,  85,  46,  -9, -60, -86, -76, -36,  17,  63,  84,  75,  37, -15, -61, -86,},
{  96,  62, -13, -76, -89, -42,  30,  81,  80,  29, -37, -80, -75, -26,  37,  83,},
{  89,  26, -59, -85, -29,  56,  88,  37, -48, -89, -48,  36,  87,  63, -16, -85,},
{  89,  -7, -87, -50,  51,  85,  10, -78, -69,  27,  88,  41, -54, -85, -13,  79,},
{  82, -38, -85,  12,  86,  16, -78, -46,  63,  74, -34, -89,  -4,  89,  40, -76,},
{  74, -67, -61,  71,  57, -73, -52,  71,  54, -71, -55,  66,  59, -65, -59,  64,},
{  61, -80, -15,  90, -27, -80,  57,  59, -79, -29,  89,   1, -91,  31,  76, -57,},
{  52, -87,  28,  69, -88,   1,  85, -66, -35,  93, -36, -64,  79,   9, -81,  48,},
{  41, -82,  60,  15, -82,  84, -14, -71,  93, -32, -57,  93, -44, -44,  83, -41,},
{  34, -78,  84, -43, -21,  74, -87,  49,  17, -75,  92, -54, -19,  81, -89,  38,},
{  30, -74,  98, -90,  55,  -8, -40,  75, -85,  70, -31, -19,  64, -86,  73, -29,},
{ -16,  42, -66,  83, -89,  84, -70,  44, -10, -25,  58, -82,  91, -85,  61, -23,},
{   8, -21,  33, -44,  55, -68,  81, -88,  90, -89,  88, -81,  70, -56,  37, -14,},
},

{
{  20,  28,  35,  42,  48,  54,  60,  65,  70,  74,  78,  80,  82,  81,  80,  76,},
{ -54, -70, -81, -87, -86, -79, -67, -49, -27,  -4,  19,  40,  59,  72,  78,  76,},
{  66,  80,  80,  62,  31,  -7, -43, -72, -87, -86, -67, -33,   9,  50,  77,  86,},
{  81,  84,  56,   7, -47, -83, -86, -55,  -3,  49,  84,  84,  49,  -7, -58, -83,},
{  89,  69,   6, -60, -90, -64,   3,  67,  88,  52, -19, -77, -82, -34,  36,  82,},
{  92,  42, -44, -89, -53,  34,  88,  59, -26, -85, -62,  19,  82,  68, -11, -80,},
{  88,   9, -76, -66,  26,  88,  35, -64, -80,   3,  83,  57, -44, -91, -17,  83,},
{  88, -26, -88, -10,  82,  43, -64, -69,  40,  84, -10, -87, -18,  86,  44, -76,},
{  84, -60, -74,  56,  72, -55, -70,  55,  68, -54, -67,  50,  67, -55, -63,  64,},
{  63, -70, -30,  80,   1, -86,  33,  78, -66, -52,  88,  20, -96,  22,  81, -58,},
{  58, -88,  15,  79, -73, -27,  91, -39, -65,  88,  -7, -78,  69,  21, -82,  46,},
{  47, -87,  51,  37, -91,  63,  18, -84,  79,  -6, -73,  86, -24, -57,  87, -42,},
{  36, -79,  79, -27, -45,  87, -79,  25,  44, -89,  86, -35, -35,  83, -82,  35,},
{  31, -75,  97, -83,  38,  18, -64,  86, -81,  53,  -8, -39,  74, -86,  69, -27,},
{ -19,  50, -76,  87, -81,  66, -42,  13,  19, -52,  78, -93,  93, -80,  57, -22,},
{  11, -30,  48, -63,  75, -86,  91, -90,  86, -81,  74, -66,  54, -40,  25,  -9,},
},

{
{  10,  18,  27,  35,  43,  51,  59,  66,  72,  78,  82,  85,  85,  83,  79,  73,},
{ -24, -45, -65, -80, -88, -88, -81, -65, -43, -18,   9,  36,  59,  75,  83,  81,},
{  37,  66,  84,  84,  64,  29, -14, -52, -81, -90, -77, -43,   1,  44,  76,  88,},
{  50,  83,  84,  50,  -7, -61, -89, -76, -30,  27,  75,  87,  58,   1, -57, -88,},
{  64,  90,  56, -15, -75, -84, -35,  38,  84,  72,   5, -67, -88, -45,  31,  85,},
{  74,  82,  11, -71, -83, -12,  71,  82,  10, -74, -79,  -1,  74,  74,  -4, -81,},
{  79,  62, -38, -87, -21,  75,  67, -31, -88, -27,  71,  72, -25, -92, -28,  81,},
{  89,  36, -78, -58,  57,  75, -33, -84,   8,  87,  12, -84, -31,  80,  50, -73,},
{  92,   0, -93,   7,  90, -14, -90,  24,  87, -32, -79,  39,  71, -49, -64,  60,},
{  87, -32, -74,  68,  41, -92,   6,  90, -50, -64,  79,  26, -86,  14,  73, -50,},
{  83, -64, -30,  90, -47, -51,  88, -25, -67,  82,  -5, -79,  70,  23, -86,  48,},
{  80, -87,  20,  58, -87,  46,  31, -83,  68,   3, -71,  83, -25, -58,  89, -43,},
{  63, -87,  62,  -5, -57,  88, -73,  20,  43, -85,  82, -33, -37,  85, -81,  34,},
{  51, -82,  87, -67,  25,  24, -66,  88, -83,  51,  -4, -44,  79, -88,  68, -27,},
{ -36,  63, -81,  90, -84,  67, -41,  11,  21, -50,  73, -87,  88, -75,  52, -20,},
{  19, -34,  47, -61,  71, -79,  85, -88,  87, -84,  79, -71,  58, -43,  26,  -9,},
},

{
{  22,  30,  36,  43,  49,  55,  61,  66,  71,  75,  78,  79,  81,  80,  78,  75,},
{ -53, -70, -80, -85, -84, -78, -67, -49, -27,  -4,  20,  41,  60,  73,  80,  79,},
{  69,  84,  81,  62,  30,  -8, -44, -71, -86, -86, -66, -32,  10,  48,  75,  84,},
{  80,  85,  55,   3, -51, -85, -85, -52,   1,  52,  83,  83,  47,  -7, -59, -83,},
{  84,  67,   5, -61, -90, -63,   6,  67,  87,  53, -22, -81, -84, -34,  38,  83,},
{  81,  44, -38, -86, -59,  27,  90,  65, -22, -88, -68,  21,  85,  69, -14, -79,},
{  94,  14, -78, -72,  22,  93,  37, -64, -82,   3,  83,  52, -44, -82, -14,  72,},
{  87, -19, -89, -13,  81,  43, -66, -66,  47,  80, -23, -89,  -4,  92,  33, -75,},
{  83, -51, -79,  53,  73, -57, -67,  57,  68, -59, -63,  61,  59, -67, -54,  64,},
{  67, -66, -37,  81,   3, -85,  32,  75, -68, -46,  91,   7, -95,  36,  76, -61,},
{  54, -76,  -2,  86, -65, -39, 100, -46, -62,  94, -23, -69,  79,   2, -73,  46,},
{  45, -75,  33,  49, -93,  58,  24, -89,  83,  -9, -71,  93, -40, -44,  87, -46,},
{  52,-101,  87, -18, -56,  86, -74,  28,  33, -71,  73, -43, -15,  72, -87,  40,},
{  37, -77,  90, -73,  32,  14, -52,  73, -74,  56, -20, -29,  76,-100,  90, -40,},
{  23, -55,  77, -87,  82, -66,  43, -13, -22,  53, -77,  90, -90,  78, -56,  22,},
{ -13,  31, -47,  61, -71,  81, -91,  94, -91,  86, -76,  63, -50,  37, -25,  10,},
},

{
{  22,  29,  36,  43,  50,  56,  62,  67,  72,  76,  78,  80,  80,  79,  76,  72,},
{ -54, -71, -82, -88, -86, -78, -64, -45, -22,   2,  24,  44,  61,  72,  77,  75,},
{  73,  85,  80,  59,  25, -14, -50, -76, -87, -81, -59, -24,  16,  51,  76,  85,},
{  91,  86,  50,  -6, -59, -89, -81, -40,  15,  61,  83,  74,  36, -14, -58, -82,},
{  93,  62,  -9, -73, -88, -45,  27,  80,  81,  32, -38, -83, -77, -25,  40,  82,},
{  91,  30, -55, -85, -34,  51,  86,  40, -43, -86, -53,  32,  89,  67, -16, -88,},
{  94,  -4, -86, -56,  46,  88,  12, -74, -68,  20,  85,  45, -50, -87, -14,  79,},
{  79, -35, -85,   9,  90,  20, -81, -51,  64,  75, -32, -88,  -5,  87,  38, -74,},
{  71, -60, -62,  66,  59, -70, -58,  73,  59, -73, -57,  66,  59, -64, -59,  64,},
{  64, -76, -25,  88, -15, -85,  54,  62, -80, -31,  91,  -1, -89,  32,  74, -57,},
{  53, -83,  15,  75, -78, -13,  90, -64, -39,  96, -37, -64,  82,   3, -79,  48,},
{  46, -90,  61,  24, -88,  80,  -8, -70,  89, -30, -54,  88, -42, -43,  84, -43,},
{  37, -82,  84, -36, -30,  76, -83,  50,  11, -66,  87, -56, -16,  84, -94,  41,},
{  27, -66,  85, -75,  42,   4, -50,  78, -81,  60, -17, -38,  83,-101,  84, -34,},
{ -21,  55, -81,  93, -91,  77, -53,  22,  12, -45,  71, -85,  84, -71,  49, -19,},
{   8, -24,  39, -52,  62, -72,  84, -91,  93, -92,  86, -74,  60, -43,  26,  -9,},
},

{
{  18,  26,  33,  41,  48,  55,  61,  67,  72,  76,  79,  81,  81,  80,  78,  73,},
{ -48, -66, -78, -86, -86, -81, -69, -51, -29,  -5,  19,  41,  60,  73,  79,  77,},
{  70,  86,  84,  66,  34,  -5, -41, -69, -84, -83, -64, -31,  11,  50,  75,  84,},
{  82,  88,  57,   5, -49, -84, -84, -49,   4,  53,  83,  81,  44, -10, -60, -83,},
{  84,  71,   4, -63, -90, -58,  11,  71,  88,  49, -26, -82, -81, -31,  38,  81,},
{  80,  44, -39, -84, -54,  31,  89,  60, -25, -87, -66,  22,  87,  72, -16, -83,},
{  82,  21, -69, -72,  13,  89,  44, -59, -87,  -2,  85,  60, -44, -92, -12,  78,},
{  83,  -7, -94, -25,  82,  56, -61, -77,  37,  87, -13, -87, -11,  86,  31, -67,},
{  79, -37, -86,  39,  83, -45, -78,  53,  75, -60, -65,  61,  58, -68, -48,  60,},
{ -64,  49,  54, -73, -24,  92, -23, -84,  67,  49, -95,  -3,  95, -42, -67,  56,},
{ -65,  76,  19, -96,  57,  49, -97,  39,  59, -91,  26,  63, -79,   5,  68, -46,},
{  57, -84,  27,  57, -91,  52,  22, -79,  78, -16, -60,  91, -52, -36,  93, -52,},
{  58, -98,  72,  -8, -57,  85, -72,  27,  31, -72,  80, -50, -12,  73, -92,  45,},
{ -45,  83, -89,  69, -28, -18,  58, -80,  79, -57,  18,  30, -69,  91, -88,  41,},
{ -29,  61, -79,  88, -82,  65, -43,  15,  17, -48,  73, -87,  88, -78,  62, -27,},
{  16, -34,  48, -60,  70, -78,  87, -91,  88, -84,  77, -68,  55, -42,  31, -13,},
}
};

extern TMatrixCoeff g_aiKLT16HP[KLT_NUM][16][16] =
{
{
{  78, 111, 144, 174, 204, 231, 256, 277, 295, 307, 315, 317, 315, 308, 297, 281,},
{-213,-278,-323,-345,-341,-313,-258,-179, -89,   6,  99, 181, 248, 294, 314, 305,},
{ 294, 340, 324, 244, 113, -45,-193,-299,-345,-321,-230, -93,  66, 212, 306, 338,},
{ 334, 332, 203,   2,-206,-335,-327,-185,  32, 234, 348, 326, 170, -56,-248,-337,},
{ 354, 268,   8,-257,-359,-226,  48, 287, 345, 181,-111,-326,-323,-119, 155, 329,},
{ 335, 159,-162,-344,-210, 139, 359, 230,-117,-355,-247, 105, 349, 272, -60,-331,},
{ 347,  36,-305,-275,  96, 363, 150,-253,-324,  19, 338, 216,-184,-355, -60, 318,},
{ 343, -84,-359, -63, 326, 185,-254,-272, 172, 330, -67,-352, -43, 358, 152,-302,},
{ 330,-219,-310, 204, 300,-209,-288, 227, 278,-234,-259, 226, 246,-247,-226, 251,},
{ 256,-257,-152, 319,  29,-356, 133, 313,-274,-190, 364,  35,-376, 127, 302,-234,},
{ 229,-329,   7, 344,-271,-143, 390,-181,-232, 369, -86,-277, 300,  34,-303, 183,},
{ 189,-331, 160, 181,-365, 239,  74,-332, 325, -49,-266, 363,-154,-193, 363,-187,},
{ 175,-362, 320, -66,-216, 352,-306, 114, 133,-306, 329,-174, -92, 316,-348, 154,},
{ 138,-308, 368,-291, 117,  79,-237, 326,-322, 225, -55,-143, 304,-374, 322,-136,},
{ -93, 231,-328, 361,-330, 260,-163,  49,  76,-199, 299,-358, 358,-308, 223, -88,},
{  51,-129, 198,-249, 284,-317, 347,-359, 355,-338, 310,-270, 219,-164, 107, -39,},
},

{
{  81, 109, 138, 166, 194, 219, 243, 265, 285, 301, 314, 322, 324, 321, 313, 297,},
{-220,-283,-330,-354,-350,-320,-263,-185, -95,  -1,  91, 172, 237, 281, 300, 292,},
{ 311, 352, 323, 231,  91, -67,-211,-310,-346,-316,-224, -88,  66, 202, 294, 328,},
{ 362, 339, 182, -38,-239,-344,-306,-145,  70, 252, 338, 302, 149, -60,-245,-343,},
{ 385, 246, -52,-305,-354,-170, 119, 323, 319, 114,-150,-318,-298,-106, 150, 331,},
{ 357, 104,-236,-340,-118, 226, 352, 150,-192,-358,-192, 145, 349, 252, -65,-340,},
{ 358, -30,-349,-199, 202, 340,  38,-312,-275, 106, 351, 165,-215,-342, -51, 316,},
{ 328,-154,-340,  48, 344,  65,-313,-184, 251, 296,-137,-357, -15, 358, 161,-305,},
{ 294,-267,-244, 284, 227,-291,-210, 284, 214,-283,-219, 264, 238,-260,-237, 256,},
{ 245,-318, -61, 360,-107,-318, 228, 235,-316,-118, 357,   3,-364, 125, 303,-230,},
{ 206,-348, 111, 276,-352,   5, 340,-265,-139, 370,-144,-255, 317,  35,-326, 193,},
{ 164,-328, 241,  59,-330, 335, -55,-283, 374,-129,-226, 374,-178,-175, 334,-165,},
{ 137,-311, 335,-174, -85, 297,-347, 196,  69,-302, 368,-214, -75, 323,-355, 153,},
{ 122,-297, 393,-361, 220, -31,-159, 298,-341, 278,-125, -76, 255,-344, 292,-116,},
{ -63, 168,-264, 330,-354, 337,-278, 174, -40,-101, 232,-328, 366,-339, 244, -92,},
{  31, -83, 132,-174, 218,-273, 325,-354, 359,-358, 351,-326, 280,-223, 150, -56,},
},

{
{  81, 113, 142, 167, 193, 217, 240, 259, 280, 297, 311, 321, 326, 326, 318, 303,},
{-215,-281,-325,-346,-343,-316,-267,-195,-108, -16,  76, 161, 235, 288, 311, 305,},
{ 265, 321, 318, 248, 124, -28,-173,-288,-349,-344,-266,-133,  35, 199, 310, 343,},
{ 325, 337, 223,  26,-187,-332,-344,-220, -14, 196, 335, 338, 195, -28,-231,-332,},
{ 358, 275,  23,-240,-360,-254,  13, 267, 351, 207, -78,-307,-329,-138, 143, 327,},
{ 368, 167,-176,-358,-213, 136, 352, 236,-104,-341,-249,  78, 329, 273, -43,-321,},
{ 352,  35,-303,-264, 104, 350, 139,-256,-322,  10, 331, 227,-177,-364, -69, 332,},
{ 353,-103,-351, -42, 326, 173,-257,-277, 162, 335, -42,-348, -74, 346, 176,-304,},
{ 336,-239,-295, 225, 289,-220,-280, 218, 274,-216,-268, 201, 268,-222,-251, 255,},
{ 251,-280,-120, 321,   5,-345, 132, 313,-264,-208, 350,  80,-382,  88, 325,-233,},
{ 230,-350,  58, 317,-293,-108, 366,-154,-258, 351, -26,-314, 274,  86,-329, 185,},
{ 189,-349, 204, 147,-364, 251,  72,-335, 317, -23,-290, 345, -96,-228, 348,-169,},
{ 143,-316, 318,-107,-179, 349,-315,  99, 178,-356, 342,-139,-142, 332,-329, 140,},
{ 125,-299, 387,-333, 151,  74,-257, 345,-326, 212, -33,-155, 296,-343, 276,-109,},
{ -77, 201,-304, 347,-325, 262,-170,  53,  77,-206, 312,-372, 371,-321, 227, -86,},
{  43,-118, 191,-251, 299,-342, 365,-362, 344,-322, 298,-263, 214,-159, 100, -37,},
},

{
{  39,  72, 107, 140, 173, 204, 235, 263, 290, 312, 330, 339, 340, 332, 316, 290,},
{ -96,-181,-260,-318,-352,-353,-323,-261,-173, -72,  37, 143, 236, 299, 332, 323,},
{ 146, 264, 337, 334, 256, 115, -55,-210,-323,-361,-307,-172,   3, 177, 305, 351,},
{ 201, 332, 336, 198, -27,-243,-355,-304,-120, 109, 299, 350, 232,   4,-227,-354,},
{ 254, 360, 226, -61,-302,-335,-140, 151, 338, 287,  19,-267,-351,-178, 124, 340,},
{ 295, 326,  42,-284,-333, -49, 285, 329,  40,-295,-318,  -4, 297, 297, -16,-323,},
{ 316, 246,-151,-346, -85, 298, 269,-126,-353,-108, 283, 288,-101,-366,-111, 323,},
{ 356, 146,-312,-231, 226, 298,-131,-337,  31, 349,  46,-336,-126, 319, 198,-291,},
{ 366,  -1,-372,  27, 360, -58,-361,  95, 347,-129,-315, 155, 286,-194,-256, 241,},
{ 348,-129,-297, 272, 162,-367,  22, 358,-198,-255, 314, 102,-344,  56, 290,-199,},
{ 330,-255,-118, 359,-187,-204, 353,-100,-269, 328, -18,-315, 279,  92,-342, 193,},
{ 319,-346,  82, 233,-349, 183, 124,-331, 272,  11,-286, 333, -98,-233, 355,-171,},
{ 253,-350, 249, -22,-226, 352,-293,  81, 173,-340, 328,-133,-147, 339,-325, 136,},
{ 205,-327, 348,-270, 101,  98,-266, 354,-330, 205, -17,-177, 317,-351, 271,-107,},
{-144, 253,-324, 359,-337, 267,-164,  46,  84,-200, 290,-347, 351,-301, 209, -80,},
{  75,-134, 189,-243, 284,-315, 339,-352, 350,-337, 315,-283, 232,-171, 106, -38,},
},

{
{  88, 118, 145, 171, 196, 220, 243, 263, 282, 298, 310, 318, 322, 321, 314, 301,},
{-211,-279,-321,-339,-336,-311,-266,-197,-110, -16,  79, 166, 240, 291, 318, 315,},
{ 275, 336, 325, 248, 121, -31,-176,-285,-346,-343,-266,-127,  40, 193, 299, 337,},
{ 322, 342, 221,  14,-202,-341,-338,-207,   4, 207, 333, 330, 188, -30,-236,-333,},
{ 334, 270,  19,-244,-359,-250,  24, 267, 349, 210, -86,-322,-336,-136, 152, 330,},
{ 324, 177,-154,-346,-236, 109, 360, 259, -88,-350,-271,  83, 341, 275, -55,-316,},
{ 377,  55,-311,-290,  87, 373, 149,-256,-329,  13, 333, 209,-177,-329, -55, 288,},
{ 348, -75,-358, -53, 323, 172,-264,-264, 189, 321, -90,-357, -17, 367, 134,-299,},
{ 330,-204,-315, 212, 291,-227,-270, 227, 270,-238,-252, 245, 234,-269,-218, 255,},
{ 266,-262,-147, 322,  13,-342, 129, 301,-272,-186, 362,  28,-379, 145, 306,-246,},
{ 218,-302,  -7, 345,-260,-156, 401,-182,-246, 378, -92,-276, 315,   8,-290, 182,},
{ 179,-298, 132, 195,-372, 234,  95,-355, 333, -36,-285, 372,-161,-175, 348,-184,},
{ 209,-405, 350, -72,-222, 344,-297, 113, 131,-283, 292,-173, -61, 290,-346, 161,},
{ 149,-310, 362,-294, 129,  57,-208, 292,-297, 225, -79,-116, 304,-399, 360,-159,},
{  93,-220, 308,-348, 330,-266, 172, -51, -87, 212,-310, 362,-360, 310,-224,  90,},
{ -51, 122,-187, 243,-283, 323,-362, 375,-364, 344,-305, 252,-198, 149,-101,  40,},
},

{
{  88, 116, 144, 172, 200, 225, 249, 270, 288, 303, 313, 320, 321, 315, 304, 288,},
{-218,-282,-329,-352,-346,-312,-258,-179, -87,   6,  95, 175, 243, 288, 308, 300,},
{ 292, 338, 321, 237, 101, -57,-201,-305,-348,-323,-235, -96,  63, 205, 304, 339,},
{ 363, 345, 199, -24,-238,-354,-323,-159,  61, 243, 332, 296, 145, -56,-234,-326,},
{ 372, 248, -36,-292,-352,-178, 110, 318, 325, 127,-151,-333,-306,-102, 160, 328,},
{ 362, 120,-221,-342,-135, 203, 344, 158,-172,-345,-212, 128, 355, 267, -63,-350,},
{ 375, -15,-343,-224, 183, 351,  47,-298,-273,  81, 339, 178,-202,-346, -55, 317,},
{ 317,-138,-342,  37, 359,  82,-323,-205, 255, 300,-129,-353, -20, 350, 153,-294,},
{ 284,-241,-250, 262, 236,-280,-233, 292, 237,-293,-229, 262, 235,-254,-235, 257,},
{ 255,-304, -99, 351, -59,-339, 215, 250,-319,-125, 363,  -3,-354, 127, 297,-227,},
{ 210,-331,  62, 300,-312, -51, 361,-257,-156, 385,-147,-256, 329,  11,-315, 194,},
{ 184,-361, 242,  97,-354, 320, -32,-278, 357,-121,-217, 350,-167,-171, 336,-171,},
{ 148,-329, 336,-144,-120, 303,-331, 198,  42,-266, 347,-226, -64, 337,-376, 163,},
{ 109,-264, 339,-299, 168,  17,-200, 314,-325, 241, -68,-150, 331,-405, 335,-136,},
{ -85, 221,-324, 374,-365, 307,-212,  90,  48,-180, 284,-341, 336,-283, 195, -75,},
{  34, -95, 158,-207, 247,-290, 335,-364, 373,-370, 343,-297, 239,-172, 105, -37,},
},

{
{  72, 103, 132, 163, 192, 218, 243, 267, 289, 306, 317, 324, 325, 321, 311, 293,},
{-193,-262,-310,-343,-346,-325,-278,-206,-117, -20,  75, 163, 240, 292, 316, 309,},
{ 279, 344, 334, 263, 134, -21,-165,-275,-336,-333,-258,-123,  45, 199, 301, 336,},
{ 330, 353, 228,  21,-197,-336,-335,-196,  15, 213, 331, 324, 176, -40,-241,-331,},
{ 337, 283,  15,-252,-358,-234,  43, 283, 351, 195,-102,-327,-324,-124, 153, 322,},
{ 320, 176,-157,-337,-217, 124, 358, 241,-101,-349,-264,  90, 349, 286, -63,-334,},
{ 326,  84,-274,-288,  54, 356, 177,-237,-346,  -9, 339, 239,-175,-370, -48, 313,},
{ 332, -26,-377, -99, 326, 224,-246,-309, 149, 347, -52,-347, -45, 343, 125,-269,},
{ 315,-150,-345, 157, 331,-181,-313, 211, 300,-241,-261, 245, 233,-271,-193, 240,},
{-254, 196, 214,-291, -94, 368, -91,-336, 270, 197,-379, -11, 380,-168,-267, 224,},
{-259, 306,  76,-383, 227, 194,-388, 157, 238,-365, 103, 251,-314,  19, 274,-182,},
{ 228,-335, 107, 227,-363, 207,  87,-316, 313, -62,-241, 366,-207,-143, 372,-209,},
{ 232,-392, 289, -31,-226, 339,-286, 108, 123,-287, 320,-200, -48, 293,-369, 180,},
{-179, 332,-356, 275,-113, -73, 231,-320, 317,-229,  71, 119,-277, 364,-351, 164,},
{-118, 243,-316, 350,-328, 261,-172,  59,  70,-193, 292,-348, 352,-313, 248,-109,},
{  66,-136, 192,-242, 278,-313, 349,-363, 354,-334, 309,-270, 221,-170, 124, -54,},
}
};

extern TMatrixCoeff g_aiKLT32[KLT_NUM][32][32] =
{
{
{  17,  22,  26,  31,  35,  39,  43,  47,  51,  54,  58,  61,  64,  66,  69,  71,  73,  74,  75,  76,  77,  78,  78,  78,  78,  78,  77,  76,  75,  73,  71,  69,},
{ -46, -56, -65, -72, -78, -82, -85, -87, -87, -85, -81, -76, -68, -59, -48, -37, -26, -15,  -3,   8,  20,  31,  42,  52,  60,  67,  73,  76,  79,  79,  78,  74,},
{ -68, -79, -85, -87, -85, -78, -68, -53, -36, -17,   3,  22,  42,  58,  71,  81,  87,  87,  84,  75,  63,  49,  31,  11, -10, -29, -46, -60, -72, -78, -80, -78,},
{ -75, -84, -85, -78, -63, -41, -15,  14,  41,  63,  79,  87,  85,  74,  55,  30,   2, -26, -52, -72, -85, -90, -87, -73, -52, -26,   3,  31,  56,  72,  82,  82,},
{ -82, -88, -79, -56, -23,  14,  48,  73,  86,  85,  69,  43,   7, -32, -63, -82, -88, -79, -57, -26,  10,  46,  73,  89,  89,  73,  46,  10, -27, -58, -77, -84,},
{ -80, -81, -63, -26,  19,  58,  83,  87,  70,  35, -11, -55, -85, -93, -75, -37,   9,  51,  80,  87,  73,  40,  -4, -49, -81, -92, -79, -44,   0,  42,  72,  85,},
{ -88, -78, -39,  11,  59,  87,  84,  52,   0, -51, -83, -87, -59,  -8,  44,  81,  90,  66,  17, -35, -75, -90, -71, -26,  26,  71,  91,  76,  33, -20, -64, -84,},
{ -97, -75, -17,  49,  91,  88,  45, -18, -70, -88, -66, -15,  47,  86,  81,  37, -26, -74, -86, -57,  -4,  52,  82,  73,  31, -28, -76, -88, -57,  -2,  54,  85,},
{ -86, -54,  10,  68,  86,  51, -17, -72, -82, -45,  19,  73,  81,  40, -25, -76, -86, -41,  32,  84,  88,  39, -38, -88, -83, -31,  44,  94,  83,  25, -44, -90,},
{ -89, -45,  34,  86,  75,   9, -64, -92, -50,  32,  86,  77,  10, -69, -92, -45,  33,  86,  76,   9, -63, -91, -47,  32,  82,  72,   4, -66, -81, -42,  29,  80,},
{ -76, -28,  48,  83,  43, -36, -86, -58,  24,  86,  69, -12, -81, -79,  -3,  79,  88,  12, -71, -91, -33,  62,  95,  41, -46, -91, -50,  37,  86,  62, -17, -83,},
{ -89, -14,  76,  78,   6, -78, -82,   7,  83,  77,  -7, -92, -70,  28,  84,  54, -40, -87, -36,  46,  87,  33, -64, -85, -14,  68,  77,   5, -75, -76,   4,  79,},
{  91,  -6, -83, -58,  31,  85,  32, -59, -78,  -2,  76,  60, -30, -85, -29,  67,  75, -13, -81, -52,  41,  92,  16, -78, -73,  19,  95,  50, -63, -98, -12,  92,},
{  92, -19, -97, -39,  69,  77, -24, -89, -29,  75,  69, -35, -86, -15,  73,  60, -38, -80,  -8,  75,  55, -48, -84,   4,  86,  45, -63, -81,  26,  95,  26, -81,},
{ -82,  34,  90,  -1, -83, -33,  67,  65, -42, -85,   7,  93,  23, -86, -49,  62,  78, -34, -90,  -2,  86,  40, -74, -63,  49,  81, -21, -93,  -7,  90,  38, -73,},
{ -72,  45,  81, -34, -84,  15,  89,   6, -92, -23,  89,  42, -82, -58,  70,  73, -54, -82,  36,  85, -15, -89,   0,  94,  11, -89, -23,  79,  33, -71, -44,  61,},
{  79, -69, -69,  69,  66, -66, -72,  68,  69, -68, -62,  60,  55, -54, -54,  50,  63, -54, -66,  55,  68, -54, -72,  56,  75, -62, -70,  66,  60, -66, -56,  62,},
{ -56,  55,  48, -65, -40,  73,  31, -81, -14,  83,  -4, -82,  23,  82, -47, -73,  60,  62, -69, -56,  82,  40, -98, -12, 104, -19, -96,  39,  84, -52, -76,  67,},
{ -53,  67,  26, -82,   7,  80, -34, -71,  58,  53, -72, -37,  92,   2, -95,  39,  79, -68, -55,  82,  32,-100,  14,  95, -61, -64,  93,  11, -88,  29,  62, -45,},
{  61, -85, -14,  97, -38, -78,  78,  38, -98,  15,  84, -61, -42,  84,  -8, -75,  42,  52, -67, -24,  91, -39, -62,  88,  -5, -86,  70,  33, -91,  25,  68, -50,},
{ -53,  85, -10, -83,  69,  36, -93,  36,  60, -81,   9,  68, -53, -30,  70, -16, -65,  66,  31, -97,  42,  62,-101,  35,  74, -99,  20,  76, -89,  -1,  81, -50,},
{  42, -75,  38,  33, -65,  25,  25, -24,  -8,  28,  -5, -46,  52,  13, -79,  70,  15,-101,  98,  10,-109, 107, -25, -77, 119, -55, -50, 105, -75, -22,  92, -53,},
{ -47,  84, -31, -64,  97, -29, -73, 116, -48, -76, 129, -55, -63, 121, -69, -47, 108, -69, -19,  78, -64,   1,  49, -51,  16,  22, -32,  18,   2, -12,  12,  -5,},
{  31, -68,  53,  13, -77,  81, -15, -66,  94, -52, -24,  92, -98,  22,  76,-111,  57,  37,-101,  86, -11, -56,  78, -58,  11,  38, -67,  65, -21, -51,  84, -41,},
{ -55, 118,-104,  25,  67,-111,  84,  -7, -70,  96, -59,  -7,  56, -66,  36,   5, -31,  45, -46,  20,  19, -47,  58, -54,  18,  41, -89,  89, -19, -73, 108, -52,},
{ -36,  80, -90,  56,   9, -65,  89, -78,  34,  26, -72,  85, -66,  15,  45, -78,  78, -47, -12,  71, -93,  76, -36, -17,  64, -87,  76, -26, -41,  87, -91,  41,},
{  30, -72,  92, -83,  45,   8, -58,  88, -91,  64, -15, -39,  77, -91,  78, -35, -25,  74, -94,  84, -46,  -2,  43, -70,  77, -63,  28,  19, -63,  88, -77,  31,},
{ -22,  53, -73,  78, -61,  29,   9, -46,  75, -90,  83, -51,  13,  27, -66,  89, -89,  68, -34,  -8,  48, -76,  87, -80,  54, -15, -23,  59, -92, 109, -92,  38,},
{  22, -55,  77, -89,  85, -66,  39,  -8, -25,  56, -83,  97, -91,  72, -46,  13,  24, -57,  80, -90,  84, -70,  50, -22, -11,  40, -63,  79, -87,  84, -63,  24,},
{  20, -50,  75, -93, 103,-107, 100, -84,  66, -45,  20,   8, -34,  56, -73,  81, -80,  73, -60,  42, -22,   1,  21, -43,  60, -70,  73, -74,  73, -66,  49, -19,},
{  -6,  15, -22,  29, -34,  36, -35,  33, -29,  24, -19,  11,   0, -13,  26, -38,  51, -64,  77, -88,  95,-103, 113,-119, 118,-112, 101, -87,  72, -54,  33, -11,},
{ -10,  25, -37,  49, -61,  73, -83,  89, -94,  99,-103, 105,-105, 105,-103,  95, -83,  72, -60,  45, -31,  18,  -9,   0,   8, -17,  21, -21,  19, -17,  13,  -5,},
},

{
{  17,  21,  25,  29,  32,  36,  40,  43,  46,  49,  52,  55,  58,  61,  64,  67,  70,  73,  75,  76,  78,  79,  80,  81,  81,  81,  81,  81,  80,  78,  76,  73,},
{ -53, -63, -73, -81, -87, -91, -93, -93, -92, -88, -83, -76, -68, -58, -47, -35, -23, -11,   0,  11,  21,  30,  40,  48,  55,  60,  64,  67,  68,  69,  67,  64,},
{  70,  81,  87,  88,  84,  75,  62,  45,  25,   3, -19, -40, -58, -72, -83, -88, -89, -85, -78, -67, -54, -38, -20,  -1,  17,  34,  49,  61,  70,  75,  77,  75,},
{  80,  89,  88,  77,  56,  31,   1, -29, -57, -78, -90, -91, -81, -63, -36,  -7,  22,  48,  69,  81,  86,  85,  75,  57,  35,   9, -16, -39, -57, -70, -76, -77,},
{  90,  92,  76,  46,   7, -32, -66, -87, -92, -80, -52, -13,  27,  61,  82,  88,  80,  59,  28,  -7, -40, -68, -83, -86, -76, -55, -27,   5,  35,  59,  74,  79,},
{  92,  87,  57,  10, -39, -77, -92, -80, -46,   1,  48,  81,  91,  75,  37,  -9, -51, -80, -87, -73, -42,   0,  40,  72,  88,  83,  61,  27, -12, -47, -70, -81,},
{  91,  73,  26, -30, -74, -89, -67, -19,  36,  76,  87,  64,  16, -38, -77, -87, -67, -24,  28,  71,  89,  77,  41,  -9, -56, -86, -90, -65, -21,  31,  70,  90,},
{  98,  68,   2, -65, -96, -76, -17,  49,  88,  81,  34, -30, -78, -86, -49,  11,  64,  85,  69,  21, -35, -76, -83, -54,  -3,  50,  84,  82,  45,  -9, -58, -86,},
{  89,  46, -29, -83, -82, -26,  48,  90,  71,   6, -61, -89, -60,   8,  71,  90,  55, -14, -74, -89, -58,   8,  69,  89,  59,  -1, -61, -89, -68, -12,  47,  85,},
{  94,  29, -56, -90, -51,  28,  83,  73,   4, -70, -86, -35,  44,  90,  65, -12, -78, -86, -32,  44,  89,  71,   5, -65, -88, -50,  22,  80,  80,  31, -34, -84,},
{  94,  13, -77, -86, -13,  71,  84,  14, -67, -81, -20,  63,  83,  26, -52, -88, -44,  45,  91,  56, -23, -87, -74,   2,  73,  86,  24, -58, -86, -51,  21,  81,},
{  80,  -3, -77, -57,  21,  78,  49, -44, -81, -30,  53,  85,  16, -70, -74,  -6,  76,  79,  -9, -84, -78,  12,  96,  75, -23, -94, -71,  21,  91,  73, -12, -87,},
{  87, -17, -92, -42,  64,  84,  -6, -90, -57,  51,  88,  14, -74, -72,  20,  90,  49, -55, -88, -16,  77,  76, -23, -86, -46,  45,  84,  25, -62, -79,  -6,  76,},
{ -84,  35,  94,   4, -88, -48,  66,  82, -24, -93, -28,  77,  71, -38, -92, -10,  86,  50, -52, -77,   0,  82,  48, -54, -78,  -2,  79,  57, -46, -86, -17,  77,},
{ -75,  51,  76, -34, -80,  10,  85,  19, -75, -46,  54,  68, -28, -82,   0,  85,  34, -77, -62,  51,  80, -10, -90, -28,  81,  65, -57, -95,  22, 103,  33, -87,},
{  85, -73, -77,  69,  73, -54, -81,  45,  85, -31, -87,  14,  88,   3, -90, -20,  88,  35, -77, -47,  59,  62, -47, -71,  31,  79, -10, -82, -14,  78,  42, -69,},
{ -65,  71,  46, -83, -29,  82,  23, -88, -13,  88,   7, -84, -13,  90,  12, -92, -16,  96,  20, -95, -26,  95,  28, -90, -29,  85,  30, -76, -37,  67,  51, -63,},
{  60, -79, -27,  99, -15, -88,  38,  76, -56, -63,  69,  54, -82, -42,  96,  18, -96,   5,  90, -22, -87,  41,  79, -54, -70,  56,  64, -58, -54,  53,  52, -53,},
{  48, -72,   1,  79, -50, -53,  68,  26, -78,   5,  77, -34, -73,  70,  42, -92,   1,  90, -31, -83,  62,  66, -95, -28, 108, -10,-100,  39,  78, -48, -64,  56,},
{ -53,  91, -24, -77,  81,  23, -94,  34,  73, -73, -29,  85, -14, -79,  61,  39, -84,  14,  77, -64, -45,  97, -15, -84,  65,  43, -87,  -1,  85, -30, -71,  55,},
{  53, -97,  42,  62,-101,  30,  71, -94,  11,  84, -75, -23,  79, -32, -49,  67,   2, -71,  48,  42, -86,  27,  64, -81,  -2,  82, -59, -45,  97, -16, -86,  59,},
{ -30,  65, -43, -31,  84, -57, -26,  87, -64, -25,  86, -51, -43,  91, -39, -57,  95, -41, -53,  99, -47, -55, 104, -49, -56,  97, -32, -65,  86,  -3, -75,  47,},
{  14, -37,  36,   4, -44,  50, -13, -49,  76, -29, -52,  93, -51, -49, 111, -75, -27, 110,-105,   8,  91,-101,  36,  50, -97,  57,  35, -88,  64,  17, -77,  44,},
{ -41,  94, -93,  30,  60,-111,  76,  15, -91, 100, -32, -61, 100, -63, -12,  73, -88,  50,  20, -78,  76, -22, -35,  66, -57,   9,  44, -69,  41,  28, -67,  34,},
{ -40,  92,-101,  60,  12, -74,  93, -67,   7,  57, -83,  56,  -2, -51,  75, -56,  12,  38, -74,  70, -22, -35,  69, -74,  37,  34, -92,  95, -27, -72, 108, -49,},
{  34, -81,  96, -70,  20,  33, -72,  88, -70,  20,  37, -75,  82, -54,  -3,  57, -81,  70, -28, -27,  67, -81,  69, -29, -28,  82, -99,  58,  18, -86,  99, -44,},
{ -26,  65, -89,  93, -76,  37,  15, -64,  97,-101,  69,  -9, -49,  88, -98,  73, -26, -24,  62, -82,  75, -46,   5,  40, -73,  80, -53,   3,  48, -76,  66, -26,},
{  21, -55,  78, -92,  97, -86,  55, -14, -25,  64, -94, 100, -77,  39,   2, -39,  70, -89,  88, -65,  25,  17, -51,  73, -72,  51, -18, -25,  66, -86,  74, -29,},
{ -17,  43, -63,  80, -95, 103, -98,  84, -64,  39,  -6, -30,  59, -76,  84, -83,  69, -44,  13,  21, -51,  71, -79,  74, -51,  15,  21, -53,  79, -90,  71, -27,},
{  10, -25,  37, -49,  61, -72,  78, -83,  87, -90,  88, -77,  55, -28,   4,  19, -43,  64, -78,  85, -85,  75, -58,  32,   1, -32,  58, -78,  95, -97,  72, -26,},
{   3,  -9,  14, -19,  27, -37,  49, -60,  68, -77,  88, -99, 106,-108, 109,-109, 103, -96,  87, -74,  58, -40,  23,  -5, -14,  29, -36,  39, -44,  45, -33,  12,},
{   2,  -6,   7,  -8,   8,  -8,   7,  -7,   7,  -8,   7,  -5,   2,   2,  -7,  15, -23,  33, -47,  64, -78,  91,-107, 122,-133, 133,-123, 110, -97,  80, -54,  19,},
},

{
{  16,  20,  25,  29,  33,  36,  40,  43,  46,  49,  53,  56,  60,  62,  65,  67,  69,  71,  73,  75,  77,  78,  80,  81,  81,  82,  82,  81,  81,  79,  77,  75,},
{ -41, -52, -60, -68, -74, -78, -81, -83, -85, -84, -83, -80, -74, -67, -58, -48, -37, -26, -15,  -4,   8,  20,  32,  44,  55,  64,  72,  78,  82,  83,  82,  78,},
{ -59, -73, -83, -87, -87, -83, -74, -61, -46, -29, -11,   9,  29,  47,  63,  75,  84,  89,  89,  84,  74,  60,  43,  22,   0, -20, -39, -55, -68, -75, -77, -74,},
{ -69, -82, -86, -81, -68, -49, -24,   3,  32,  57,  77,  87,  88,  80,  64,  42,  16, -11, -38, -61, -79, -89, -89, -79, -61, -37,  -8,  22,  50,  70,  81,  82,},
{ -73, -84, -80, -63, -34,   0,  36,  65,  83,  88,  77,  54,  21, -16, -48, -72, -84, -83, -68, -40,  -4,  33,  65,  85,  92,  83,  57,  19, -22, -58, -81, -88,},
{ -70, -78, -68, -37,   6,  48,  79,  91,  81,  52,   8, -39, -78, -98, -88, -55,  -8,  37,  72,  88,  82,  54,   9, -37, -73, -89, -80, -49,  -5,  38,  67,  80,},
{ -84, -82, -50,   0,  52,  85,  88,  61,  12, -39, -75, -86, -66, -23,  28,  71,  91,  77,  36, -16, -64, -89, -80, -42,  11,  61,  92,  83,  40, -16, -62, -84,},
{-100, -87, -32,  37,  85,  94,  60,   2, -57, -88, -76, -30,  33,  79,  83,  46, -13, -63, -82, -65, -17,  40,  78,  79,  38, -21, -73, -88, -59,  -4,  52,  85,},
{ -91, -63,   3,  69,  91,  56, -13, -72, -85, -49,  16,  72,  84,  45, -20, -72, -85, -49,  18,  74,  88,  51, -21, -78, -86, -42,  32,  87,  84,  28, -40, -88,},
{ -96, -50,  33,  89,  75,  10, -61, -89, -49,  28,  81,  75,  13, -62, -89, -51,  24,  79,  78,  20, -55, -89, -52,  22,  80,  79,  12, -64, -89, -50,  27,  87,},
{ -87, -30,  51,  88,  43, -39, -86, -57,  27,  88,  65, -15, -81, -77,  -2,  75,  87,  23, -59, -90, -43,  44,  92,  58, -31, -91, -60,  28,  84,  65, -14, -83,},
{ -96, -15,  77,  81,  -1, -80, -71,  11,  83,  72, -15, -91, -66,  27,  88,  56, -38, -87, -45,  39,  89,  44, -54, -89, -28,  59,  82,  12, -69, -75,  -1,  81,},
{ -92,   5,  80,  54, -40, -82, -22,  67,  75,  -5, -77, -59,  34,  89,  28, -70, -81,   8,  86,  62, -36, -91, -29,  67,  77,  -7, -87, -54,  53,  96,  17, -91,},
{  89, -23, -88, -29,  67,  69, -23, -88, -28,  74,  68, -33, -87, -18,  74,  66, -36, -85, -13,  77,  61, -42, -87,  -9,  86,  59, -58, -86,  16,  95,  34, -84,},
{  94, -45, -88,   9,  81,  28, -67, -64,  44,  88, -11, -92, -24,  84,  51, -57, -79,  20,  87,  19, -77, -52,  58,  74, -37, -86,  12,  89,  16, -86, -48,  79,},
{ -87,  64,  80, -51, -82,  28,  86,  -2, -88, -21,  83,  41, -76, -57,  64,  69, -48, -80,  25,  87,  -4, -87, -12,  88,  22, -84, -34,  75,  44, -68, -56,  67,},
{  86, -84, -56,  83,  51, -77, -61,  77,  63, -78, -57,  71,  51, -64, -49,  57,  59, -55, -66,  53,  67, -48, -73,  44,  78, -47, -73,  50,  65, -55, -57,  58,},
{  59, -69, -28,  73,  12, -74,  -2,  77, -11, -77,  24,  76, -35, -77,  53,  69, -65, -58,  69,  53, -77, -43,  89,  29,-100,  -7, 103, -15,-101,  41,  92, -73,},
{ -56,  80,   5, -82,  34,  64, -48, -57,  66,  43, -81, -25,  92,  -7, -87,  37,  75, -61, -61,  78,  40, -92,  -6,  98, -41, -76,  85,  28, -98,  21,  77, -52,},
{  59, -95,  17,  88, -72, -50,  91,  15, -99,  31,  79, -70, -34,  86, -16, -77,  52,  53, -75, -24,  96, -31, -76,  80,  16, -87,  47,  48, -80,  13,  57, -38,},
{ -48,  89, -41, -62,  92,  -2, -81,  53,  37, -76,  20,  63, -64, -27,  89, -33, -74,  87,  19,-100,  54,  53, -94,  19,  82, -85,  -2,  84, -76, -14,  74, -41,},
{  38, -78,  53,  33, -91,  45,  52, -83,  23,  61, -74,  -2,  72, -55, -25,  77, -43, -42,  84, -29, -61,  89, -30, -66, 108, -45, -61, 109, -57, -46,  97, -49,},
{ -26,  53, -38, -15,  56, -39, -24,  68, -43, -30,  79, -42, -47,  94, -51, -45, 106, -72, -29, 105, -92,   8,  78,-105,  60,  23, -90,  95, -28, -61,  91, -42,},
{  29, -71,  68,   0, -81,  98, -24, -77, 112, -53, -42, 103, -86,  -4,  96,-106,  28,  67,-103,  56,  25, -73,  68, -23, -25,  48, -46,  29,   6, -47,  59, -27,},
{  37, -91, 107, -59, -29,  98, -95,  22,  62, -98,  69,   3, -63,  79, -49,  -8,  58, -76,  49,  12, -67,  80, -51,   2,  46, -76,  75, -36, -32,  84, -81,  33,},
{ -27,  70, -92,  70,  -8, -58,  90, -73,  19,  41, -83,  82, -41, -17,  65, -74,  49,  -5, -46,  79, -80,  47,   2, -53,  86, -89,  62,  -6, -63, 107, -98,  40,},
{  31, -82, 113,-106,  67,  -8, -51,  88, -96,  69, -13, -48,  89, -93,  67, -16, -43,  81, -86,  60, -14, -32,  63, -70,  54, -26,  -6,  35, -59,  71, -58,  23,},
{ -15,  40, -62,  71, -65,  45, -10, -31,  64, -82,  78, -51,  11,  36, -80, 100, -91,  62, -21, -25,  66, -91,  94, -71,  32,  10, -45,  75, -97, 102, -80,  31,},
{  14, -41,  63, -76,  82, -75,  52, -19, -17,  53, -85,  99, -89,  65, -33,  -7,  47, -77,  92, -91,  78, -56,  27,   7, -41,  66, -80,  86, -85,  76, -53,  20,},
{  14, -39,  63, -83, 100,-112, 107, -88,  64, -41,  16,  14, -42,  62, -72,  72, -66,  55, -34,   8,  16, -37,  54, -67,  78, -83,  81, -78,  73, -62,  43, -16,},
{  -6,  17, -28,  41, -56,  69, -74,  73, -68,  61, -50,  36, -20,   4,  12, -27,  45, -63,  77, -89,  98,-105, 107,-102,  97, -92,  81, -68,  53, -38,  23,  -8,},
{  -3,   6, -11,  21, -35,  51, -66,  77, -85,  93,-103, 109,-109, 109,-108, 103, -95,  87, -76,  61, -48,  40, -32,  23, -14,   6,   0,  -4,   6,  -8,   8,  -4,},
},

{
{   7,  11,  15,  20,  25,  29,  34,  38,  43,  47,  51,  55,  59,  63,  66,  69,  72,  75,  77,  79,  81,  82,  83,  84,  84,  84,  83,  81,  79,  76,  73,  69,},
{ -15, -25, -36, -47, -57, -66, -73, -79, -84, -87, -88, -87, -84, -78, -71, -62, -51, -39, -25, -12,   3,  18,  32,  45,  57,  67,  75,  81,  84,  85,  83,  79,},
{ -23, -40, -56, -69, -80, -85, -86, -83, -74, -61, -45, -25,  -4,  19,  41,  61,  78,  88,  92,  90,  83,  70,  52,  31,  10, -13, -35, -54, -70, -80, -83, -81,},
{ -31, -53, -72, -84, -88, -82, -66, -44, -16,  14,  42,  66,  81,  88,  84,  70,  48,  19, -12, -41, -65, -83, -91, -87, -71, -46, -16,  17,  48,  71,  85,  88,},
{ -38, -66, -85, -90, -80, -52, -16,  21,  55,  78,  88,  82,  60,  27, -10, -46, -73, -85, -82, -62, -29,  12,  51,  78,  89,  84,  61,  26, -15, -54, -80, -91,},
{ -45, -75, -90, -81, -50,  -3,  45,  78,  91,  81,  48,   2, -45, -80, -92, -76, -40,   7,  50,  80,  87,  69,  30, -18, -59, -84, -84, -58, -15,  31,  67,  85,},
{ -54, -82, -85, -57,  -5,  52,  88,  89,  57,   6, -48, -84, -87, -56,  -4,  48,  83,  86,  54,   3, -49, -84, -84, -51,   0,  53,  85,  83,  48,  -7, -58, -88,},
{ -62, -88, -75, -24,  38,  82,  85,  45, -16, -66, -85, -63,  -6,  53,  85,  75,  28, -33, -80, -85, -47,  17,  75,  90,  58,  -4, -64, -92, -72, -15,  50,  93,},
{ -71, -92, -59,  11,  74,  90,  50, -21, -77, -87, -40,  35,  83,  79,  27, -41, -86, -77, -17,  56,  93,  69,  -4, -70, -84, -47,  19,  75,  79,  32, -33, -82,},
{ -84, -95, -34,  52,  97,  63, -20, -83, -80, -14,  63,  89,  47, -31, -84, -70,  -6,  62,  83,  42, -34, -87, -68,   5,  71,  80,  27, -47, -84, -54,  21,  81,},
{ -74, -77,  -4,  73,  80,  11, -66, -79, -18,  59,  81,  25, -49, -81, -38,  41,  85,  50, -32, -87, -62,  29,  96,  66, -28, -95, -69,  22,  93,  75, -13, -89,},
{ -78, -67,  18,  86,  56, -39, -90, -39,  57,  95,  26, -68, -85, -13,  68,  76,   6, -73, -73,   3,  83,  72, -31, -92, -45,  50,  85,  28, -60, -87,  -8,  82,},
{ -84, -54,  42,  81,  18, -70, -66,  22,  82,  41, -53, -79,  -8,  74,  66, -25, -89, -38,  60,  83,   1, -88, -57,  48,  91,  17, -83, -75,  35, 104,  26, -89,},
{ 104,  43, -79, -79,  31,  96,  20, -85, -70,  48,  93,   4, -85, -53,  47,  79,   6, -78, -52,  50,  76, -15, -80, -23,  68,  59, -42, -80,   3,  84,  34, -72,},
{ -92, -23,  87,  51, -68, -70,  46,  84, -16, -92, -11,  87,  41, -68, -73,  37,  93,   0, -89, -37,  71,  62, -54, -73,  32,  84,  -4, -87, -22,  77,  46, -68,},
{ -85,  -4,  87,  12, -80, -22,  76,  33, -72, -46,  72,  56, -61, -70,  40,  88, -18, -96,  -3,  93,  24, -94, -31,  97,  32, -92, -34,  80,  44, -70, -54,  65,},
{ 101, -16,-100,  26,  92, -39, -87,  41,  85, -40, -78,  31,  75, -27, -77,  21,  81, -21, -81,  18,  85, -26, -90,  42,  86, -50, -72,  48,  66, -50, -60,  55,},
{ -80,  27,  79, -49, -64,  69,  43, -75, -25,  77,   5, -71,   2,  77, -14, -77,  20,  79, -32, -77,  51,  71, -85, -42, 105,  10,-104,  12,  99, -35, -87,  65,},
{  83, -44, -67,  78,  28, -94,  15,  91, -50, -74,  74,  49, -91, -18,  93, -12, -87,  37,  77, -61, -52,  87,   0, -85,  48,  57, -75, -18,  79, -18, -66,  47,},
{  64, -49, -37,  79, -14, -74,  64,  33, -85,  18,  72, -55, -46,  75,  22, -92,  14,  89, -60, -58, 101, -17, -83,  86,  14, -97,  55,  57, -91,   4,  82, -52,},
{ -79,  71,  25, -95,  50,  58, -93,  13,  81, -76, -19,  91, -47, -60,  81,   7, -80,  48,  49, -89,  20,  70, -87,  18,  69, -80,   6,  70, -69, -14,  81, -47,},
{  70, -74,   1,  73, -76,   0,  75, -69, -10,  80, -66, -20,  85, -53, -41,  84, -23, -66,  82,  -6, -79,  90, -29, -59,  99, -46, -47,  95, -53, -41,  93, -49,},
{ -72,  86, -23, -56,  87, -44, -39,  90, -58, -29,  96, -73, -25, 100, -74, -22,  89, -71,  -7,  73, -78,  24,  44, -83,  68,  -2, -67,  84, -33, -44,  77, -38,},
{  69, -95,  51,  28, -86,  85, -30, -48,  94, -70,  -4,  75, -90,  30,  55, -90,  44,  40, -91,  72,  -3, -61,  84, -61,  12,  47, -83,  66,  -2, -63,  78, -34,},
{ -65,  92, -59,   0,  56, -83,  65, -11, -48,  80, -65,   9,  60, -92,  58,  11, -70,  87, -51, -16,  74, -92,  70, -24, -34,  84, -95,  47,  33, -89,  87, -35,},
{ -55,  87, -79,  38,  21, -72,  95, -78,  24,  43, -91,  99, -57, -16,  78, -98,  68,  -4, -60,  93, -82,  42,   2, -39,  66, -73,  52,  -4, -47,  74, -67,  28,},
{  46, -82,  93, -75,  32,  21, -71,  98, -90,  52,  -2, -50,  91,-101,  70, -14, -42,  80, -89,  66, -24, -17,  51, -75,  74, -53,  17,  29, -66,  81, -67,  27,},
{ -36,  67, -82,  82, -61,  27,  13, -49,  74, -80,  71, -46,  11,  30, -71,  94, -86,  55, -11, -33,  62, -76,  83, -80,  57, -23, -19,  64, -97, 105, -82,  32,},
{  40, -71,  89, -96,  91, -74,  47, -14, -23,  57, -84,  97, -93,  76, -52,  18,  22, -55,  77, -85,  77, -63,  46, -24,  -4,  31, -54,  73, -80,  74, -52,  19,},
{ -27,  50, -67,  82, -91,  93, -91,  83, -69,  52, -34,  12,  16, -40,  59, -72,  74, -66,  51, -31,   9,  12, -34,  56, -75,  88, -96,  97, -87,  72, -49,  18,},
{ -14,  30, -42,  55, -64,  72, -79,  82, -81,  78, -73,  64, -50,  33, -17,   0,  17, -33,  48, -60,  68, -77,  85, -91,  96, -97,  93, -84,  70, -54,  35, -13,},
{  -4,  11, -16,  20, -24,  31, -44,  54, -62,  69, -76,  85, -91, 100,-109, 115,-112, 107,-100,  87, -73,  62, -54,  49, -40,  29, -21,  15,  -9,   4,   0,  -1,},
},

{
{  20,  25,  29,  33,  37,  41,  45,  48,  52,  55,  58,  60,  63,  65,  68,  69,  71,  72,  74,  75,  76,  76,  77,  77,  78,  77,  76,  76,  76,  75,  74,  72,},
{ -44, -53, -61, -68, -74, -78, -81, -83, -84, -83, -80, -75, -68, -61, -52, -42, -32, -20,  -9,   3,  15,  27,  39,  50,  60,  68,  75,  80,  84,  85,  84,  80,},
{ -64, -77, -83, -86, -86, -81, -72, -59, -43, -24,  -4,  16,  36,  52,  66,  77,  85,  88,  86,  80,  71,  58,  41,  22,   0, -20, -38, -54, -69, -77, -80, -78,},
{ -70, -83, -85, -79, -64, -46, -22,   5,  33,  57,  78,  89,  90,  82,  65,  40,  11, -19, -46, -69, -83, -89, -88, -76, -56, -31,  -2,  25,  50,  68,  79,  81,},
{ -70, -79, -73, -54, -27,   4,  36,  62,  79,  84,  76,  55,  22, -16, -49, -74, -89, -88, -70, -40,  -2,  36,  68,  89,  97,  86,  59,  19, -24, -59, -81, -89,},
{ -79, -90, -76, -40,   7,  50,  80,  93,  84,  54,   7, -41, -77, -93, -84, -52,  -6,  39,  71,  82,  74,  48,   8, -34, -68, -86, -81, -52,  -8,  35,  67,  81,},
{ -79, -77, -46,   0,  48,  81,  85,  60,  13, -36, -72, -86, -70, -25,  29,  74,  94,  76,  31, -22, -68, -91, -80, -38,  17,  66,  94,  85,  40, -17, -66, -85,},
{-109, -86, -21,  50,  97,  97,  54,  -8, -65, -91, -75, -27,  38,  81,  84,  45, -17, -63, -79, -57, -12,  37,  70,  71,  38, -13, -64, -84, -60, -12,  48,  84,},
{ -87, -57,   8,  65,  83,  55,  -9, -63, -78, -48,  17,  69,  75,  40, -21, -70, -81, -45,  24,  77,  89,  48, -25, -81, -90, -49,  30,  96,  96,  39, -42,-103,},
{-102, -47,  45,  98,  78,   0, -75, -91, -42,  37,  84,  71,  10, -62, -86, -47,  20,  71,  74,  27, -39, -84, -64,   5,  71,  82,  27, -51, -86, -59,  24,  85,},
{ -60, -30,  34,  68,  39, -17, -68, -60,   9,  69,  71,  10, -67, -87, -22,  66,  98,  37, -55, -99, -63,  35, 103,  74, -20, -95, -79,  17,  91,  80,  -9, -91,},
{ -72, -18,  63,  70,  17, -66, -91,  -4,  76,  91,  15, -94, -93,   3,  88,  87, -14, -93, -70,  16,  89,  70, -23, -90, -56,  32,  79,  34, -48, -66,  -3,  55,},
{ 106, -10, -86, -63,  26,  99,  40, -62, -84, -23,  72,  80, -14, -85, -50,  52,  87,  -2, -73, -56,  18,  87,  37, -62, -71,  -1,  78,  60, -55, -87,  -8,  78,},
{ 104, -26,-100, -33,  71,  70, -24, -80, -33,  58,  66, -21, -75, -23,  61,  62, -30, -72, -16,  62,  65, -32, -94, -15,  90,  66, -60, -97,  18, 100,  33, -85,},
{ -95,  38,  98,  -2, -86, -41,  68,  77, -38, -88,   2,  93,  24, -85, -48,  59,  75, -27, -82, -11,  73,  53, -58, -73,  31,  82,  -3, -90, -22,  87,  40, -66,},
{ -60,  41,  64, -36, -70,  13,  76,  12, -85, -30,  90,  50, -86, -68,  71,  80, -54, -85,  27,  84,   9, -89, -32,  99,  35, -91, -40,  77,  42, -69, -51,  66,},
{  85, -79, -63,  85,  57, -78, -72,  74,  78, -71, -74,  63,  59, -47, -55,  35,  67, -40, -68,  35,  67, -25, -73,  25,  83, -38, -84,  58,  66, -61, -53,  56,},
{ -60,  55,  54, -71, -42,  79,  28, -87,  -8,  81,  -3, -80,  17,  87, -51, -69,  72,  46, -69, -47,  68,  54, -85, -41, 109,   0,-102,  32,  84, -48, -70,  60,},
{  35, -53,   1,  51, -23, -32,  15,  35, -16, -55,  43,  57, -91,  -9,  94, -35, -69,  41,  67, -41, -88,  95,  54,-144,  42, 108,-113, -11,  95, -42, -53,  44,},
{  60,-105,  42,  62, -73,   4,  17,  21, -17, -44,  73, -13, -84,  94,   1, -77,  54,  -2, -11,   5, -16,  39, -45, -12,  94, -88, -10, 113,-124,  -4, 130, -81,},
{ -30,  19,  56, -79, -20, 117, -75, -62, 120, -26, -69,  46,  15, -10, -38,  39,  49,-113,  23, 120,-111, -18,  91, -79,  25,  36, -64,  49, -15, -31,  60, -32,},
{ -67, 115, -28,-107, 111,  26,-118,  60,  52, -78,  13,  40, -18, -36,  40,  16, -71,  42,  64, -98,  -3, 100, -91,   2,  78, -77,  11,  45, -50,   9,  25, -17,},
{ -30,  52, -30,  -1,   2,  12,   9, -58,  71, -19, -60, 114, -84, -33, 134,-120,   2, 118,-128,  15,  93, -95,  33,  21, -43,  34, -24,  28, -10, -37,  61, -30,},
{  34, -69,  34,  54, -93,  29,  76,-119,  46,  86,-131,  37,  86,-123,  48,  70,-107,  35,  55, -69,   7,  55, -57,   0,  46, -43,   4,  35, -37,   2,  25, -15,},
{ -52, 114,-110,  31,  66,-115,  77,  23,-100,  87, -13, -52,  68, -36, -14,  32,  -9,  -6,  -9,  28, -23,  -9,  48, -67,  41,  24, -90, 100, -28, -71, 113, -57,},
{  43, -94, 113, -80,   3,  57, -81,  79, -41, -13,  50, -68,  67, -31, -20,  53, -72,  63,   0, -71,  93, -70,  25,  23, -59,  78, -73,  30,  40, -94, 102, -48,},
{  27, -64,  82, -80,  53,  -6, -41,  71, -81,  61, -17, -29,  67, -85,  83, -51, -11,  70,-104,  97, -46, -17,  59, -78,  78, -61,  24,  23, -68,  96, -88,  37,},
{ -18,  42, -61,  69, -61,  36,  -3, -34,  68, -85,  78, -49,  11,  30, -68,  88, -85,  60, -17, -34,  77, -97,  92, -71,  36,   8, -45,  73, -99, 109, -89,  36,},
{  18, -46,  70, -89,  95, -77,  43,  -5, -36,  71, -96, 104, -93,  71, -43,  11,  25, -60,  82, -84,  64, -41,  20,   6, -32,  54, -70,  82, -88,  79, -55,  21,},
{ -15,  36, -53,  67, -81,  88, -80,  63, -45,  26,   3, -35,  58, -74,  86, -88,  80, -69,  48, -21,  -7,  32, -57,  75, -89,  97, -92,  82, -77,  68, -50,  20,},
{  -9,  25, -38,  49, -62,  72, -74,  73, -70,  64, -56,  40, -17,  -7,  25, -37,  53, -70,  86, -97,  97, -96,  97, -97,  93, -86,  74, -60,  45, -31,  18,  -7,},
{  -9,  24, -37,  51, -67,  81, -88,  90, -91,  93, -93,  89, -87,  89, -90,  91, -90,  88, -79,  65, -52,  43, -36,  30, -23,  14,  -8,   5,  -3,   1,   0,   0,},
},

{
{  21,  24,  28,  31,  35,  38,  42,  45,  49,  52,  56,  59,  62,  65,  68,  70,  72,  74,  75,  76,  77,  78,  79,  79,  80,  79,  78,  77,  76,  74,  71,  69,},
{ -47, -56, -65, -73, -79, -84, -87, -89, -88, -86, -82, -76, -68, -59, -49, -37, -26, -14,  -2,   9,  20,  31,  41,  51,  59,  66,  71,  75,  77,  77,  75,  71,},
{  67,  78,  86,  88,  86,  79,  68,  53,  34,  14,  -6, -27, -46, -62, -75, -83, -88, -87, -83, -74, -61, -45, -27,  -7,  11,  29,  46,  59,  70,  77,  79,  77,},
{  77,  87,  88,  81,  64,  40,  11, -20, -49, -71, -86, -91, -86, -71, -49, -21,   9,  36,  60,  77,  87,  87,  78,  61,  40,  15, -10, -33, -54, -69, -78, -78,},
{  84,  87,  77,  52,  16, -23, -57, -80, -89, -83, -62, -30,   9,  46,  73,  87,  87,  72,  43,   8, -29, -62, -83, -90, -83, -63, -35,  -1,  32,  59,  75,  80,},
{  87,  83,  60,  20, -26, -65, -86, -84, -60, -20,  25,  64,  87,  85,  60,  17, -29, -66, -86, -85, -62, -23,  22,  62,  86,  89,  72,  37,  -7, -47, -75, -87,},
{  95,  79,  37, -19, -69, -91, -79, -39,  15,  62,  83,  77,  42, -11, -58, -84, -80, -48,   1,  49,  81,  87,  60,  11, -40, -78, -90, -73, -30,  23,  66,  90,},
{  97,  69,   9, -55, -91, -82, -33,  30,  79,  89,  55,  -5, -63, -89, -70, -19,  41,  79,  81,  46, -11, -64, -88, -68, -16,  40,  80,  86,  52,  -4, -56, -87,},
{  91,  52, -20, -79, -87, -39,  34,  84,  81,  28, -45, -90, -78, -19,  48,  87,  75,  20, -49, -89, -76, -17,  52,  89,  71,  14, -50, -87, -73, -18,  41,  81,},
{  95,  37, -48, -94, -64,  16,  80,  82,  20, -56, -88, -53,  21,  82,  78,  16, -56, -88, -54,  18,  76,  82,  25, -51, -86, -63,   3,  72,  88,  43, -31, -87,},
{  91,  19, -66, -89, -24,  63,  88,  30, -52, -88, -43,  42,  89,  52, -33, -87, -63,  16,  82,  76,   2, -78, -86, -16,  62,  87,  37, -47, -87, -58,  19,  83,},
{  86,   3, -80, -72,  21,  89,  54, -40, -91, -45,  54,  92,  32, -63, -88, -17,  69,  82,  13, -71, -82,  -6,  77,  76,  -1, -76, -74,   6,  77,  73,  -5, -81,},
{  86, -19, -83, -40,  49,  74,   5, -71, -58,  28,  82,  29, -61, -77,   5,  86,  58, -41, -87, -35,  61,  89,  -2, -90, -63,  35,  96,  43, -67, -98, -12,  94,},
{ -85,  29,  96,  15, -86, -60,  51,  93,  -1, -95, -50,  67,  83, -21, -85, -32,  61,  69, -15, -81, -35,  64,  69, -25, -84, -29,  69,  74, -27, -89, -29,  79,},
{ -84,  50,  93, -27, -93,  -5,  89,  39, -80, -66,  56,  89, -29, -94,  -3,  87,  42, -67, -70,  33,  85,   2, -85, -30,  66,  60, -34, -80,   1,  76,  35, -65,},
{  63, -51, -63,  48,  68, -42, -75,  34,  86, -26, -91,  11,  94,   2, -86, -25,  80,  47, -67, -65,  49,  81, -34, -92,  15,  95,  10, -97, -29,  87,  53, -78,},
{ -66,  61,  59, -67, -53,  72,  52, -77, -48,  82,  37, -75, -34,  76,  30, -73, -37,  74,  43, -69, -56,  75,  65, -87, -63,  86,  60, -78, -56,  68,  53, -61,},
{  62, -71, -36,  80,  12, -82,   0,  89, -22, -86,  45,  74, -62, -62,  78,  44, -82, -34,  83,  31, -86, -24,  96,  -1, -95,  24,  87, -39, -76,  48,  71, -63,},
{  62, -87, -18,  98, -29, -85,  64,  61, -91, -19,  91, -20, -77,  53,  55, -73, -35,  85,   9, -84,  12,  84, -49, -67,  81,  34, -90,  -1,  83, -28, -63,  48,},
{ -53,  84,  -2, -90,  66,  51,-102,  14,  90, -74, -34,  92, -28, -72,  67,  40, -85,  -2,  89, -41, -67,  79,  13, -89,  44,  61, -73, -23,  81, -22, -64,  47,},
{  46, -83,  26,  67, -86,  10,  72, -73,  -1,  72, -64, -17,  73, -31, -59,  71,  23, -88,  25,  80, -83, -21, 102, -76, -30, 105, -59, -55,  94,  -8, -77,  49,},
{ -48,  94, -50, -48, 102, -55, -48, 104, -56, -44,  99, -55, -37,  90, -55, -38,  86, -31, -61,  79,  -8, -72,  81, -13, -70,  76,  -2, -67,  63,  12, -72,  43,},
{  17, -34,  19,  16, -47,  43,   7, -67,  78, -28, -51, 100, -69, -28, 106, -92,  -7, 106,-102,  -3,  98,-100,  25,  66,-100,  51,  31, -79,  61,  13, -66,  38,},
{ -38,  88, -79,   9,  71,-104,  69,  15, -92, 109, -59, -30, 100, -98,  22,  67, -92,  39,  38, -77,  58,  -4, -43,  65, -53,   1,  55, -73,  40,  28, -65,  33,},
{ -42,  95, -90,  31,  35, -73,  69, -29, -19,  54, -65,  48,  -2, -47,  70, -56,   9,  47, -77,  59,  -8, -49,  85, -83,  35,  45,-105, 101, -22, -85, 122, -56,},
{  31, -76,  90, -61,   1,  56, -85,  77, -41,  -6,  56, -88,  84, -43, -18,  72, -96,  76, -17, -52,  94, -89,  53,  -1, -49,  82, -81,  39,  26, -79,  82, -34,},
{ -32,  79,-100,  90, -52,  -1,  48, -78,  87, -74,  41,   9, -59,  94,-102,  74, -19, -43,  89,-101,  72, -22, -21,  53, -72,  68, -42,   3,  38, -68,  65, -27,},
{  22, -54,  71, -72,  58, -30,  -4,  35, -59,  74, -76,  64, -45,  14,  29, -71,  96, -91,  64, -21, -27,  60, -79,  89, -82,  54, -13, -38,  89,-118, 100, -39,},
{ -24,  63, -89, 104,-110, 100, -73,  39,  -4, -30,  62, -84,  89, -83,  69, -49,  21,  12, -42,  64, -74,  70, -59,  42, -19, -11,  41, -65,  80, -79,  57, -20,},
{  13, -37,  61, -80,  96,-112, 120,-117, 109,-100,  89, -72,  49, -25,   5,  13, -28,  38, -44,  47, -42,  28, -11,  -4,  21, -38,  53, -64,  72, -70,  52, -19,},
{   1,  -3,   6, -11,  16, -25,  34, -40,  48, -58,  69, -79,  86, -94, 100, -98,  91, -82,  67, -45,  19,   5, -27,  47, -69,  89, -97,  95, -91,  82, -57,  20,},
{   1,  -2,   4,  -8,  12, -11,   7,  -3,   0,   5, -12,  21, -28,  38, -54,  71, -84,  93,-102, 107,-108, 107,-105, 104,-102,  96, -84,  70, -57,  43, -26,   9,},
},


{
{  17,  21,  26,  30,  34,  38,  42,  45,  49,  52,  55,  59,  62,  64,  67,  69,  71,  73,  75,  76,  77,  78,  79,  79,  80,  79,  79,  78,  77,  76,  74,  71,},
{ -42, -52, -61, -69, -76, -80, -84, -86, -87, -86, -83, -78, -71, -63, -53, -43, -32, -20,  -8,   3,  15,  27,  38,  49,  58,  66,  73,  77,  80,  81,  79,  75,},
{ -62, -74, -83, -87, -87, -82, -72, -59, -42, -24,  -4,  16,  36,  54,  68,  79,  86,  89,  86,  79,  68,  54,  36,  16,  -5, -25, -42, -58, -70, -77, -79, -77,},
{ -71, -83, -86, -81, -67, -47, -21,   7,  36,  60,  79,  88,  88,  78,  60,  35,   7, -21, -48, -69, -83, -89, -86, -74, -54, -28,   0,  28,  53,  71,  80,  81,},
{ -77, -86, -80, -60, -29,   7,  42,  69,  85,  87,  73,  47,  12, -26, -58, -79, -87, -81, -61, -29,   7,  44,  72,  88,  90,  76,  49,  12, -26, -58, -79, -85,},
{ -78, -84, -68, -33,  12,  54,  82,  90,  75,  41,  -5, -50, -82, -93, -78, -41,   5,  48,  78,  87,  74,  42,  -2, -46, -78, -90, -78, -46,  -2,  40,  71,  84,},
{ -88, -82, -46,   6,  56,  87,  86,  54,   3, -48, -80, -86, -60, -11,  41,  79,  89,  67,  20, -32, -73, -89, -72, -29,  23,  69,  91,  78,  35, -19, -64, -85,},
{ -99, -80, -22,  45,  89,  90,  49, -12, -67, -89, -68, -17,  45,  84,  81,  37, -25, -71, -84, -58,  -6,  49,  82,  75,  32, -26, -75, -88, -58,  -4,  54,  86,},
{ -90, -59,   8,  70,  89,  52, -17, -73, -83, -44,  22,  76,  82,  38, -28, -77, -83, -39,  31,  82,  86,  39, -35, -85, -82, -33,  39,  90,  82,  26, -42, -89,},
{ -95, -47,  37,  90,  75,   4, -69, -90, -43,  38,  86,  71,   3, -70, -88, -40,  35,  84,  72,   8, -62, -89, -46,  30,  82,  73,   8, -65, -85, -47,  28,  84,},
{ -80, -29,  51,  84,  39, -40, -85, -52,  30,  86,  62, -19, -82, -73,   4,  79,  84,  12, -69, -90, -33,  58,  95,  46, -43, -94, -55,  34,  88,  66, -15, -85,},
{ -87, -13,  73,  76,   2, -78, -76,  12,  83,  72, -15, -93, -65,  31,  88,  54, -42, -89, -41,  45,  91,  39, -60, -90, -22,  64,  81,  10, -72, -76,   2,  79,},
{  93,  -7, -83, -56,  37,  86,  26, -66, -76,   3,  79,  58, -35, -87, -28,  69,  79, -13, -84, -55,  39,  92,  23, -74, -74,  13,  89,  53, -56, -95, -14,  87,},
{  93, -22, -96, -32,  72,  72, -29, -89, -24,  76,  66, -38, -85, -12,  75,  58, -41, -79,  -7,  75,  55, -47, -85,   2,  87,  50, -63, -84,  22,  95,  30, -82,},
{ -87,  38,  91,  -6, -85, -29,  72,  62, -49, -84,  16,  93,  16, -87, -45,  65,  75, -34, -87,  -4,  82,  41, -70, -65,  47,  81, -18, -91, -11,  88,  41, -73,},
{ -76,  52,  78, -43, -80,  25,  85,  -4, -90, -15,  90,  34, -84, -51,  72,  68, -57, -79,  36,  84, -12, -89,  -5,  94,  14, -90, -26,  80,  37, -72, -50,  66,},
{  82, -74, -64,  77,  58, -75, -64,  76,  63, -75, -57,  67,  52, -61, -51,  56,  59, -57, -63,  55,  67, -53, -72,  56,  74, -59, -68,  61,  60, -60, -55,  59,},
{ -58,  61,  41, -73, -25,  76,  13, -82,   4,  81, -20, -78,  34,  78, -56, -67,  68,  55, -73, -49,  82,  39, -96, -16, 104, -11, -99,  31,  89, -48, -80,  67,},
{ -54,  71,  17, -82,  21,  72, -42, -62,  62,  46, -76, -28,  93,  -8, -90,  45,  73, -69, -53,  83,  32,-100,  11,  97, -61, -65,  94,  13, -92,  29,  69, -51,},
{  60, -88,  -1,  94, -55, -64,  86,  23, -98,  28,  78, -68, -36,  86, -13, -76,  47,  52, -70, -24,  92, -34, -69,  86,   5, -88,  61,  42, -89,  19,  68, -47,},
{ -55,  93, -25, -79,  86,  16, -92,  50,  47, -81,  18,  62, -57, -26,  75, -24, -65,  74,  24, -98,  47,  60,-100,  32,  72, -94,  17,  74, -82,  -3,  75, -45,},
{  46, -92,  60,  32, -93,  59,  23, -65,  37,  23, -41,   1,  32, -12, -35,  49,   1, -70,  73,  11, -92,  90, -16, -80, 118, -50, -62, 119, -76, -38, 109, -59,},
{ -29,  51, -17, -45,  66, -14, -65,  92, -29, -75, 114, -39, -77, 124, -57, -62, 121, -71, -35, 102, -76,  -7,  70, -74,  31,  25, -57,  50, -14, -30,  48, -23,},
{  27, -63,  55,   7, -74,  86, -21, -74, 111, -56, -43, 114, -99,  -2, 104,-114,  29,  74,-110,  53,  38, -81,  61, -11, -32,  44, -33,  17,   5, -32,  40, -17,},
{ -49, 108,-106,  41,  49,-105,  92, -21, -59,  93, -66,   4,  49, -68,  47,  -6, -29,  52, -54,  23,  22, -54,  64, -51,   8,  51, -93,  83,  -8, -82, 108, -49,},
{ -36,  82, -97,  66,  -2, -57,  87, -81,  39,  19, -66,  83, -66,  16,  41, -74,  75, -45, -14,  71, -91,  72, -30, -21,  65, -87,  76, -25, -44,  93, -94,  41,},
{  30, -73,  94, -87,  53,  -2, -49,  82, -90,  68, -22, -33,  76, -94,  82, -40, -21,  71, -95,  85, -44,  -6,  48, -73,  77, -59,  23,  22, -63,  85, -73,  29,},
{ -20,  50, -71,  78, -67,  40,  -3, -36,  68, -86,  83, -57,  20,  22, -64,  89, -91,  70, -34, -11,  54, -81,  90, -80,  51, -12, -27,  63, -93, 106, -87,  35,},
{  20, -51,  74, -89,  92, -79,  53, -19, -17,  51, -81,  97, -93,  76, -52,  20,  17, -52,  77, -87,  81, -66,  46, -18, -13,  42, -64,  78, -87,  82, -59,  22,},
{  17, -43,  65, -83,  98,-107, 104, -91,  75, -57,  33,  -3, -25,  48, -66,  75, -77,  73, -60,  41, -20,  -3,  26, -47,  64, -75,  78, -77,  75, -68,  49, -19,},
{  -7,  20, -31,  41, -52,  61, -66,  69, -69,  68, -66,  59, -47,  34, -20,   6,  11, -29,  47, -64,  77, -88,  98,-105, 109,-107,  98, -85,  71, -54,  35, -13,},
{  -4,  11, -18,  25, -34,  44, -55,  63, -70,  76, -84,  90, -94, 100,-105, 106,-103, 100, -93,  83, -71,  62, -55,  47, -38,  28, -19,  14,  -8,   3,   1,  -1,},
}
};

extern TMatrixCoeff g_aiKLT32HP[KLT_NUM][32][32] =
{
{
{  69,  87, 105, 124, 142, 158, 173, 188, 203, 217, 230, 243, 255, 265, 275, 283, 290, 296, 301, 305, 309, 311, 313, 314, 314, 311, 307, 303, 299, 292, 284, 275,},
{-184,-223,-258,-289,-313,-329,-342,-347,-347,-340,-325,-302,-272,-234,-193,-149,-106, -59, -12,  33,  79, 124, 167, 206, 241, 268, 291, 305, 314, 316, 310, 296,},
{-273,-315,-342,-350,-341,-314,-270,-213,-144, -68,  12,  89, 166, 232, 284, 324, 348, 350, 334, 300, 254, 194, 124,  43, -39,-116,-185,-241,-287,-313,-320,-310,},
{-299,-336,-341,-312,-250,-165, -58,  56, 162, 251, 317, 348, 340, 296, 221, 121,   8,-106,-208,-288,-341,-361,-346,-292,-210,-106,  11, 125, 223, 289, 326, 329,},
{-329,-351,-315,-223, -91,  57, 194, 293, 345, 340, 278, 171,  28,-128,-250,-328,-351,-317,-229,-103,  41, 183, 293, 355, 355, 294, 183,  39,-108,-231,-308,-335,},
{-320,-325,-251,-103,  74, 231, 333, 350, 279, 138, -45,-218,-339,-370,-300,-148,  37, 206, 318, 347, 291, 160, -17,-194,-323,-370,-315,-178,   1, 169, 288, 340,},
{-352,-313,-156,  45, 237, 349, 336, 206,   1,-204,-331,-349,-237, -33, 174, 325, 359, 262,  70,-141,-301,-358,-282,-106, 103, 283, 364, 305, 133, -79,-255,-335,},
{-389,-300, -68, 195, 365, 353, 178, -70,-279,-353,-265, -60, 187, 342, 325, 147,-105,-296,-343,-230, -16, 207, 329, 290, 123,-111,-305,-350,-227,  -9, 217, 342,},
{-344,-217,  42, 272, 344, 203, -66,-287,-327,-180,  78, 291, 325, 161, -99,-306,-342,-166, 128, 334, 352, 157,-152,-351,-333,-126, 175, 374, 331,  99,-176,-361,},
{-357,-181, 136, 344, 301,  36,-257,-367,-200, 130, 346, 309,  41,-278,-369,-182, 131, 344, 303,  37,-254,-365,-189, 128, 328, 286,  17,-264,-325,-168, 117, 319,},
{-303,-113, 193, 333, 173,-142,-343,-234,  95, 342, 277, -46,-325,-315, -12, 315, 354,  49,-284,-365,-131, 248, 378, 163,-182,-365,-200, 150, 345, 248, -68,-330,},
{-355, -54, 306, 311,  23,-312,-329,  28, 332, 309, -28,-369,-278, 111, 338, 215,-161,-348,-146, 182, 349, 134,-254,-341, -54, 271, 309,  19,-301,-303,  16, 317,},
{ 364, -24,-331,-233, 124, 340, 130,-236,-313,  -9, 303, 239,-120,-339,-115, 266, 301, -52,-324,-210, 165, 370,  66,-312,-291,  77, 378, 201,-251,-392, -48, 366,},
{ 369, -75,-390,-156, 277, 310, -95,-357,-115, 301, 278,-142,-345, -59, 292, 239,-150,-320, -30, 299, 220,-191,-337,  15, 345, 181,-251,-325, 104, 378, 104,-326,},
{-326, 135, 359,  -3,-332,-134, 270, 259,-166,-340,  27, 371,  91,-343,-195, 248, 313,-136,-358,  -8, 346, 158,-296,-252, 196, 323, -82,-371, -29, 362, 153,-293,},
{-289, 180, 324,-134,-338,  60, 354,  23,-370, -93, 357, 166,-330,-231, 278, 290,-218,-328, 145, 340, -61,-358,  -2, 376,  42,-357, -90, 318, 131,-283,-175, 244,},
{ 316,-275,-274, 274, 264,-264,-287, 273, 277,-273,-246, 241, 219,-216,-215, 200, 250,-218,-264, 220, 272,-216,-290, 225, 301,-247,-280, 265, 242,-265,-223, 248,},
{-224, 219, 193,-262,-161, 292, 123,-323, -56, 333, -18,-330,  93, 329,-188,-290, 241, 249,-275,-223, 328, 161,-391, -46, 414, -75,-384, 156, 337,-207,-303, 268,},
{-214, 266, 104,-326,  28, 321,-134,-286, 231, 211,-289,-148, 368,   6,-379, 156, 318,-273,-221, 328, 130,-399,  54, 379,-246,-255, 372,  46,-353, 114, 248,-181,},
{ 245,-342, -54, 389,-153,-313, 311, 154,-391,  62, 335,-245,-169, 336, -33,-298, 169, 208,-266, -97, 364,-155,-249, 351, -18,-343, 280, 133,-363,  99, 273,-201,},
{-211, 339, -41,-330, 277, 143,-374, 144, 240,-325,  35, 270,-214,-120, 281, -63,-261, 264, 124,-387, 168, 246,-406, 138, 295,-398,  78, 304,-354,  -3, 323,-200,},
{ 170,-301, 152, 134,-261,  98, 102, -97, -31, 113, -21,-183, 207,  50,-314, 281,  59,-405, 390,  39,-435, 427,-100,-309, 476,-219,-199, 422,-300, -87, 368,-212,},
{-188, 337,-122,-258, 388,-117,-294, 462,-191,-305, 516,-222,-251, 483,-276,-187, 431,-274, -77, 313,-256,   3, 197,-204,  63,  87,-128,  70,   6, -48,  49, -22,},
{ 122,-272, 212,  52,-307, 322, -61,-263, 374,-209, -97, 367,-392,  89, 304,-445, 229, 150,-402, 342, -44,-222, 311,-232,  42, 153,-267, 260, -82,-206, 337,-164,},
{-222, 470,-415, 100, 267,-443, 335, -26,-280, 384,-235, -30, 223,-263, 145,  21,-123, 182,-185,  81,  78,-189, 234,-215,  71, 166,-357, 357, -78,-293, 433,-209,},
{-144, 322,-359, 225,  36,-261, 357,-314, 135, 103,-287, 341,-262,  59, 178,-312, 312,-188, -48, 283,-373, 303,-143, -70, 257,-347, 305,-105,-164, 348,-363, 165,},
{ 119,-289, 369,-331, 181,  33,-233, 354,-362, 254, -59,-155, 310,-365, 313,-140,-102, 295,-377, 337,-184,  -9, 172,-280, 310,-253, 111,  78,-253, 351,-308, 125,},
{ -88, 212,-293, 311,-245, 118,  37,-186, 300,-362, 332,-204,  51, 107,-262, 355,-356, 271,-135, -32, 193,-305, 347,-322, 217, -62, -92, 236,-368, 434,-367, 151,},
{  87,-219, 309,-356, 341,-264, 154, -31,-100, 224,-333, 390,-363, 287,-185,  53,  95,-229, 322,-361, 336,-279, 201, -87, -43, 161,-253, 314,-349, 338,-253,  97,},
{  79,-200, 299,-371, 414,-429, 398,-337, 262,-179,  81,  32,-135, 222,-294, 326,-320, 293,-240, 167, -86,   3,  86,-171, 240,-282, 294,-296, 291,-264, 197, -78,},
{ -24,  61, -90, 116,-137, 144,-140, 130,-114,  97, -75,  44,   1, -53, 105,-152, 203,-256, 308,-351, 380,-413, 451,-475, 472,-448, 404,-349, 290,-216, 132, -45,},
{ -40,  99,-148, 196,-243, 291,-331, 357,-375, 394,-413, 422,-418, 419,-410, 380,-332, 287,-239, 181,-122,  74, -35,   1,  33, -66,  84, -83,  76, -69,  51, -21,},
},

{
{  69,  83,  99, 115, 130, 145, 159, 172, 184, 196, 208, 221, 233, 246, 258, 269, 281, 291, 298, 306, 311, 316, 321, 324, 325, 326, 326, 323, 319, 313, 304, 293,},
{-211,-253,-291,-323,-347,-363,-371,-372,-367,-354,-332,-305,-271,-231,-187,-139, -92, -45,   1,  44,  84, 122, 159, 191, 218, 240, 256, 267, 273, 274, 269, 257,},
{ 281, 323, 347, 352, 336, 301, 249, 178,  98,  12, -77,-158,-231,-289,-331,-352,-356,-340,-311,-269,-216,-152, -80,  -6,  68, 137, 196, 244, 279, 300, 307, 299,},
{ 321, 356, 351, 307, 225, 123,   5,-117,-227,-312,-360,-363,-325,-250,-145, -26,  89, 193, 275, 325, 345, 340, 300, 230, 140,  37, -65,-156,-229,-280,-305,-307,},
{ 359, 369, 304, 182,  29,-128,-264,-348,-367,-318,-207, -53, 107, 243, 329, 353, 321, 235, 111, -28,-159,-270,-334,-344,-305,-221,-107,  20, 142, 237, 294, 314,},
{ 366, 347, 227,  40,-158,-308,-367,-321,-184,   4, 192, 326, 364, 300, 150, -37,-205,-318,-346,-290,-166,  -2, 161, 287, 350, 332, 244, 106, -50,-187,-282,-324,},
{ 363, 293, 106,-119,-298,-355,-268, -77, 143, 304, 347, 255,  65,-151,-308,-347,-267, -98, 111, 282, 355, 310, 164, -38,-225,-346,-360,-262, -82, 123, 281, 360,},
{ 390, 271,   7,-260,-382,-303, -69, 197, 353, 323, 135,-120,-313,-346,-198,  44, 257, 342, 277,  86,-140,-303,-331,-214, -11, 201, 337, 329, 179, -36,-232,-346,},
{ 358, 186,-114,-333,-329,-102, 191, 358, 284,  23,-242,-354,-239,  32, 285, 360, 222, -57,-296,-358,-230,  30, 276, 357, 235,  -5,-244,-357,-273, -47, 186, 338,},
{ 377, 118,-223,-362,-205, 111, 332, 292,  18,-278,-345,-141, 176, 361, 259, -47,-312,-343,-128, 178, 356, 286,  19,-262,-351,-200,  88, 321, 322, 125,-137,-334,},
{ 377,  51,-306,-343, -50, 286, 338,  55,-269,-325, -79, 253, 333, 104,-207,-350,-177, 180, 364, 226, -92,-348,-296,   8, 293, 344,  96,-232,-344,-203,  85, 325,},
{ 319, -11,-306,-227,  83, 314, 195,-177,-324,-118, 211, 341,  65,-279,-297, -22, 304, 316, -36,-334,-313,  47, 386, 301, -92,-376,-283,  83, 366, 294, -47,-347,},
{ 349, -67,-368,-167, 255, 334, -22,-361,-228, 204, 354,  55,-296,-288,  78, 359, 195,-218,-354, -65, 310, 303, -91,-343,-185, 179, 334,  99,-249,-316, -23, 304,},
{-338, 142, 375,  15,-354,-191, 265, 326, -98,-372,-111, 306, 285,-152,-369, -39, 343, 201,-206,-309,  -2, 329, 193,-214,-314, -10, 316, 228,-182,-343, -66, 308,},
{-299, 206, 303,-137,-320,  38, 339,  74,-302,-183, 217, 273,-114,-329,   0, 340, 136,-307,-248, 205, 319, -41,-358,-110, 325, 259,-228,-381,  87, 414, 130,-348,},
{ 340,-292,-309, 277, 293,-218,-326, 178, 342,-125,-348,  57, 354,  12,-361, -78, 351, 141,-306,-188, 238, 246,-186,-286, 124, 316, -38,-328, -56, 313, 167,-276,},
{-260, 286, 185,-334,-118, 330,  92,-350, -51, 354,  30,-337, -51, 360,  47,-367, -65, 383,  81,-379,-105, 380, 111,-362,-117, 341, 120,-305,-147, 270, 203,-253,},
{ 238,-316,-106, 397, -62,-354, 150, 304,-222,-254, 275, 217,-327,-167, 383,  73,-382,  20, 359, -89,-346, 162, 316,-216,-281, 225, 256,-231,-217, 212, 207,-210,},
{ 190,-289,   3, 315,-198,-213, 271, 106,-311,  21, 310,-135,-291, 280, 168,-370,   4, 359,-125,-332, 249, 265,-381,-113, 430, -41,-402, 157, 313,-191,-257, 226,},
{-211, 365, -96,-308, 324,  90,-377, 136, 290,-294,-115, 340, -58,-314, 244, 156,-336,  57, 309,-256,-181, 387, -59,-338, 262, 173,-346,  -5, 341,-121,-284, 220,},
{ 211,-386, 167, 248,-406, 120, 285,-377,  46, 338,-301, -91, 317,-128,-198, 269,   6,-284, 192, 167,-343, 110, 255,-323,  -7, 329,-236,-180, 387, -64,-343, 235,},
{-121, 258,-170,-122, 338,-228,-106, 348,-258, -98, 345,-204,-171, 363,-157,-230, 382,-163,-213, 397,-187,-220, 416,-196,-222, 387,-127,-262, 344, -14,-301, 187,},
{  54,-149, 143,  16,-178, 199, -52,-196, 305,-116,-209, 374,-202,-196, 445,-301,-110, 440,-421,  30, 364,-404, 145, 200,-388, 226, 139,-354, 257,  67,-307, 177,},
{-165, 376,-370, 120, 240,-445, 305,  60,-364, 398,-126,-244, 401,-252, -47, 293,-350, 198,  80,-313, 305, -90,-139, 264,-227,  37, 176,-276, 164, 112,-269, 136,},
{-161, 369,-403, 238,  48,-294, 370,-267,  29, 230,-332, 226,  -8,-205, 298,-223,  46, 153,-297, 281, -88,-138, 276,-296, 146, 137,-367, 380,-110,-288, 432,-197,},
{ 137,-324, 382,-280,  79, 131,-290, 352,-279,  80, 150,-301, 329,-215, -11, 228,-325, 281,-111,-110, 268,-325, 275,-115,-112, 329,-394, 231,  72,-344, 395,-175,},
{-103, 260,-354, 371,-303, 148,  60,-256, 387,-405, 274, -35,-198, 352,-391, 293,-102, -94, 250,-328, 298,-184,  20, 160,-293, 319,-212,  11, 194,-306, 264,-103,},
{  85,-219, 313,-370, 389,-344, 219, -58,-100, 256,-377, 398,-309, 156,   7,-155, 282,-357, 352,-258, 101,  68,-204, 290,-288, 204, -70,-102, 262,-344, 294,-116,},
{ -68, 173,-251, 320,-380, 411,-393, 334,-254, 154, -22,-120, 234,-302, 337,-334, 275,-176,  51,  85,-204, 284,-316, 296,-205,  62,  82,-211, 318,-358, 283,-107,},
{  40,-101, 150,-195, 244,-289, 314,-330, 348,-362, 353,-306, 219,-113,  16,  76,-172, 255,-310, 341,-339, 301,-233, 127,   4,-130, 230,-314, 378,-386, 289,-105,},
{  14, -38,  56, -75, 106,-146, 196,-240, 272,-309, 353,-396, 424,-433, 437,-435, 414,-383, 347,-298, 231,-160,  93, -19, -56, 114,-143, 157,-175, 179,-131,  47,},
{   9, -22,  29, -31,  33, -33,  29, -28,  30, -33,  28, -18,   8,   7, -29,  58, -94, 134,-189, 254,-311, 365,-427, 489,-530, 530,-493, 441,-387, 321,-217,  77,},
},

{
{  62,  82,  99, 115, 131, 145, 158, 170, 183, 196, 211, 226, 239, 249, 258, 266, 276, 284, 292, 300, 307, 314, 319, 323, 326, 327, 327, 325, 323, 317, 309, 299,},
{-164,-207,-242,-271,-295,-312,-326,-333,-338,-337,-331,-319,-298,-268,-232,-192,-149,-104, -61, -17,  32,  81, 128, 174, 219, 256, 287, 311, 327, 331, 326, 311,},
{-235,-293,-332,-349,-349,-332,-297,-246,-184,-118, -43,  35, 117, 189, 251, 300, 336, 356, 356, 337, 298, 242, 171,  88,   1, -82,-155,-221,-273,-302,-308,-295,},
{-275,-326,-343,-324,-273,-196, -98,  13, 127, 230, 308, 350, 352, 319, 255, 168,  64, -44,-150,-243,-315,-356,-358,-317,-245,-147, -32,  89, 200, 281, 325, 329,},
{-293,-337,-322,-253,-138,   2, 142, 259, 332, 351, 309, 218,  84, -64,-193,-286,-334,-332,-271,-159, -15, 134, 259, 342, 369, 330, 227,  77, -89,-234,-324,-351,},
{-279,-312,-273,-149,  23, 190, 314, 364, 326, 209,  32,-154,-312,-391,-353,-219, -32, 149, 287, 352, 327, 215,  37,-149,-293,-357,-321,-197, -20, 151, 270, 318,},
{-337,-330,-199,  -2, 207, 342, 352, 242,  48,-155,-299,-344,-265, -92, 112, 286, 364, 309, 144, -65,-256,-357,-320,-170,  42, 244, 366, 332, 160, -65,-250,-335,},
{-401,-346,-127, 146, 340, 375, 242,   7,-228,-351,-303,-119, 133, 316, 331, 183, -53,-250,-328,-258, -69, 159, 314, 314, 152, -84,-290,-353,-238, -18, 209, 340,},
{-365,-251,  14, 277, 365, 225, -53,-289,-341,-198,  64, 288, 337, 181, -80,-289,-341,-197,  71, 296, 353, 205, -86,-311,-342,-169, 129, 349, 334, 112,-162,-351,},
{-385,-199, 133, 356, 301,  39,-244,-355,-197, 114, 325, 299,  51,-246,-356,-203,  95, 317, 313,  80,-222,-355,-209,  89, 321, 314,  50,-258,-354,-201, 108, 346,},
{-347,-122, 203, 350, 171,-157,-345,-228, 110, 353, 261, -60,-325,-308, -10, 299, 348,  93,-236,-358,-174, 175, 367, 232,-126,-365,-239, 111, 336, 259, -54,-334,},
{-383, -58, 307, 323,  -5,-319,-286,  43, 332, 289, -61,-363,-265, 109, 350, 226,-151,-346,-179, 155, 357, 175,-215,-358,-113, 237, 327,  47,-274,-301,  -3, 322,},
{-366,  21, 321, 218,-159,-329, -89, 268, 299, -22,-307,-236, 135, 356, 112,-282,-326,  31, 346, 248,-143,-365,-117, 267, 309, -29,-347,-215, 211, 386,  66,-366,},
{ 355, -92,-350,-117, 266, 278, -91,-352,-113, 297, 273,-133,-350, -73, 295, 264,-146,-341, -54, 306, 244,-168,-347, -35, 344, 235,-231,-345,  64, 379, 134,-337,},
{ 377,-178,-353,  34, 322, 110,-267,-254, 177, 351, -43,-368, -96, 336, 203,-227,-317,  79, 346,  77,-308,-207, 231, 298,-148,-343,  49, 356,  63,-344,-190, 314,},
{-350, 255, 320,-205,-326, 114, 342, -10,-351, -83, 333, 164,-304,-227, 256, 277,-193,-318, 101, 348, -17,-347, -49, 354,  90,-334,-138, 301, 174,-274,-223, 269,},
{ 344,-338,-224, 333, 203,-310,-243, 310, 253,-311,-228, 282, 206,-257,-197, 227, 234,-218,-262, 213, 269,-191,-291, 175, 311,-187,-291, 198, 262,-219,-228, 232,},
{ 235,-276,-111, 294,  46,-298,  -7, 309, -44,-307,  97, 302,-141,-307, 211, 275,-261,-232, 275, 213,-309,-174, 355, 115,-399, -28, 413, -58,-402, 165, 367,-291,},
{-226, 321,  21,-329, 135, 257,-191,-230, 265, 173,-323, -98, 369, -30,-348, 149, 299,-244,-242, 312, 159,-369, -26, 391,-164,-305, 341, 110,-391,  85, 307,-207,},
{ 237,-380,  67, 351,-287,-199, 363,  62,-398, 125, 316,-279,-135, 346, -63,-310, 207, 211,-300, -96, 384,-124,-303, 321,  66,-349, 190, 191,-318,  51, 228,-152,},
{-191, 355,-163,-246, 367, -10,-324, 212, 149,-303,  82, 251,-256,-107, 355,-132,-295, 346,  77,-401, 215, 212,-375,  76, 326,-341,  -9, 337,-305, -54, 297,-164,},
{ 152,-311, 211, 132,-365, 178, 207,-333,  93, 246,-297,  -8, 287,-218,-100, 308,-171,-169, 337,-116,-246, 356,-121,-264, 434,-181,-246, 437,-228,-185, 389,-196,},
{-105, 213,-151, -60, 225,-155, -97, 272,-173,-122, 317,-169,-186, 375,-204,-179, 422,-288,-116, 419,-367,  32, 311,-419, 241,  93,-362, 381,-110,-246, 364,-166,},
{ 116,-286, 271,  -1,-323, 391, -98,-309, 447,-211,-168, 411,-345, -16, 384,-422, 110, 267,-410, 223, 101,-293, 272, -94,-102, 193,-185, 115,  24,-190, 236,-106,},
{ 148,-363, 427,-235,-118, 391,-379,  88, 248,-394, 276,  10,-252, 315,-196, -33, 232,-303, 196,  46,-268, 320,-204,   8, 186,-306, 300,-144,-129, 335,-324, 133,},
{-109, 280,-368, 279, -30,-233, 362,-291,  77, 166,-331, 326,-164, -70, 259,-296, 197, -21,-183, 317,-318, 188,  10,-213, 343,-356, 250, -24,-252, 427,-391, 160,},
{ 126,-329, 452,-424, 267, -31,-204, 353,-385, 276, -53,-192, 355,-373, 268, -63,-173, 325,-345, 239, -56,-127, 251,-280, 217,-103, -24, 139,-234, 283,-232,  91,},
{ -60, 160,-248, 285,-262, 180, -38,-122, 255,-328, 313,-206,  46, 142,-318, 399,-365, 248, -84, -99, 263,-365, 375,-285, 126,  39,-182, 299,-387, 410,-322, 126,},
{  57,-165, 253,-305, 328,-298, 208, -78, -67, 212,-340, 395,-355, 259,-131, -27, 189,-309, 366,-364, 312,-223, 108,  30,-165, 263,-320, 343,-339, 303,-214,  80,},
{  54,-155, 253,-332, 401,-448, 430,-350, 255,-165,  63,  57,-166, 246,-289, 289,-265, 219,-136,  33,  63,-146, 217,-268, 311,-332, 324,-313, 291,-248, 173, -65,},
{ -24,  68,-113, 164,-224, 276,-297, 292,-273, 243,-200, 145, -80,  14,  50,-109, 178,-252, 310,-355, 392,-419, 427,-410, 388,-366, 324,-270, 212,-152,  91, -31,},
{ -11,  26, -46,  84,-139, 204,-266, 310,-340, 373,-412, 435,-437, 437,-432, 411,-379, 349,-302, 243,-192, 159,-128,  93, -56,  24,   2, -15,  23, -32,  32, -14,},
},

{
{  26,  43,  60,  78,  99, 117, 135, 152, 171, 189, 205, 221, 236, 251, 264, 277, 289, 299, 309, 317, 325, 329, 332, 334, 337, 336, 331, 325, 316, 305, 290, 275,},
{ -60, -99,-143,-188,-227,-262,-293,-317,-336,-347,-352,-348,-335,-313,-284,-247,-205,-156,-102, -46,  11,  71, 129, 181, 229, 269, 301, 323, 337, 340, 332, 315,},
{ -93,-159,-222,-277,-319,-342,-346,-331,-298,-246,-179,-101, -15,  75, 164, 244, 310, 352, 370, 362, 334, 281, 209, 126,  38, -51,-140,-217,-280,-320,-333,-324,},
{-124,-213,-289,-336,-352,-328,-266,-175, -64,  54, 169, 263, 325, 350, 336, 280, 190,  76, -47,-162,-259,-331,-364,-348,-285,-185, -64,  69, 194, 286, 339, 352,},
{-154,-264,-339,-360,-318,-209, -66,  83, 218, 314, 354, 328, 239, 107, -42,-185,-291,-338,-327,-250,-116,  47, 205, 312, 357, 334, 245, 103, -61,-215,-322,-364,},
{-181,-301,-358,-325,-200, -11, 179, 314, 366, 322, 190,   8,-181,-320,-369,-305,-159,  27, 201, 321, 349, 274, 119, -71,-237,-338,-337,-234, -60, 126, 268, 339,},
{-216,-330,-341,-227, -20, 208, 351, 357, 230,  23,-194,-335,-348,-223, -18, 192, 331, 342, 215,  11,-198,-337,-338,-203,   0, 214, 342, 331, 191, -26,-233,-351,},
{-248,-353,-301, -97, 151, 328, 338, 181, -65,-265,-342,-250, -25, 210, 341, 299, 111,-133,-319,-341,-188,  69, 301, 360, 232, -14,-256,-368,-290, -59, 201, 373,},
{-285,-369,-236,  45, 297, 361, 199, -83,-309,-348,-160, 140, 331, 315, 108,-166,-344,-309, -68, 222, 373, 276, -14,-279,-334,-189,  77, 300, 316, 129,-132,-330,},
{-334,-380,-136, 210, 388, 251, -81,-331,-318, -57, 252, 356, 186,-124,-335,-280, -24, 247, 330, 169,-135,-349,-271,  19, 282, 321, 107,-186,-335,-217,  82, 324,},
{-297,-308, -18, 293, 320,  43,-264,-315, -73, 238, 323,  98,-196,-325,-152, 163, 339, 200,-130,-347,-250, 117, 384, 264,-114,-379,-278,  89, 371, 301, -53,-356,},
{-314,-270,  74, 344, 224,-156,-362,-158, 230, 380, 103,-273,-341, -53, 273, 305,  23,-292,-294,  11, 333, 286,-124,-369,-181, 199, 340, 111,-242,-346, -31, 327,},
{-334,-215, 167, 325,  70,-278,-264,  89, 330, 166,-211,-317, -33, 296, 265,-102,-355,-150, 240, 331,   5,-350,-229, 192, 365,  68,-330,-300, 139, 416, 105,-356,},
{ 414, 171,-317,-318, 124, 383,  78,-339,-278, 192, 373,  17,-342,-213, 189, 316,  26,-310,-207, 201, 303, -60,-318, -90, 273, 236,-169,-319,  12, 337, 137,-287,},
{-370, -91, 346, 203,-270,-282, 184, 336, -65,-368, -44, 349, 163,-271,-291, 147, 371,   0,-355,-149, 285, 246,-215,-293, 126, 338, -17,-346, -87, 310, 185,-272,},
{-339, -16, 346,  50,-318, -88, 304, 130,-289,-184, 288, 224,-245,-279, 159, 351, -70,-384, -11, 370,  95,-378,-125, 388, 127,-370,-135, 319, 175,-280,-216, 261,},
{ 403, -66,-398, 106, 368,-155,-347, 164, 339,-160,-313, 124, 302,-106,-307,  86, 324, -84,-323,  71, 339,-105,-360, 167, 344,-200,-289, 192, 263,-198,-238, 220,},
{-321, 109, 317,-197,-255, 275, 173,-299,-100, 307,  20,-282,   7, 310, -57,-310,  82, 314,-130,-309, 205, 282,-342,-169, 419,  39,-415,  49, 395,-140,-350, 262,},
{ 332,-178,-266, 312, 111,-375,  61, 363,-200,-294, 297, 195,-363, -71, 371, -48,-349, 146, 307,-242,-210, 348,   1,-340, 194, 227,-298, -73, 315, -74,-263, 187,},
{ 255,-197,-148, 315, -57,-298, 256, 133,-341,  72, 288,-219,-185, 302,  88,-367,  58, 356,-240,-232, 404, -70,-333, 343,  57,-387, 222, 229,-364,  17, 329,-209,},
{-316, 284, 100,-379, 201, 231,-373,  54, 324,-303, -75, 365,-187,-240, 323,  29,-322, 193, 196,-354,  79, 282,-348,  72, 274,-320,  25, 282,-278, -55, 322,-188,},
{ 280,-295,   4, 291,-303,  -1, 301,-277, -39, 321,-265, -82, 338,-213,-164, 336, -92,-265, 330, -23,-316, 360,-116,-235, 397,-185,-190, 378,-211,-165, 374,-195,},
{-289, 344, -93,-225, 346,-175,-156, 361,-233,-115, 385,-292,-101, 400,-295, -87, 354,-282, -27, 293,-311,  95, 176,-333, 273,  -8,-270, 337,-131,-174, 307,-154,},
{ 277,-379, 204, 114,-346, 342,-118,-194, 375,-280, -16, 299,-360, 120, 222,-358, 175, 162,-366, 289, -14,-244, 334,-246,  47, 187,-331, 264,  -9,-253, 313,-136,},
{-262, 368,-236,   1, 224,-331, 261, -45,-193, 320,-259,  35, 239,-366, 234,  44,-280, 347,-205, -66, 296,-366, 281, -94,-137, 335,-379, 189, 133,-357, 348,-141,},
{-219, 346,-314, 153,  84,-289, 380,-313,  96, 172,-366, 395,-227, -64, 312,-392, 272, -17,-239, 372,-330, 169,   8,-158, 264,-292, 209, -18,-187, 298,-269, 111,},
{ 182,-329, 372,-299, 127,  84,-284, 393,-359, 209,  -7,-202, 365,-404, 282, -57,-166, 318,-355, 265, -95, -68, 204,-298, 295,-213,  70, 115,-264, 323,-270, 107,},
{-144, 267,-327, 326,-244, 107,  50,-197, 294,-322, 283,-185,  43, 122,-285, 376,-344, 219, -44,-131, 248,-303, 332,-320, 230, -92, -75, 256,-388, 420,-330, 127,},
{ 158,-285, 354,-385, 365,-295, 187, -56, -90, 229,-335, 388,-373, 306,-206,  72,  86,-219, 307,-341, 309,-253, 186, -94, -16, 124,-217, 290,-321, 294,-208,  76,},
{-108, 199,-267, 329,-362, 371,-363, 333,-276, 209,-138,  48,  64,-161, 234,-288, 296,-265, 204,-124,  36,  50,-137, 225,-300, 353,-384, 386,-349, 287,-198,  74,},
{ -58, 119,-169, 219,-256, 289,-316, 327,-324, 311,-290, 257,-201, 133, -70,   1,  70,-133, 192,-238, 273,-309, 340,-364, 383,-389, 373,-335, 281,-217, 141, -52,},
{ -17,  43, -65,  82, -95, 125,-174, 217,-248, 274,-305, 339,-365, 399,-437, 459,-448, 427,-398, 348,-291, 246,-217, 195,-159, 116, -85,  62, -38,  14,   0,  -2,},
},

{
{  82,  99, 114, 131, 148, 164, 178, 193, 207, 220, 232, 242, 250, 260, 270, 278, 284, 289, 296, 301, 303, 305, 307, 309, 311, 309, 306, 305, 305, 302, 295, 286,},
{-175,-212,-242,-272,-295,-311,-324,-332,-334,-331,-321,-300,-274,-242,-206,-169,-128, -82, -34,  11,  60, 108, 155, 198, 239, 273, 302, 320, 334, 341, 336, 320,},
{-257,-308,-333,-344,-344,-324,-288,-236,-171, -97, -16,  65, 144, 209, 265, 308, 340, 351, 343, 320, 283, 231, 164,  86,   1, -79,-150,-217,-275,-309,-319,-312,},
{-279,-330,-340,-314,-258,-184, -89,  21, 130, 228, 311, 355, 360, 330, 259, 158,  43, -75,-186,-274,-331,-358,-352,-303,-224,-123,  -9, 100, 199, 272, 317, 325,},
{-280,-317,-291,-216,-107,  17, 143, 249, 315, 336, 304, 218,  88, -63,-196,-297,-355,-352,-282,-161,  -9, 144, 271, 355, 386, 344, 236,  76, -95,-237,-325,-356,},
{-317,-359,-305,-161,  27, 199, 321, 371, 334, 217,  27,-164,-309,-370,-336,-209, -24, 155, 282, 329, 297, 192,  34,-135,-270,-342,-323,-207, -31, 140, 269, 324,},
{-318,-310,-186,   1, 194, 324, 341, 241,  54,-145,-288,-346,-279, -99, 117, 298, 376, 302, 123, -88,-271,-364,-319,-150,  67, 265, 377, 339, 160, -68,-262,-342,},
{-435,-342, -83, 202, 390, 387, 217, -30,-261,-365,-301,-106, 151, 322, 336, 179, -70,-252,-316,-230, -47, 148, 282, 284, 154, -52,-255,-336,-240, -46, 192, 335,},
{-347,-228,  34, 259, 332, 219, -35,-253,-313,-192,  66, 274, 302, 161, -83,-280,-325,-179,  96, 307, 355, 190, -98,-322,-361,-195, 120, 384, 385, 158,-169,-410,},
{-407,-187, 179, 390, 312,   2,-301,-364,-170, 147, 336, 285,  38,-247,-343,-187,  81, 285, 296, 107,-157,-337,-256,  21, 282, 328, 107,-204,-346,-235,  98, 342,},
{-240,-120, 137, 270, 156, -66,-271,-240,  34, 274, 283,  39,-269,-347, -90, 263, 392, 147,-220,-394,-254, 141, 412, 296, -81,-380,-315,  68, 365, 320, -35,-365,},
{-286, -71, 254, 280,  70,-264,-364, -18, 305, 363,  62,-377,-373,  10, 354, 348, -54,-370,-280,  64, 358, 281, -90,-358,-222, 127, 315, 134,-190,-264, -11, 220,},
{ 424, -40,-346,-252, 104, 396, 161,-248,-336, -93, 287, 318, -57,-342,-200, 210, 346,  -6,-291,-222,  74, 349, 148,-249,-285,  -5, 311, 239,-218,-350, -34, 313,},
{ 414,-103,-399,-133, 285, 281, -94,-322,-131, 233, 265, -84,-298, -94, 245, 247,-120,-288, -65, 247, 260,-129,-376, -59, 361, 265,-239,-389,  71, 398, 132,-341,},
{-378, 154, 391,  -8,-346,-165, 274, 307,-150,-354,   9, 372,  95,-342,-194, 237, 302,-107,-329, -44, 290, 213,-233,-291, 122, 329, -10,-360, -88, 347, 159,-262,},
{-239, 165, 257,-144,-281,  53, 303,  48,-339,-120, 358, 202,-345,-271, 285, 321,-218,-339, 109, 335,  35,-357,-126, 394, 140,-365,-159, 308, 168,-276,-203, 264,},
{ 339,-317,-253, 338, 227,-310,-288, 298, 312,-284,-298, 252, 234,-187,-221, 140, 267,-158,-273, 142, 270,-101,-294,  99, 331,-152,-336, 231, 265,-243,-213, 226,},
{-240, 221, 216,-285,-166, 316, 112,-346, -32, 326, -13,-318,  69, 346,-206,-276, 289, 182,-276,-186, 273, 215,-341,-164, 434,  -2,-408, 129, 336,-193,-281, 242,},
{ 138,-211,   5, 205, -94,-128,  61, 142, -62,-218, 171, 229,-363, -36, 376,-141,-277, 165, 266,-166,-352, 378, 216,-575, 167, 432,-450, -44, 380,-167,-212, 175,},
{ 239,-421, 168, 250,-292,  16,  69,  85, -67,-178, 292, -50,-334, 377,   3,-307, 216,  -6, -45,  21, -63, 158,-179, -50, 375,-351, -38, 452,-497, -15, 518,-325,},
{-118,  77, 224,-316, -80, 469,-299,-249, 480,-105,-277, 183,  59, -41,-151, 157, 198,-453,  90, 481,-442, -73, 365,-315, 100, 144,-255, 197, -59,-125, 239,-129,},
{-267, 460,-113,-428, 446, 104,-473, 239, 206,-311,  52, 162, -74,-145, 161,  64,-283, 170, 257,-390, -11, 402,-363,   9, 313,-307,  45, 180,-200,  37,  99, -69,},
{-121, 208,-122,  -3,   8,  49,  35,-234, 286, -78,-241, 457,-337,-133, 538,-480,   8, 472,-511,  60, 371,-378, 134,  85,-171, 134, -98, 113, -41,-149, 245,-120,},
{ 135,-275, 135, 217,-372, 118, 305,-475, 185, 343,-523, 147, 343,-491, 193, 278,-427, 139, 221,-275,  29, 220,-228,   1, 185,-171,  16, 142,-148,   8,  99, -59,},
{-208, 458,-439, 125, 266,-460, 309,  91,-399, 350, -53,-209, 271,-142, -55, 129, -36, -24, -37, 112, -93, -35, 191,-269, 166,  96,-361, 400,-111,-282, 451,-227,},
{ 173,-376, 451,-318,  12, 227,-322, 315,-163, -53, 198,-273, 268,-122, -82, 211,-288, 252,   1,-284, 373,-281, 101,  91,-234, 312,-293, 121, 158,-375, 408,-190,},
{ 110,-256, 330,-321, 213, -22,-163, 283,-324, 242, -69,-117, 267,-340, 332,-202, -43, 278,-414, 389,-186, -69, 238,-310, 313,-244,  94,  93,-271, 384,-351, 147,},
{ -72, 169,-242, 275,-243, 146, -13,-135, 272,-342, 314,-197,  45, 119,-270, 352,-341, 239, -67,-135, 307,-386, 368,-285, 144,  31,-181, 292,-395, 436,-355, 145,},
{  72,-185, 278,-355, 380,-310, 173, -19,-143, 285,-384, 414,-373, 286,-174,  46, 101,-238, 328,-335, 257,-166,  79,  26,-127, 214,-279, 326,-351, 315,-219,  82,},
{ -59, 142,-210, 270,-324, 353,-322, 250,-180, 105,  11,-139, 233,-296, 343,-351, 320,-275, 192, -82, -28, 130,-226, 302,-358, 386,-367, 330,-306, 270,-200,  82,},
{ -37, 101,-151, 194,-248, 287,-296, 293,-280, 257,-224, 161, -67, -26,  99,-150, 212,-281, 345,-386, 389,-386, 389,-386, 373,-344, 296,-241, 181,-122,  74, -28,},
{ -37,  95,-149, 203,-267, 324,-353, 360,-366, 373,-370, 358,-349, 355,-361, 364,-361, 352,-318, 260,-206, 171,-145, 120, -90,  57, -32,  22, -12,   3,   1,  -2,},
},

{
{  84,  98, 110, 124, 138, 152, 166, 181, 196, 209, 223, 236, 249, 262, 272, 281, 289, 294, 300, 305, 310, 313, 315, 317, 319, 316, 313, 310, 304, 296, 286, 275,},
{-187,-223,-260,-293,-318,-335,-349,-355,-354,-345,-329,-304,-274,-237,-197,-150,-103, -56,  -8,  37,  82, 125, 165, 203, 237, 264, 284, 298, 307, 307, 299, 284,},
{ 269, 314, 344, 353, 343, 316, 271, 211, 138,  58, -25,-107,-183,-249,-300,-334,-351,-348,-330,-295,-244,-181,-107, -29,  45, 116, 182, 237, 279, 307, 316, 308,},
{ 309, 348, 354, 324, 255, 158,  44, -78,-194,-285,-344,-364,-344,-284,-195, -84,  36, 145, 239, 309, 347, 347, 311, 244, 159,  62, -38,-132,-216,-277,-311,-313,},
{ 336, 350, 307, 208,  65, -91,-226,-319,-355,-330,-247,-119,  35, 183, 291, 349, 348, 286, 173,  33,-115,-248,-332,-362,-333,-253,-139,  -4, 129, 234, 299, 320,},
{ 348, 334, 240,  81,-105,-261,-345,-334,-239, -81,  99, 254, 347, 341, 238,  68,-115,-262,-345,-339,-249, -93,  90, 249, 344, 357, 287, 148, -29,-188,-300,-350,},
{ 380, 317, 147, -78,-275,-365,-317,-154,  59, 246, 334, 309, 167, -43,-234,-338,-319,-192,   4, 198, 325, 346, 240,  44,-160,-312,-361,-292,-119,  90, 262, 360,},
{ 390, 275,  37,-218,-366,-327,-130, 121, 316, 357, 220, -19,-253,-355,-281, -78, 163, 317, 323, 184, -44,-258,-353,-274, -65, 159, 318, 343, 210, -14,-225,-348,},
{ 363, 207, -80,-314,-348,-156, 136, 336, 326, 112,-180,-359,-311, -74, 192, 347, 300,  81,-196,-358,-305, -68, 208, 358, 286,  55,-201,-350,-291, -71, 165, 325,},
{ 381, 149,-191,-376,-256,  65, 320, 328,  82,-225,-352,-211,  82, 327, 313,  64,-224,-350,-216,  72, 304, 327, 101,-203,-344,-252,  14, 286, 353, 172,-125,-348,},
{ 365,  74,-266,-354, -98, 253, 353, 120,-209,-350,-174, 170, 354, 209,-131,-346,-251,  65, 329, 305,   9,-313,-343, -62, 247, 349, 149,-186,-348,-230,  75, 333,},
{ 345,  13,-319,-286,  85, 358, 217,-161,-366,-179, 216, 369, 128,-251,-354, -67, 275, 329,  50,-286,-328, -22, 308, 304,  -6,-305,-294,  23, 309, 292, -18,-324,},
{ 346, -76,-331,-158, 198, 297,  18,-283,-234, 112, 329, 116,-243,-310,  22, 342, 233,-164,-349,-139, 246, 357,  -9,-362,-251, 139, 382, 170,-269,-391, -49, 378,},
{-340, 117, 383,  62,-344,-241, 205, 370,  -4,-380,-199, 269, 331, -85,-340,-126, 246, 278, -58,-323,-138, 255, 277, -99,-334,-116, 275, 297,-110,-357,-117, 314,},
{-337, 201, 374,-109,-373, -19, 354, 157,-319,-264, 226, 356,-117,-377, -14, 348, 170,-270,-281, 131, 341,   8,-341,-122, 265, 239,-137,-321,   6, 306, 141,-261,},
{ 251,-206,-252, 192, 272,-169,-299, 138, 342,-103,-363,  45, 376,   6,-345,-100, 318, 187,-269,-259, 198, 322,-134,-366,  60, 382,  42,-387,-117, 350, 211,-310,},
{-263, 244, 235,-268,-214, 288, 207,-309,-193, 328, 146,-299,-135, 302, 122,-293,-146, 296, 174,-276,-224, 302, 260,-347,-254, 344, 242,-310,-222, 273, 212,-242,},
{ 249,-286,-142, 322,  49,-327,  -1, 358, -87,-345, 179, 295,-248,-247, 310, 177,-326,-136, 333, 123,-343, -95, 386,  -5,-379,  95, 347,-155,-305, 192, 283,-250,},
{ 248,-346, -71, 393,-118,-340, 258, 244,-365, -75, 363, -82,-308, 213, 220,-290,-140, 339,  37,-336,  50, 335,-197,-266, 324, 137,-362,  -5, 331,-112,-254, 192,},
{-211, 335,  -9,-359, 262, 205,-406,  55, 359,-297,-135, 368,-112,-290, 267, 160,-342,  -9, 355,-163,-268, 317,  54,-357, 177, 245,-290, -91, 323, -87,-254, 188,},
{ 185,-334, 104, 268,-343,  40, 288,-292,  -5, 288,-254, -70, 293,-125,-235, 283,  93,-353,  99, 321,-332, -83, 409,-304,-118, 419,-236,-221, 375, -32,-308, 194,},
{-194, 375,-200,-192, 407,-221,-193, 417,-226,-178, 395,-219,-146, 359,-219,-153, 345,-126,-242, 316, -31,-288, 326, -50,-278, 306,  -9,-270, 251,  48,-289, 170,},
{  70,-135,  76,  64,-186, 172,  30,-267, 312,-111,-203, 401,-277,-112, 424,-370, -29, 423,-408, -13, 392,-399,  99, 263,-399, 204, 122,-317, 243,  52,-265, 152,},
{-153, 351,-314,  34, 283,-417, 277,  59,-366, 437,-236,-119, 399,-392,  87, 269,-368, 155, 154,-309, 230, -17,-172, 262,-211,   2, 219,-291, 160, 111,-260, 131,},
{-169, 381,-360, 124, 140,-290, 276,-118, -78, 215,-262, 191,  -9,-188, 281,-224,  38, 189,-309, 238, -31,-195, 341,-334, 140, 179,-422, 405, -89,-340, 490,-222,},
{ 124,-304, 362,-243,   3, 222,-338, 307,-165, -24, 223,-353, 334,-172, -71, 289,-384, 303, -68,-207, 376,-357, 210,  -5,-197, 330,-322, 154, 105,-315, 326,-137,},
{-126, 317,-400, 359,-210,  -2, 193,-312, 347,-296, 163,  36,-234, 376,-409, 297, -75,-172, 354,-402, 288, -88, -85, 210,-287, 274,-168,  13, 151,-271, 260,-108,},
{  87,-215, 283,-287, 233,-120, -17, 139,-236, 298,-304, 258,-179,  56, 115,-286, 382,-363, 255, -84,-107, 240,-315, 356,-328, 218, -51,-153, 356,-472, 399,-156,},
{ -95, 251,-357, 418,-441, 402,-293, 155, -15,-122, 249,-334, 357,-332, 277,-195,  82,  48,-168, 258,-295, 281,-236, 170, -75, -45, 162,-261, 321,-316, 227, -81,},
{  51,-148, 244,-319, 384,-450, 481,-469, 436,-401, 356,-288, 196,-102,  18,  54,-113, 151,-177, 187,-166, 113, -44, -18,  84,-153, 211,-257, 289,-282, 208, -78,},
{   3, -10,  25, -43,  65,-100, 134,-161, 190,-230, 277,-318, 346,-377, 399,-391, 364,-327, 268,-179,  78,  21,-108, 188,-276, 356,-388, 381,-365, 328,-228,  82,},
{   2,  -7,  16, -31,  46, -45,  29, -13,   0,  18, -50,  84,-111, 152,-217, 283,-334, 373,-406, 430,-431, 428,-421, 414,-409, 383,-334, 281,-229, 173,-105,  35,},
},

{
{  68,  85, 102, 119, 136, 152, 166, 181, 195, 208, 222, 235, 247, 258, 268, 277, 285, 292, 299, 304, 309, 312, 316, 318, 319, 318, 316, 313, 309, 303, 294, 284,},
{-168,-208,-245,-277,-303,-321,-336,-343,-346,-342,-331,-312,-285,-251,-213,-170,-126, -80, -32,  14,  61, 108, 153, 195, 234, 265, 291, 308, 319, 322, 316, 301,},
{-247,-298,-332,-347,-346,-326,-288,-235,-169, -95, -15,  65, 145, 215, 273, 317, 346, 354, 345, 316, 273, 214, 143,  62, -20, -99,-170,-230,-279,-308,-316,-306,},
{-282,-331,-345,-324,-269,-188, -85,  30, 142, 241, 315, 353, 351, 312, 239, 140,  28, -86,-191,-275,-331,-357,-346,-295,-215,-113,   0, 113, 212, 282, 321, 325,},
{-308,-343,-319,-239,-116,  28, 168, 277, 340, 347, 293, 190,  48,-104,-231,-316,-349,-324,-242,-118,  29, 174, 288, 353, 360, 304, 195,  49,-104,-233,-314,-342,},
{-313,-335,-274,-132,  48, 215, 328, 358, 299, 164, -20,-199,-329,-373,-312,-164,  22, 194, 311, 347, 296, 168,  -7,-183,-311,-361,-314,-184,  -7, 162, 282, 335,},
{-351,-327,-182,  23, 226, 348, 343, 218,  14,-190,-321,-344,-239, -43, 165, 316, 357, 267,  81,-127,-292,-357,-289,-116,  93, 275, 365, 313, 141, -76,-255,-339,},
{-394,-319, -90, 179, 357, 359, 195, -50,-268,-354,-273, -67, 181, 337, 322, 148, -98,-284,-336,-232, -24, 198, 328, 299, 129,-104,-300,-352,-233, -14, 214, 346,},
{-359,-237,  33, 280, 356, 210, -67,-293,-333,-178,  89, 303, 326, 152,-112,-310,-333,-158, 123, 327, 346, 158,-138,-338,-329,-132, 158, 361, 329, 105,-167,-357,},
{-379,-189, 147, 361, 298,  15,-274,-359,-171, 152, 346, 284,  13,-280,-353,-161, 140, 335, 287,  31,-250,-355,-186, 121, 328, 294,  30,-259,-341,-187, 112, 336,},
{-322,-115, 202, 335, 158,-159,-340,-209, 121, 343, 250, -75,-329,-293,  15, 316, 336,  46,-277,-358,-133, 231, 381, 184,-174,-374,-220, 138, 354, 263, -61,-342,},
{-349, -54, 294, 303,  10,-311,-303,  49, 333, 288, -59,-372,-262, 124, 352, 216,-166,-358,-163, 180, 363, 155,-239,-359, -90, 258, 326,  39,-286,-306,   6, 314,},
{ 373, -27,-334,-224, 146, 344, 103,-263,-304,  14, 315, 231,-140,-347,-111, 276, 316, -51,-337,-221, 157, 368,  91,-294,-297,  52, 356, 211,-225,-379, -54, 349,},
{ 372, -87,-382,-128, 289, 287,-115,-354, -97, 305, 264,-152,-342, -49, 300, 233,-164,-317, -29, 298, 222,-188,-338,   6, 348, 199,-251,-336,  90, 378, 118,-329,},
{-348, 153, 363, -26,-339,-114, 288, 248,-195,-336,  64, 371,  63,-349,-180, 260, 302,-137,-347, -14, 330, 164,-279,-259, 187, 326, -72,-364, -43, 351, 165,-294,},
{-304, 208, 310,-173,-320, 100, 341, -16,-361, -59, 359, 136,-335,-205, 287, 270,-228,-315, 144, 337, -48,-357, -19, 375,  57,-358,-105, 320, 148,-287,-200, 264,},
{ 326,-296,-258, 308, 234,-300,-255, 304, 254,-301,-229, 270, 207,-246,-202, 223, 235,-230,-253, 221, 269,-214,-288, 223, 294,-236,-272, 242, 240,-241,-219, 235,},
{-233, 246, 163,-290, -99, 306,  51,-326,  16, 324, -80,-312, 137, 313,-224,-268, 273, 218,-293,-197, 328, 155,-383, -65, 414, -45,-394, 126, 356,-192,-318, 269,},
{-215, 284,  66,-328,  82, 288,-166,-248, 247, 182,-304,-111, 370, -33,-360, 180, 291,-277,-210, 331, 128,-400,  45, 390,-244,-259, 374,  51,-367, 115, 276,-202,},
{ 239,-351,  -4, 374,-219,-256, 343,  92,-390, 111, 313,-273,-142, 343, -51,-304, 189, 208,-279, -95, 366,-137,-275, 343,  18,-354, 246, 166,-355,  74, 271,-189,},
{-220, 374,-101,-315, 345,  64,-367, 200, 189,-324,  71, 249,-227,-106, 301, -97,-262, 295,  98,-390, 186, 240,-400, 129, 286,-377,  68, 294,-330, -11, 298,-180,},
{ 186,-367, 239, 127,-372, 236,  93,-260, 149,  90,-165,   3, 130, -48,-139, 195,   5,-279, 290,  45,-369, 360, -64,-318, 472,-200,-248, 478,-304,-152, 437,-237,},
{-115, 204, -70,-180, 266, -56,-261, 368,-114,-300, 455,-155,-308, 495,-230,-249, 483,-284,-139, 409,-305, -29, 281,-295, 123,  99,-226, 199, -55,-120, 190, -92,},
{ 107,-251, 219,  27,-297, 344, -82,-296, 443,-224,-171, 455,-397,  -6, 416,-458, 116, 295,-442, 214, 152,-325, 246, -42,-129, 177,-133,  70,  18,-129, 160, -70,},
{-197, 431,-423, 162, 194,-419, 369, -84,-236, 374,-262,  14, 196,-271, 189, -24,-116, 209,-217,  94,  89,-216, 255,-203,  34, 204,-373, 333, -31,-326, 431,-198,},
{-143, 327,-386, 265,  -7,-229, 347,-322, 156,  78,-266, 334,-263,  65, 165,-296, 301,-179, -57, 283,-363, 287,-121, -86, 261,-347, 304,-102,-178, 373,-375, 164,},
{ 119,-290, 378,-349, 211,  -6,-195, 328,-361, 273, -86,-133, 305,-375, 329,-159, -83, 285,-379, 338,-177, -24, 191,-292, 308,-236,  91,  89,-253, 341,-294, 118,},
{ -80, 198,-282, 310,-266, 159, -11,-143, 271,-344, 332,-228,  80,  89,-255, 358,-365, 281,-136, -46, 216,-324, 358,-320, 206, -47,-109, 251,-373, 426,-349, 139,},
{  79,-204, 297,-355, 368,-317, 210, -76, -67, 206,-325, 389,-372, 304,-208,  81,  69,-208, 307,-349, 325,-265, 183, -71, -53, 167,-254, 314,-346, 328,-238,  89,},
{  67,-171, 261,-333, 391,-428, 417,-365, 300,-227, 131, -14,-100, 193,-263, 300,-307, 291,-241, 165, -78, -12, 103,-187, 258,-301, 311,-310, 301,-270, 197, -76,},
{ -30,  79,-122, 164,-208, 244,-265, 276,-277, 273,-263, 236,-189, 134, -80,  24,  43,-115, 188,-256, 309,-354, 394,-421, 434,-429, 394,-341, 283,-217, 139, -50,},
{ -17,  44, -71, 100,-135, 177,-221, 253,-278, 305,-335, 360,-378, 400,-419, 422,-412, 399,-374, 331,-284, 247,-219, 189,-151, 110, -76,  54, -34,  11,   3,  -3,},
}
};

extern TMatrixCoeff g_aiKLT64[KLT_NUM][64][64] =
{
{
{  18,  20,  23,  25,  27,  29,  31,  33,  35,  37,  39,  41,  42,  44,  47,  48,  50,  52,  53,  55,  56,  57,  59,  61,  63,  64,  65,  66,  68,  68,  70,  71,  72,  73,  73,  74,  75,  75,  76,  76,  77,  77,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  77,  76,  76,  75,  74,  74,  73,  72,  71,  70,  69,},
{ -47, -52, -57, -62, -66, -70, -74, -77, -80, -83, -85, -87, -88, -88, -88, -88, -87, -86, -86, -84, -81, -78, -76, -72, -68, -64, -60, -55, -51, -45, -40, -34, -28, -21, -15,  -7,  -1,   5,  11,  16,  21,  26,  32,  37,  42,  47,  51,  55,  58,  61,  64,  67,  69,  71,  73,  74,  75,  75,  75,  75,  75,  73,  71,  68,},
{  62,  68,  74,  78,  81,  84,  85,  85,  84,  82,  79,  76,  71,  63,  54,  45,  37,  28,  19,   9,  -1, -12, -23, -33, -43, -51, -61, -69, -76, -82, -86, -89, -91, -91, -91, -88, -84, -79, -74, -68, -62, -55, -48, -39, -30, -20, -10,   1,  10,  18,  27,  35,  43,  50,  58,  64,  69,  74,  77,  79,  79,  80,  79,  75,},
{  70,  77,  83,  86,  87,  85,  82,  75,  68,  57,  45,  32,  18,   3, -12, -26, -39, -51, -61, -70, -78, -83, -87, -88, -87, -82, -75, -67, -57, -46, -33, -19,  -4,  11,  25,  40,  53,  65,  74,  82,  88,  90,  91,  89,  84,  78,  71,  61,  50,  38,  25,  12,   0, -14, -28, -40, -52, -62, -70, -75, -79, -82, -81, -78,},
{  76,  83,  86,  86,  82,  74,  62,  47,  31,  13,  -5, -23, -40, -55, -69, -80, -88, -91, -90, -85, -77, -65, -50, -32, -13,   8,  29,  47,  63,  75,  84,  90,  92,  89,  82,  69,  55,  38,  19,   0, -19, -36, -52, -65, -76, -84, -88, -87, -84, -77, -68, -55, -41, -26,  -9,   9,  25,  40,  54,  65,  74,  79,  79,  75,},
{  75,  80,  80,  75,  67,  54,  36,  16,  -6, -28, -49, -65, -77, -85, -87, -84, -75, -61, -42, -20,   4,  28,  50,  70,  85,  93,  92,  85,  74,  57,  37,  14, -10, -32, -53, -71, -84, -91, -92, -86, -75, -59, -39, -17,   8,  32,  53,  71,  85,  92,  94,  89,  79,  64,  45,  25,   4, -17, -36, -53, -67, -76, -80, -77,},
{  80,  84,  82,  73,  58,  37,  10, -18, -45, -70, -87, -95, -95, -86, -67, -42, -14,  16,  42,  65,  82,  91,  93,  85,  68,  44,  15, -16, -44, -66, -81, -89, -88, -77, -59, -36, -11,  15,  40,  62,  76,  83,  84,  77,  63,  42,  18,  -9, -33, -55, -72, -83, -87, -84, -74, -57, -36, -13,  11,  35,  57,  73,  81,  81,},
{  78,  80,  73,  59,  37,   9, -21, -50, -70, -82, -86, -79, -62, -38,  -6,  26,  54,  76,  87,  87,  78,  57,  29,  -6, -39, -64, -81, -87, -83, -66, -42, -13,  18,  48,  72,  87,  90,  81,  63,  35,   3, -29, -58, -79, -89, -88, -78, -58, -32,  -2,  30,  56,  77,  92,  96,  87,  68,  42,  11, -22, -51, -74, -88, -91,},
{  80,  81,  70,  50,  20, -16, -50, -75, -88, -87, -73, -48, -15,  20,  54,  79,  90,  88,  72,  42,   6, -32, -64, -87, -96, -87, -61, -25,  14,  46,  70,  84,  86,  73,  46,  10, -25, -57, -80, -87, -81, -64, -37,  -4,  33,  64,  84,  90,  81,  63,  35,   1, -34, -66, -85, -91, -85, -66, -37,  -3,  33,  64,  80,  85,},
{  87,  86,  69,  39,  -1, -40, -73, -92, -93, -74, -39,   3,  42,  72,  87,  85,  66,  32,  -7, -42, -72, -88, -83, -61, -26,  16,  55,  80,  88,  77,  51,  16, -24, -58, -82, -88, -75, -49, -14,  25,  57,  79,  86,  78,  54,  17, -21, -54, -79, -87, -77, -53, -19,  19,  54,  80,  91,  88,  67,  30, -14, -57, -84, -93,},
{ -83, -79, -56, -19,  24,  62,  87,  91,  74,  40,  -4, -49, -82, -94, -81, -46,  -2,  41,  74,  87,  80,  53,  11, -34, -71, -88, -79, -51, -11,  29,  60,  79,  80,  61,  28, -16, -56, -82, -89, -73, -40,   3,  47,  80,  92,  83,  53,   9, -36, -70, -87, -86, -64, -24,  21,  61,  85,  92,  79,  46,   2, -45, -78, -92,},
{ -85, -75, -45,  -5,  41,  76,  92,  82,  46,  -6, -56, -88, -94, -72, -27,  28,  71,  93,  87,  54,   4, -46, -81, -91, -78, -38,  17,  62,  87,  86,  63,  23, -26, -71, -95, -86, -53,  -5,  42,  75,  87,  75,  40,  -4, -49, -82, -92, -72, -32,  13,  55,  81,  84,  63,  26, -20, -59, -83, -82, -54, -12,  29,  62,  78,},
{ -87, -70, -32,  15,  59,  84,  79,  46,  -1, -47, -71, -71, -49, -12,  29,  63,  78,  65,  26, -21, -60, -80, -70, -39,   8,  59,  90,  83,  45,  -8, -54, -84, -87, -58,  -6,  46,  84,  95,  71,  21, -35, -79, -96, -85, -40,  20,  71,  99,  90,  53,   1, -52, -87, -94, -70, -23,  32,  79,  98,  85,  40, -23, -72, -94,},
{ -86, -64, -20,  29,  68,  81,  63,  25, -24, -66, -79, -61, -22,  25,  66,  86,  72,  27, -30, -76, -93, -71, -16,  45,  83,  85,  54,   0, -54, -86, -85, -49,  10,  68,  97,  85,  35, -30, -80, -96, -73, -25,  33,  78,  94,  71,  15, -48, -90, -92, -57,  -4,  48,  85,  88,  54,   4, -53, -89, -87, -52,   7,  60,  85,},
{-101, -74, -15,  50,  95,  98,  56, -10, -69, -91, -74, -28,  29,  75,  88,  59,   1, -55, -83, -72, -32,  26,  73,  84,  58,   6, -58, -93, -82, -36,  27,  79,  94,  69,   5, -64, -95, -78, -30,  30,  76,  88,  59,   6, -53, -89, -80, -31,  31,  75,  83,  54,   3, -51, -80, -71, -29,  26,  70,  81,  56,   6, -49, -80,},
{-106, -67,   0,  66, 103,  87,  24, -49, -91, -83, -35,  26,  73,  85,  52,  -9, -67, -89, -65,  -6,  59,  91,  67,  14, -43, -83, -80, -28,  40,  82,  82,  40, -24, -73, -83, -50,   8,  64,  83,  57,   1, -53, -80, -66, -12,  49,  86,  70,  19, -36, -77, -78, -42,  16,  69,  87,  59,   3, -60,-100, -85, -17,  54,  97,},
{ -84, -47,  13,  65,  83,  50, -10, -63, -77, -51,   6,  63,  84,  54,  -6, -66, -91, -59,   9,  69,  91,  60,  -9, -68, -88, -52,  17,  73,  84,  45, -19, -78, -89, -46,  25,  84,  89,  36, -37, -84, -80, -31,  39,  87,  79,  27, -40, -88, -85, -32,  43,  92,  92,  38, -40, -92, -89, -36,  37,  91,  89,  27, -41, -86,},
{ -86, -43,  24,  74,  81,  35, -36, -87, -80, -15,  58,  95,  68,  -1, -74,-100, -64,  19,  90, 101,  47, -42,-101, -84, -15,  54,  98,  73,  -6, -73, -95, -52,  31,  90,  81,  17, -52, -87, -60,   2,  58,  81,  51,  -9, -68, -85, -38,  32,  78,  70,  17, -44, -79, -63,   0,  62,  81,  46, -20, -74, -76, -28,  32,  73,},
{ -96, -41,  44,  94,  81,   8, -72,-104, -58,  38, 103,  92,  17, -76,-112, -63,  30,  98,  95,  19, -75,-109, -50,  44,  94,  80,   0, -82, -90, -39,  46,  96,  69,  -7, -75, -84, -29,  44,  78,  57,  -2, -61, -75, -37,  29,  77,  61,   4, -55, -71, -34,  22,  59,  59,  17, -35, -60, -49,   1,  52,  63,  30, -22, -60,},
{ -62, -23,  34,  70,  49, -13, -67, -71,  -7,  67,  79,  30, -44, -89, -55,  27,  84,  69,  -1, -69, -86, -28,  59,  91,  41, -40, -96, -67,  31,  93,  79,   2, -84, -99, -24,  68,  94,  45, -36, -84, -68,   3,  69,  80,  31, -41, -86, -64,   7,  76,  89,  33, -43, -93, -77,   1,  85,  96,  29, -53, -97, -64,  25,  90,},
{ -90, -30,  60,  99,  55, -37,-106, -74,  25, 104,  88, -14, -93, -87,  -5,  90,  95,   4, -81, -94, -21,  78,  92,  23, -64, -92, -23,  72,  84,  28, -55, -87, -29,  40,  71,  37, -34, -64, -42,   8,  59,  63,   9, -46, -70, -33,  33,  69,  51,  -9, -67, -70, -14,  63,  89,  35, -55, -91, -48,  35,  97,  63, -21, -83,},
{ -49,   5,  37,  38,  -2, -47, -24,  10,  34,  28, -16, -24, -10,   4,  16,  -1, -12,   6,   2,  -7,  -1,  -8,  17,  18, -12, -22, -25,   6,  53,  27, -31, -54, -41,  41,  88,  22, -53, -87, -43,  70, 104,  40, -60,-128, -43,  95, 115,  24,-102,-130,  -7, 111, 125,  16,-120,-135,   0, 122, 114,   2,-120,-109,  12, 112,},
{  18,  15, -16, -29, -19,   3,  45,  37, -12, -53, -53,  19,  67,  43, -19, -73, -48,  38,  76,  43, -36, -98, -34,  64,  86,  33, -81,-114,   5,  94,  95,   9,-114,-105,  34, 123,  83, -50,-132, -62,  53, 118,  77, -58,-120, -63,  42, 114,  63, -45, -85, -60,  19,  80,  52, -17, -45, -46,  -6,  38,  29,  17,  -3, -36,},
{ 149,   2,-124,-126,  -6, 132, 115, -28,-130, -96,  47, 126,  70, -47,-118, -56,  71,  98,  29, -71,-107,  -1,  98,  75, -22, -95, -59,  49,  81,  40, -41, -89, -21,  64,  59,   0, -60, -48,  23,  47,  31, -11, -46, -22,  22,  35,  13, -29, -34,   8,  25,  17,  -5, -20,  -6,  20,   7,  -6, -17, -15,  20,  27,  -6, -19,},
{  78, -14, -70, -50,  24,  75,  33, -46, -75, -10,  69,  56, -18, -72, -41,  48,  72,   2, -64, -53,  25,  78,  23, -61, -63,  11,  83,  42, -59, -74,  -7,  65,  67, -24, -79, -23,  53,  66,   1, -82, -60,  40,  86,  37, -67, -92,   7,  99,  70, -51,-115, -37,  87, 107,  -2,-109, -82,  43, 107,  43, -68,-112, -14, 111,},
{  72, -13, -73, -44,  38,  73,  11, -65, -53,  28,  73,  26, -55, -70,   8,  83,  44, -53, -80,  -5,  81,  65, -47, -97, -15,  88,  64, -39, -74, -27,  61,  73, -22, -92, -25,  79,  66, -36, -89, -28,  70,  85, -14,-100, -46,  66,  93, -12,-102, -46,  60,  92,  10,-101, -64,  57,  93,  14, -85, -72,  43, 100,  21, -87,},
{ -93,  20, 101,  52, -73, -94,  19,  99,  41, -79, -87,  30, 104,  33, -77, -85,  30,  97,  24, -80, -80,  32, 118,  29,-107, -72,  64,  95, -15, -97, -37,  70,  79, -18, -93, -32,  77,  66, -38, -82, -17,  72,  64, -42, -72,   3,  56,  33, -39, -49,  17,  49,  15, -38, -50,  14,  65,  30, -53, -62,  19,  73,  25, -65,},
{ -52,  17,  58,  16, -45, -40,  28,  56,  -9, -62, -20,  52,  52, -28, -58,  -9,  50,  38, -34, -60,   3,  65,  41, -62, -68,  41,  77,  -3, -78, -31,  58,  66, -30, -83,   1,  93,  34, -86, -73,  46,  98,  12, -94, -69,  72, 107, -41,-116, -14, 106,  68, -71,-104,  24, 107,  24, -91, -73,  57, 105,  -7,-122, -47, 108,},
{  96, -37,-108, -21,  99,  62, -68, -91,  26, 110,  15,-111, -55,  87,  89, -41,-107,  -6,  96,  48, -72, -92,  58, 110, -38,-105,  15,  92,  16, -73, -52,  48,  78, -27, -82,  -6,  77,  40, -65, -64,  39,  73,   0, -73, -33,  72,  40, -52, -50,  29,  63, -10, -65, -10,  54,  26, -32, -44,  14,  60,   7, -69, -29,  59,},
{  46, -29, -53,   7,  57,  20, -51, -43,  40,  55, -24, -59,   5,  60,  14, -51, -33,  43,  45, -26, -59,   1,  82,  12, -91, -28,  91,  45, -85, -57,  65,  76, -41,-100,  35, 107, -17,-111, -15, 104,  50, -85, -76,  50,  92, -15,-107,   0, 106,  18, -90, -48,  65,  84, -53, -96,  33, 107,  -7,-107, -23, 100,  57, -90,},
{ -84,  53,  98, -27,-110,  -3, 115,  31,-107, -57, 102,  70, -79, -90,  50, 106, -23,-111,  -5, 105,  35,-109, -48, 114,  49,-105, -50,  84,  53, -57, -67,  38,  78, -36, -80,  36,  71, -21, -64,   5,  59,  10, -56, -26,  47,  39, -44, -46,  43,  51, -33, -58,  25,  60, -14, -62,   1,  59,  10, -47, -26,  48,  38, -50,},
{  71, -53, -75,  38,  78, -22, -80,  13,  80, -13, -72,   6,  63,  12, -63, -31,  65,  37, -58, -44,  64,  49, -72, -56,  77,  49, -72, -47,  72,  52, -68, -61,  62,  76, -76, -71,  77,  63, -62, -64,  51,  68, -32, -82,  29,  88, -25, -96,  14,  99,   3,-102, -16,  98,  30, -94, -45,  91,  54, -79, -67,  86,  72, -87,},
{  88, -78, -85,  65,  95, -58,-106,  70, 102, -81, -85,  66,  76, -47, -71,  34,  73, -32, -72,  28,  76, -39, -76,  57,  78, -69, -72,  71,  61, -66, -58,  55,  67, -62, -58,  77,  36, -78, -29,  73,  31, -59, -37,  48,  32, -50, -21,  43,  31, -43, -41,  38,  60, -45, -73,  54,  76, -65, -67,  71,  59, -70, -62,  69,},
{ -63,  58,  55, -55, -58,  61,  57, -69, -49,  78,  36, -77, -41,  90,  37,-102, -29, 103,  38,-115, -28, 133, -27,-114,  78,  71,-101, -30, 102,  11,-102,  -6, 107, -11,-102,  26,  92, -28, -84,  24,  80, -24, -75,  21,  77, -34, -69,  42,  62, -46, -61,  49,  52, -49, -40,  41,  33, -37, -30,  34,  28, -28, -35,  34,},
{ -22,  27,  12, -29,  -3,  22,   2, -22,   6,  22, -20, -13,  34,  -8, -27,  27,  14, -31, -14,  38,   8, -52,  34,  38, -75,   9,  77, -51, -64,  68,  54, -76, -49, 103,  10,-104,  28,  85, -41, -69,  38,  68, -36, -78,  48,  90, -91, -58, 108,  34,-102, -38,  99,  38,-117, -26, 149,  -4,-155,  35, 137, -67,-110,  92,},
{ -67,  74,  55, -84, -44,  95,  24,-101,   7,  99, -36, -99,  61,  97, -88, -79,  99,  72,-115, -66, 165, -11,-159,  92,  94,-122, -16, 103, -36, -66,  41,  53, -41, -51,  64,  25, -71,   2,  57, -14, -50,  23,  41, -23, -37,  36,  21, -39, -10,  30,   2, -13,  -7,  15,  12, -29,  -3,  30, -19, -14,  32,  -7, -23,  15,},
{  28, -35, -17,  44,   9, -49,   0,  59, -23, -62,  60,  33, -77,  -3,  82, -23, -74,  34,  75, -57, -59, 100, -16, -84,  67,  35, -83,  23,  63, -52, -42,  70,  18,-103,  65,  69,-116,  -3, 102, -26, -80,  28,  78, -33, -86,  71,  62,-101, -27, 113, -12,-110,  41,  98, -73, -72, 101,  28,-100,   8,  80, -25, -65,  47,},
{  73, -96, -36, 107,   3,-103,  35,  86, -74, -47,  92,   3, -86,  37,  55, -54, -32,  66,   7, -73,  42,  35, -70,  20,  63, -72, -10,  84, -53, -53,  80,  28, -99,  18,  81, -66, -32,  81,  -1, -84,  23,  79, -41, -71,  72,  46, -91, -15, 100, -30, -93,  71,  67,-103, -11, 104, -45, -69,  75,  20, -69,  20,  41, -30,},
{ -58,  81,  24, -98,  16,  95, -62, -70, 103,  11,-102,  39,  76, -65, -47,  87,  -1, -87,  46,  64, -86,   6,  68, -67,   6,  57, -75,   7,  94, -72, -58,  99,  -4, -98,  93,  15,-103,  55,  69, -86, -31,  97, -11, -90,  59,  58, -84, -16,  82, -20, -67,  48,  36, -60,  -3,  64, -34, -50,  66,  13, -67,  24,  36, -28,},
{ -16,  59, -51, -31,  73, -44,   2,  34, -36,  -9,  10,  34,  -2, -63,  53,  10, -65,  82, -22, -82, 100, -33, -19,  53, -67,  38,   4, -62, 112, -52, -79, 118, -48, -47, 116, -95, -16,  93, -71,  16,  31, -42,  11,  -6,  29,  19, -93,  65,  50,-131,  87,  68,-154,  52,  94,-125,  52,  54, -95,  29,  34, -35,  25, -12,},
{  20, -17, -31,  52,  -9, -58,  77,  -2, -83,  61,  26, -60,   7,  47, -20, -53,  61,  30, -96,  30,  72, -94,  43,  52,-117,  59,  62,-123,  71,  63,-113,   9,  86, -57, -19,  60, -27, -48,  42,  63, -90, -27, 112, -54, -59, 104, -36, -61,  63,  10, -34, -12,  42,  -1, -67,  56,  55,-113,  23, 100,-104, -13, 112, -69,},
{ -78, 130, -16,-111,  67,  60, -77, -16,  81, -50, -43, 101, -26, -97,  96,  17,-102,  74,  45,-102,  28,  52, -59,  29,  19, -58,  35,  31, -50,  -6,  52, -21, -45,  69, -20, -54,  69, -15, -57,  72,  -2, -73,  61,  20, -64,  22,  41, -48,  -6,  50, -13, -53,  47,  32, -91,  47,  71,-119,  23, 109,-114, -12, 115, -71,},
{  34, -53,   1,  67, -70,  -9,  88, -71, -22,  71, -24, -28,  11,  35, -34, -32,  86, -45, -55,  97, -51, -18,  73, -83,  25,  55,-107,  71,  62,-140,  48, 100,-143,  55,  68,-117,  56,  49, -84,  24,  51, -52, -10,  38,   4, -49,  27,  34, -69,  25,  71, -94,  -4, 103, -93,   0,  91, -91,   4,  70, -65,   6,  35, -21,},
{  23, -32,  -9,  45, -37,  -2,  46, -52,   8,  41, -49,  21,  24, -52,  23,  34, -42,   3,  19,  -8,  -5,  -3,  24, -41,  21,  40, -85,  59,  23, -85,  69,   4, -80, 115, -77, -30, 134,-140,  13, 147,-153, -24, 163,-103, -48, 124, -72, -41,  97, -54, -29,  68, -34, -37,  65, -11, -53,  55,  -2, -56,  51,  19, -64,  34,},
{  20, -30,  11,   4,   6, -32,  28,  21, -63,  62,  -8, -76, 102, -19, -88, 106, -33, -54,  90, -51, -17,  47, -46,  38, -19,  -8,  35, -55,  54,  -3, -64,  78, -31, -41,  91, -90,  28,  71,-121,  53,  80,-128,  31, 100,-120,  14, 105,-125,  23, 103,-123,  25,  81,-101,  41,  42, -74,  37,  22, -52,  36,  -2, -21,  13,},
{ -56,  87, -10, -76,  63,  35, -99,  57,  42,-109,  79,  42,-126,  69,  62,-131,  66,  58,-109,  46,  60,-108,  69,  22, -95,  96, -37, -40,  89, -66, -12,  68, -65,  24,  22, -48,  46, -14, -19,  27, -10, -10,   6,  12,  -2, -25,  36, -17, -36,  76, -47, -39,  91, -62,  -6,  72, -89,  29,  63,-103,  50,  49,-102,  54,},
{  63,-113,  35,  91,-127,  40,  83,-135,  72,  46,-116,  84,  21,-107,  85,  26,-109,  82,  20, -99,  90, -22, -53,  95, -78,  13,  54, -79,  54,   9, -55,  41,  -2, -18,  19,  -6,  -7,  -2,  19, -10, -22,  45, -28, -31,  82, -62,  -8,  68, -85,  43,  39, -93,  64,   9, -61,  72, -38, -19,  59, -63,  22,  46, -81,  41,},
{ -57, 123, -83, -40, 124, -97, -11, 112,-130,  55,  58,-125,  89,  17,-101, 102, -35, -44,  83, -58,  -1,  41, -46,  30,  -6,  -9,  10, -16,  30, -25,  -5,  28, -35,  29, -13, -16,  51, -67,  38,  36, -92,  77,  -3, -76, 101, -43, -46,  97, -80,   2,  85,-106,  38,  51, -86,  60,  -7, -39,  61, -48,   2,  45, -56,  25,},
{  17, -43,  44, -10, -39,  66, -49,  -3,  57, -80,  51,  12, -59,  63, -29, -19,  59, -63,  27,  21, -54,  64, -47,   1,  48, -72,  64, -19, -49,  83, -58,   2,  48, -75,  71, -45,  -3,  64, -96,  67,  14, -93, 103, -34, -54, 105, -97,  35,  50,-112, 100, -19, -61,  98, -93,  52,   9, -76, 115, -88,  -5, 103,-130,  61,},
{ -58, 137,-133,  37,  77,-129, 107, -29, -57, 106, -98,  39,  47, -97,  74, -13, -47,  90, -93,  47,  19, -65,  82, -79,  51,   3, -59,  92, -81,  31,  32, -78,  86, -54,   4,  43, -79,  87, -45, -25,  70, -63,  13,  37, -51,  33,  -7, -12,  15, -10,   7, -10,   7,   8, -30,  44, -29, -18,  66, -73,  15,  70,-101,  48,},
{ -26,  65, -70,  28,  30, -62,  66, -49,  15,  27, -62,  74, -48,  -9,  64, -92,  81, -30, -41,  98,-110,  76, -22, -35,  91,-120,  93, -21, -58, 105,-108,  66,   4, -63,  96,-100,  71, -17, -49, 100,-101,  46,  28, -83,  88, -46,  -6,  43, -62,  58, -26, -21,  57, -68,  53,  -6, -48,  79, -77,  33,  28, -56,  49, -20,},
{  37, -88,  93, -45, -26,  74, -80,  56, -17, -29,  66, -79,  64, -27, -17,  56, -79,  70, -34, -13,  56, -83,  87, -65,  16,  46, -90,  96, -74,  30,  28, -72,  87, -78,  62, -29, -26,  76, -97,  82, -26, -44,  88, -83,  45,  11, -65,  87, -71,  17,  53,-102, 100, -56,  -1,  46, -67,  72, -61,  20,  41, -91,  97, -42,},
{ -27,  70, -78,  44,   6, -46,  67, -68,  52, -18, -23,  58, -73,  65, -33, -15,  64, -88,  67, -16, -35,  70, -80,  72, -56,  31,   1, -31,  61, -84,  72, -30, -18,  57, -78,  91, -90,  58,  -5, -53,  99,-107,  72, -14, -42,  82,-100,  77, -21, -36,  81,-101,  85, -44, -12,  73,-103,  93, -55,  -4,  65,-104,  92, -35,},
{  35, -90, 119,-101,  45,  19, -69, 100,-111,  93, -46, -23,  90,-130, 129, -84,  13,  53, -92,  96, -78,  52, -24, -13,  48, -62,  56, -48,  34,  -3, -40,  70, -74,  58, -36,  15,  14, -50,  74, -73,  48,  -8, -31,  56, -69,  62, -34,  -6,  47, -72,  75, -56,  17,  29, -67,  81, -66,  32,  11, -51,  69, -70,  53, -19,},
{ -28,  74, -99,  91, -56,   6,  43, -78,  96, -96,  75, -39,  -3,  44, -68,  76, -69,  46, -12, -19,  41, -53,  56, -54,  38,  -9, -10,  20, -41,  63, -67,  48, -11, -27,  56, -79,  91, -88,  71, -38, -10,  63, -99,  99, -70,  20,  33, -68,  87, -88,  64, -24, -20,  63, -94,  98, -71,  31,  11, -60, 102,-122, 101, -38,},
{ -17,  43, -64,  73, -68,  50, -20, -17,  51, -79,  95, -86,  54,  -7, -40,  76, -99, 103, -87,  56, -21, -14,  48, -75,  94,-102,  88, -59,  21,  25, -71,  97, -99,  87, -72,  55, -27, -10,  42, -65,  76, -70,  43,  -2, -33,  58, -77,  79, -65,  38,  -6, -30,  61, -78,  76, -52,  16,  20, -54,  80, -95,  98, -77,  29,},
{  19, -54,  83, -93,  85, -65,  36,  -5, -27,  55, -79,  92, -83,  56, -27,  -9,  53, -95, 120,-130, 124,-113, 101, -78,  46, -14, -16,  42, -67,  83, -77,  53, -20, -10,  32, -55,  82,-105, 117,-109,  88, -62,  24,  18, -42,  49, -50,  44, -25,  -6,  37, -56,  60, -54,  41, -27,  14,   4, -24,  35, -39,  40, -29,  10,},
{ -16,  47, -71,  81, -77,  67, -52,  36, -20,   3,  19, -47,  70, -79,  75, -67,  56, -40,  15,  11, -36,  62, -84,  95, -99,  92, -77,  61, -38,   5,  30, -56,  72, -84,  91, -94,  90, -74,  51, -25,  -7,  48, -86, 109,-108,  93, -72,  47, -13, -20,  40, -52,  58, -56,  41, -14, -17,  46, -73,  97,-106,  93, -62,  21,},
{  15, -41,  56, -58,  45, -27,   7,  12, -30,  50, -75,  97,-110, 113,-107,  95, -83,  73, -58,  44, -37,  36, -37,  45, -53,  57, -59,  62, -68,  75, -80,  76, -65,  57, -57,  58, -60,  68, -77,  86, -94,  96, -93,  82, -59,  33,  -6, -13,  25, -33,  31, -20,   2,  17, -39,  60, -77,  86, -92,  93, -85,  71, -48,  17,},
{  11, -31,  48, -56,  56, -58,  59, -54,  45, -35,  26, -18,  10,  -5,   5,  -8,   9,  -8,  13, -21,  26, -38,  58, -71,  82, -96, 106,-113, 121,-128, 121, -96,  63, -38,  23, -10,  -7,  26, -46,  64, -73,  77, -70,  48, -15, -22,  58, -87, 106,-117, 122,-116,  94, -67,  39,  -7, -19,  37, -52,  63, -69,  68, -53,  20,},
{ -22,  60, -92, 111,-120, 120,-107,  90, -73,  52, -30,   5,  25, -52,  70, -84, 101,-120, 126,-115,  99, -89,  81, -70,  54, -38,  23,  -5, -15,  35, -53,  64, -67,  67, -70,  79, -88,  90, -83,  71, -58,  46, -33,  18,  -1, -15,  26, -31,  30, -24,  14,  -1, -12,  24, -34,  42, -46,  44, -38,  25, -11,   5,  -2,   0,},
{   0,  -4,  13, -23,  31, -36,  40, -44,  47, -54,  67, -75,  69, -58,  47, -38,  29, -19,  10,   1,  -9,  18, -29,  41, -50,  51, -48,  52, -58,  57, -49,  39, -25,  14, -10,   8,   0, -18,  42, -63,  81, -98, 114,-124, 131,-135, 131,-120, 100, -73,  39,  -4, -26,  48, -64,  81, -94, 101,-102,  97, -85,  74, -55,  21,},
{  -6,  15, -27,  42, -54,  63, -72,  78, -79,  77, -77,  80, -77,  67, -60,  54, -46,  38, -28,  11,   6, -19,  27, -36,  47, -60,  69, -79,  88, -93,  92, -84,  74, -67,  64, -63,  60, -55,  51, -51,  52, -49,  44, -34,  20,  -7,  -5,  18, -37,  56, -71,  87, -99, 108,-110, 111,-111, 106, -95,  76, -50,  32, -22,   9,},
{ -15,  40, -61,  78, -92,  99, -98,  94, -91,  89, -89,  89, -82,  72, -64,  56, -51,  50, -47,  41, -36,  30, -25,  24, -19,  11,  -9,  11, -12,  10,  -7,   0,   8, -14,  16, -17,  19, -25,  32, -37,  43, -50,  56, -64,  72, -80,  93,-105, 111,-114, 116,-114, 106, -98,  92, -86,  74, -60,  48, -38,  27, -15,   8,  -2,},
},

{
{  17,  19,  21,  23,  24,  25,  27,  28,  30,  32,  33,  35,  37,  40,  43,  45,  48,  50,  51,  53,  54,  55,  56,  57,  59,  61,  63,  64,  65,  67,  68,  69,  70,  71,  73,  73,  74,  75,  75,  76,  77,  77,  78,  78,  79,  79,  79,  80,  80,  80,  80,  80,  80,  81,  80,  80,  80,  78,  78,  77,  76,  76,  75,  73,},
{ -46, -52, -58, -63, -67, -71, -75, -79, -82, -85, -88, -90, -91, -92, -92, -92, -91, -90, -89, -87, -84, -81, -78, -74, -69, -65, -61, -56, -51, -46, -40, -34, -28, -23, -16, -10,  -5,   0,   5,  11,  16,  21,  26,  32,  37,  42,  46,  51,  55,  58,  61,  64,  67,  69,  70,  71,  72,  72,  72,  72,  72,  69,  67,  65,},
{ -71, -77, -83, -88, -92, -94, -96, -95, -94, -90, -85, -78, -71, -61, -50, -38, -27, -17,  -6,   4,  14,  25,  36,  46,  55,  62,  68,  73,  78,  82,  85,  86,  85,  84,  82,  79,  75,  71,  65,  59,  53,  47,  41,  34,  25,  16,   7,  -2, -11, -19, -27, -35, -42, -49, -56, -61, -66, -69, -72, -73, -73, -73, -72, -70,},
{ -75, -83, -88, -91, -91, -89, -83, -74, -64, -50, -35, -20,  -3,  14,  32,  49,  63,  74,  83,  89,  92,  93,  91,  87,  80,  70,  60,  47,  33,  19,   4, -13, -27, -41, -54, -64, -73, -79, -83, -86, -86, -84, -81, -76, -70, -63, -55, -45, -35, -24, -12,   0,  11,  23,  35,  45,  53,  60,  65,  69,  72,  73,  72,  70,},
{ -74, -80, -83, -81, -76, -67, -54, -35, -14,   7,  28,  46,  64,  78,  88,  93,  94,  89,  80,  67,  51,  32,  11, -11, -31, -49, -65, -77, -86, -91, -93, -90, -82, -72, -60, -43, -25,  -7,  11,  28,  44,  58,  69,  77,  84,  88,  89,  85,  78,  70,  59,  46,  32,  15,  -1, -18, -33, -46, -57, -66, -71, -74, -75, -73,},
{ -82, -90, -90, -85, -72, -53, -29,  -1,  27,  53,  73,  87,  94,  93,  84,  69,  50,  28,   5, -21, -46, -67, -84, -93, -97, -90, -74, -54, -34, -12,   9,  32,  53,  70,  81,  88,  89,  83,  74,  59,  41,  22,   1, -21, -42, -59, -72, -81, -86, -86, -81, -71, -59, -43, -25,  -8,  10,  27,  43,  54,  64,  70,  73,  71,},
{ -86, -93, -88, -73, -50, -18,  17,  50,  75,  92,  97,  93,  77,  53,  24,  -7, -34, -58, -75, -86, -89, -80, -65, -42, -14,  16,  48,  73,  88,  94,  91,  80,  61,  37,  12, -15, -42, -63, -78, -86, -86, -80, -66, -47, -25,  -3,  18,  38,  57,  70,  80,  85,  82,  73,  59,  40,  19,  -3, -24, -43, -60, -71, -76, -75,},
{ -99, -98, -83, -57, -22,  19,  57,  85,  97,  96,  82,  54,  19, -18, -51, -75, -89, -89, -76, -53, -21,  12,  42,  68,  84,  88,  79,  57,  29,  -1, -29, -52, -70, -80, -82, -75, -58, -33,  -4,  26,  51,  70,  82,  86,  79,  65,  46,  20,  -6, -32, -57, -75, -85, -87, -82, -67, -45, -21,   5,  31,  55,  72,  82,  83,},
{ -85, -80, -60, -32,   3,  38,  67,  83,  81,  64,  37,   2, -34, -64, -79, -79, -68, -44, -12,  23,  57,  81,  90,  86,  65,  31, -13, -54, -83, -93, -89, -70, -38,  -4,  31,  62,  81,  89,  86,  69,  41,  10, -24, -55, -78, -90, -92, -77, -52, -22,  11,  43,  72,  92,  97,  90,  72,  45,  13, -19, -49, -70, -82, -85,},
{ -83, -80, -56, -19,  23,  61,  85,  90,  72,  37,  -8, -53, -85, -94, -79, -45,  -5,  37,  70,  90,  91,  70,  34, -11, -53, -82, -95, -88, -62, -24,  18,  57,  83,  92,  84,  60,  24, -14, -48, -75, -88, -85, -68, -39,  -4,  31,  60,  81,  87,  76,  52,  19, -16, -51, -75, -88, -85, -67, -39,  -6,  29,  58,  75,  80,},
{ -86, -79, -47,  -2,  45,  83,  98,  85,  48,  -3, -55, -93,-104, -85, -40,  19,  68,  98, 101,  75,  27, -27, -73, -98, -97, -68, -17,  35,  70,  87,  82,  54,  14, -27, -59, -80, -83, -63, -30,   8,  44,  70,  84,  79,  55,  21, -15, -49, -72, -80, -71, -48, -17,  21,  53,  74,  79,  69,  48,  18, -16, -46, -66, -73,},
{ -86, -70, -28,  21,  64,  85,  76,  43,  -6, -54, -83, -83, -53,  -4,  47,  83,  90,  67,  22, -34, -79, -95, -81, -43,   8,  55,  89,  93,  65,  19, -31, -74, -88, -80, -53,  -6,  42,  77,  88,  76,  46,   4, -42, -77, -91, -82, -53, -10,  32,  66,  85,  85,  63,  21, -21, -57, -81, -89, -78, -45,   1,  44,  74,  90,},
{-118, -85, -21,  47,  97, 107,  73,  14, -52, -97, -98, -57,   6,  64,  96,  91,  54,  -3, -57, -89, -86, -50,   7,  57,  83,  80,  42, -17, -64, -84, -75, -40,  13,  62,  85,  77,  48,   0, -45, -73, -82, -65, -29,  17,  56,  78,  75,  53,  17, -23, -57, -74, -72, -48, -13,  27,  58,  77,  77,  55,  14, -30, -64, -80,},
{ -80, -51,  -3,  44,  73,  68,  31, -17, -60, -77, -58,  -9,  47,  82,  76,  33, -24, -68, -80, -54,  -4,  46,  76,  73,  36, -22, -79, -97, -67, -11,  47,  90,  94,  64,  10, -53, -93, -97, -65, -10,  45,  83,  94,  73,  23, -35, -80, -92, -74, -34,  15,  58,  82,  84,  57,  13, -35, -83,-104, -87, -38,  27,  78, 104,},
{-101, -63,   4,  66,  95,  73,  11, -54, -88, -74, -20,  46,  88,  81,  30, -37, -83, -85, -44,  20,  74,  89,  57,  -3, -55, -84, -77, -24,  41,  82,  88,  52, -10, -61, -85, -80, -30,  35,  79,  87,  61,  14, -43, -83, -87, -56,   1,  55,  82,  77,  46,  -2, -49, -79, -76, -47,   5,  65,  94,  85,  44, -15, -63, -90,},
{ -93, -48,  22,  75,  85,  44, -23, -70, -74, -37,  20,  68,  76,  38, -25, -75, -82, -42,  21,  71,  81,  47, -13, -67, -82, -51,  15,  74,  85,  50, -12, -71, -90, -62,   1,  66,  91,  69,  17, -41, -82, -85, -50,  11,  68,  97,  79,  17, -48, -88, -88, -50,   9,  68,  93,  79,  28, -42, -91,-100, -67,   1,  64, 100,},
{ -93, -39,  36,  83,  81,  24, -48, -89, -70,  -5,  64,  93,  57, -19, -79, -87, -42,  30,  82,  80,  30, -36, -81, -73, -24,  48,  98,  72,  -5, -67, -87, -58,   8,  69,  86,  54,  -6, -68, -89, -64,  -3,  58,  92,  79,  18, -52, -95, -82, -32,  29,  77,  91,  60,  -6, -67, -90, -68,  -6,  57,  86,  68,  13, -40, -75,},
{-108, -35,  55, 102,  79,  -6, -88,-105, -43,  58, 113,  78, -15, -91, -96, -30,  51,  92,  71,   2, -67, -92, -53,  25,  82,  78,  17, -58, -83, -51,   9,  65,  82,  47, -23, -75, -81, -30,  35,  76,  68,  20, -42, -79, -67, -17,  44,  80,  68,  22, -40, -79, -75, -24,  42,  82,  70,  15, -44, -75, -63, -13,  35,  65,},
{ -74, -24,  41,  78,  51, -24, -75, -60,   4,  69,  78,  18, -54, -79, -36,  35,  76,  60,  -4, -66, -80, -37,  45,  93,  64, -15, -93, -92, -14,  65, 100,  63, -25,-105, -94,  -4,  71,  93,  50, -22, -79, -84, -30,  41,  88,  82,  16, -65, -99, -69,   4,  69,  97,  60, -18, -85, -87, -36,  32,  82,  76,  22, -32, -71,},
{ -78, -18,  59,  82,  31, -49, -87, -45,  42,  93,  56, -34, -92, -63,  24,  86,  68,  -7, -72, -81, -27,  57,  93,  46, -41, -98, -62,  26,  88,  80,   3, -81, -95, -36,  57, 106,  56, -29, -86, -84, -16,  66,  95,  53, -29, -90, -80, -10,  63,  91,  54, -17, -76, -82, -32,  37,  80,  68,   3, -57, -69, -35,  19,  61,},
{ -60, -14,  55,  63,  23, -41, -83, -30,  50,  84,  43, -65, -97, -33,  56, 109,  44, -71,-103, -54,  52, 127,  48, -69,-106, -49,  77, 110,  17, -62, -92, -38,  74,  94,  25, -54, -86, -31,  45,  58,  42,  -8, -54, -35, -20,  16,  55,  37,   3, -35, -71, -36,  28,  74,  77,  12, -85, -96, -30,  53, 111,  60, -34, -88,},
{-125,  21, 107,  93, -30,-138, -55,  66, 114,  41, -97, -87,   5,  76,  74, -38, -86, -15,  37,  66,  24, -73, -44,  17,  42,  42, -36, -64,  22,  40,  24,  -8, -75, -17,  69,  52,  11, -73, -98,  27,  89,  73,   1,-117, -81,  39,  85,  77, -20,-111, -49,  28,  74,  65, -31, -84, -22,  22,  49,  39, -44, -43,  11,  24,},
{ -73,   0,  68,  63,  -4, -73, -64,  17,  80,  57, -28, -87, -48,  44,  82,  34, -51, -78, -26,  53,  87,  26, -73, -86, -12,  70,  82,   7, -73, -75,  -6,  74,  80,   1, -76, -80, -10,  77,  89,  17, -72, -97, -31,  67,  98,  37, -55, -96, -49,  39,  89,  70, -14, -94, -86,  -9,  76,  98,  33, -55, -89, -57,  17,  83,},
{ -42,   4,  42,  28,  -8, -43, -33,  28,  52,  17, -37, -56,   0,  57,  45, -11, -61, -40,  22,  63,  46, -31, -85, -31,  52,  73,  15, -60, -67,   2,  67,  68,  -1, -88, -77,  29,  93,  61, -27, -93, -62,  26,  89,  73, -28,-100, -72,  25, 109,  85, -29,-111, -88,  24, 119,  93, -26,-121, -93,  28, 120,  92, -12,-114,},
{  77, -15, -83, -46,  49,  79,   9, -69, -59,  29,  75,  23, -62, -70,  21,  92,  38, -59, -90,  -9, 100,  70, -57,-101, -23,  93,  85, -41, -87, -37,  48,  92,  15, -97, -68,  56,  91,  16, -71, -73,   4,  77,  69, -29, -87, -43,  46,  85,  23, -65, -74,  -8,  74,  63, -17, -78, -45,  42,  82,  23, -66, -94,  -7,  95,},
{  92, -32,-101, -31,  77,  80, -31, -93, -27,  76,  76, -36, -98, -14,  88,  68, -46, -93, -22,  85,  76, -40, -98, -21,  84,  63, -53, -70,  12,  62,  39, -37, -77, -15,  72,  73, -25, -89, -38,  57,  82,   9, -80, -65,  35,  82,  34, -65, -79,   7,  77,  57, -29, -87, -22,  66,  63, -18, -81, -37,  58,  91,  12, -93,},
{  97, -48,-100,  -9,  91,  56, -66, -78,  17,  92,  30, -90, -60,  57,  83,  -8, -94, -42,  70,  82, -21, -98, -32,  84,  69, -52, -70,   9,  60,  39, -40, -74,   4,  79,  38, -64, -67,  28,  78,  17, -59, -63,  21,  86,  21, -65, -66,  28,  89,  25, -72, -71,  13,  90,  51, -64, -89,   2,  86,  56, -45,-100, -22,  93,},
{ -78,  44,  88, -11, -81, -30,  67,  58, -45, -77,  18,  83,  12, -79, -42,  67,  68, -33, -81,  -6,  87,  44, -79, -83,  58,  97, -33, -95,  -8,  88,  61, -53,-106,   6, 113,  33, -83, -70,  44,  91,  10, -82, -63,  51,  85,   1, -83, -45,  51,  70,  -8, -64, -41,  43,  75,  -5, -85, -48,  66,  71, -21, -85, -29,  77,},
{  71, -53, -71,  27,  75,  -1, -76, -19,  65,  40, -55, -55,  37,  66, -16, -73, -13,  68,  43, -53, -69,  33,  91, -13, -93,   1,  82,  22, -69, -46,  41,  75, -10, -96, -13,  95,  38, -85, -70,  51, 100,   2, -92, -51,  59,  91, -29,-107, -15,  90,  68, -49, -98,   3,  91,  37, -72, -75,  44,  98,  -6,-106, -45,  96,},
{  87, -62, -89,  37,  95,  -8,-105, -15, 104,  29, -94, -37,  81,  53, -70, -76,  54,  91, -21,-101,  -6, 107,  20,-109, -32, 103,  43, -85, -59,  58,  82, -27,-108,  16, 105, -11, -78, -13,  62,  41, -47, -56,  19,  65,  10, -70, -27,  63,  44, -43, -59,  12,  62,  18, -55, -40,  40,  64, -21, -85,  -1,  90,  34, -73,},
{ -46,  37,  51, -29, -62,  24,  71, -24, -69,  17,  61, -10, -61,  -1,  65,  17, -64, -29,  60,  43, -66, -47,  69,  53, -79, -52,  81,  51, -72, -60,  51,  79, -23,-102,   4, 124, -12,-115,  -6, 102,  46, -83, -70,  54,  81, -28, -98,  18,  95,  20, -83, -63,  65,  93, -53, -99,  35,  98, -11, -93, -19,  92,  49, -82,},
{ -89,  94,  73, -91, -77,  87,  83, -89, -80,  90,  73, -79, -73,  68,  77, -59, -83,  44,  93, -38,-102,  51,  89, -57, -72,  57,  65, -52, -74,  42,  78, -26, -77,  16,  71, -18, -67,  27,  59, -15, -59,  -7,  57,  28, -59, -38,  57,  49, -47, -66,  30,  86, -22, -78,   5,  81,   2, -81,  -8,  62,  27, -56, -40,  53,},
{  65, -78, -34,  74,  33, -73, -40,  85,  27, -84, -15,  70,  12, -62,  -7,  63,   3, -65,  -4,  67,   0, -66,  -1,  74,  -4, -78,  23,  76, -50, -65,  44,  64, -32, -80,  37,  98, -69, -83,  74,  69, -51, -77,  38,  92, -43,-102,  43,  98, -17, -95, -15, 102,  32, -99, -50,  99,  58,-101, -50,  84,  54, -72, -61,  69,},
{ -56,  67,  29, -67, -18,  75,  -3, -70,  21,  62, -21, -71,  24,  85, -42, -77,  37,  70, -22, -77,  33,  87, -85, -52, 118,   2,-110,  10, 101,   3,-106, -11, 144, -57,-113, 112,  51,-102, -34,  73,  52, -67, -59,  70,  51, -78, -32,  65,  30, -40, -54,  35,  68, -40, -68,  43,  54, -39, -39,  34,  46, -41, -54,  51,},
{ -35,  48,  20, -59,  -9,  68, -12, -67,  34,  49, -42, -34,  59,  10, -63,   5,  65, -11, -68,  21,  70, -54, -45,  94, -19, -85,  73,  34, -84,  -6,  85,  -5,-101,  69,  50,-103,  31,  58, -39, -31,  23,  36, -20, -50,  17,  84, -63, -65,  84,  43, -84, -39,  95,  34,-125,  -8, 148, -36,-139,  76, 100, -78, -79,  75,},
{ -56,  71,  29, -74, -16,  85,  -7, -85,  35,  72, -49, -64,  63,  47, -71, -32,  74,  30, -82, -24, 112, -30,-108,  88,  68,-115, -10, 112, -23, -98,  28, 100, -54, -95,  93,  56, -97, -21,  83,   4, -71,   2,  64,  -1, -68,  -1,  79,  -8, -79,  12,  74,  -3, -86,  18,  88, -35, -80,  47,  58, -44, -44,  35,  46, -42,},
{ -56,  96, -11, -83,  34,  70, -46, -66,  72,  39, -95,  24,  73, -60, -43,  55,  40, -49, -55,  74,  33,-116,  63,  72,-110,  14,  74, -64,  -1,  37,  -4, -32,   2,  63, -51, -54,  89,  -1, -76,  44,  47, -46, -38,  23,  77, -47, -90, 104,  39,-123,  19, 108, -53, -82,  63,  67, -71, -50,  86,  12, -93,  32,  79, -61,},
{  46, -79,  13,  66, -37, -45,  47,  32, -67,  -1,  74, -37, -51,  53,  26, -59,  -5,  59,  -2, -68,  29,  58, -73, -10, 104, -80, -47, 126, -56, -82,  82,  47,-104,  37,  45, -85,  49,  49, -54, -50,  59,  57, -87, -33, 103,  -6, -96,  41,  83, -61, -88,  99,  48,-126,  36,  95, -93, -28,  93, -25, -57,  34,  28, -24,},
{  15,   3, -55,  45,   5, -52,  86, -43, -50,  54,  -6,  21, -28, -29,  63, -45,  -7,  80, -65, -29,  57, -38,  50, -24, -58, 100, -87, -11, 166,-129, -92, 171, -67, -43,  97, -84,   8,  42, -45,  64, -26, -69,  68,  -7,  -8,  40, -73,  15,  49, -52,  50, -13, -54,  65, -37, -19, 107, -92, -46, 108, -53, -20,  71, -46,},
{  50, -68, -23,  92, -21,-103, 103,  35,-126,  43,  79, -73, -33,  80,   1, -92,  42,  89, -95, -52, 132, -47, -72, 111, -49, -70, 132, -63, -73, 103,  19,-105,  38,  66, -74,  -4,  64, -37, -50,  74,  18, -85,  21,  60, -38, -35,  33,  36, -54, -26,  82, -12, -65,  42,  19, -40,  22,  12, -34,  23,   0, -17,  26, -15,},
{  50, -95,  28,  85, -95,  -5,  88, -68, -25,  74, -15, -64,  28,  73, -79, -28, 111, -61, -66, 106, -23, -61,  66, -19, -29,  40,  -3, -27,   4,  32, -17, -31,  48, -13, -43,  73, -35, -58,  93,  -9, -82,  48,  58, -67, -21,  63,   0, -64,  22,  71, -68, -50, 120, -34,-102, 126, -14,-118, 114,  27,-114,  46,  57, -46,},
{ -13,  31, -14, -37,  71, -50, -17,  80, -71,   2,  44, -48,  27,   2, -14,  22, -47,  45,  18, -79,  76, -22, -51, 102, -83,  -8,  98,-130,  62,  76,-125,  29,  92,-136,  68,  67,-132,  49,  69, -71, -23,  71, -11, -55,  15,  79, -64, -61, 124, -26,-108,  99,  27,-101,  57,   5, -24,  34, -35,   4,  25, -28,  24, -11,},
{ -50,  99, -39, -70,  78,  23, -91,  42,  64, -95,   3, 103, -84, -50, 122, -37, -88,  92,  28,-105,  42,  58, -80,  35,  33, -77,  49,  18, -56,  30,  31, -49,  13,  33, -44,  22,  11, -43,  21,  44, -49, -21,  67, -29, -40,  53,   3, -46,   2,  52, -11, -65,  65,  21,-101,  78,  35,-122,  69,  81,-136,  11, 125, -82,},
{ -60, 109, -40, -72,  98,  -5,-109, 123, -13,-117, 138, -16,-127, 132,  11,-141, 112,  34,-128,  80,  43,-114,  83,   7, -80,  82, -46,   5,  39, -50,  16,  24, -33,  13,   2,   0,   2, -15,  25, -15, -17,  33, -13, -22,  44, -37,  -2,  38, -47,  30,  11, -57,  60, -19, -20,  41, -44,  23,  20, -56,  40,  26, -78,  46,},
{  33, -66,  26,  57, -93,  53,  29, -97, 100, -32, -62, 110, -69, -33, 101, -66, -23,  70, -41, -26,  56, -26, -18,  32, -19,  -6,  24, -15, -10,  18,   6, -38,  40,  -6, -39,  83, -89,   5, 102, -88, -45, 122, -34,-115, 137,   0,-129, 127, -14, -96, 104, -21, -68,  82, -31, -13,  26, -17,   3, -23,  31,  36, -95,  51,},
{  39, -95,  72,  29,-105,  82,   5, -80,  85, -25, -47,  76, -32, -48,  79, -20, -62,  68,   4, -68,  75, -35, -35,  93, -89,  31,  36, -82,  74,  -4, -61,  70, -36,  -8,  45, -73,  72, -17, -62,  79,  -5, -63,  34,  39, -44, -27,  77, -46, -39,  97, -59, -47, 118, -95,   5,  91,-117,  59,  37,-102,  74,  24, -87,  49,},
{  -4,  14, -14,   1,   7,  -6,   7,  -8,   4,  -1,  -7,  21, -14, -12,  28, -20,  -3,  22, -21,   5,   1,  -7,  31, -44,  20,  19, -47,  48, -16, -24,  45, -34, -17,  85, -95,  14, 106,-164,  56, 133,-167,  -4, 154,-103, -60, 147, -77, -59, 120, -54, -61, 111, -56, -52, 103, -51, -40,  90, -55, -32,  63,  -5, -48,  30,},
{ -59, 149,-151,  35, 103,-154, 103,   3, -96, 120, -64, -28,  84, -63, -14,  70, -49, -19,  55, -33, -11,  37, -36,  17,  14, -32,  22,  -7,   4,  -1, -11,  22, -20,   5,  11, -23,  30, -30,   4,  46, -66,  24,  38, -76,  69, -16, -41,  68, -65,  17,  61, -99,  60,  13, -74,  98, -61, -10,  69, -97,  50,  65,-128,  65,},
{ -33,  82, -90,  40,  34, -87, 101, -68,  -4,  80,-110,  68,  19, -90,  98, -38, -52, 111, -91,   3,  76,-104,  86, -39, -17,  61, -84,  87, -48, -29,  86, -81,  30,  28, -60,  76, -76,  38,  31, -86,  68,   9, -72,  69, -11, -56,  87, -66,  -2,  82, -98,  25,  54, -78,  64, -28, -13,  40, -44,  24,   7, -33,  31, -11,},
{  16, -40,  40, -13, -16,  26, -29,  33, -34,  19,  21, -59,  55, -10, -42,  68, -50,  -6,  60, -69,  30,  10, -33,  55, -73,  60, -24, -23,  67, -72,  29,  32, -83, 101, -71,  12,  57,-112, 100, -12, -86, 109, -42, -62, 126,-100,   9,  84,-116,  73,   7, -66,  76, -62,  54, -37,  -2,  60,-118, 119, -28, -92, 126, -55,},
{ -30,  78, -98,  68,  -6, -47,  82, -94,  78, -30, -42, 104,-111,  52,  36,-102, 116, -67, -26, 103,-119,  81, -26, -27,  73,-101,  96, -50, -18,  72, -83,  49,  12, -61,  74, -57,  29,  -5, -12,  28, -36,  25,   3, -39,  61, -44,   4,  25, -35,  33, -19, -13,  48, -74,  70, -18, -44,  87,-106,  71,  12, -86,  95, -39,},
{  37, -94, 119, -93,  26,  35, -67,  79, -82,  66, -25, -27,  72, -87,  60,  -9, -44,  82, -82,  41,  19, -73, 107,-108,  77, -29, -27,  85,-116, 101, -44, -35,  96,-112,  94, -55,   6,  35, -63,  73, -46, -12,  61, -76,  56,  -3, -48,  67, -45,  -7,  62, -87,  72, -36,  -3,  31, -40,  48, -49,  16,  30, -55,  51, -21,},
{  -9,  30, -53,  61, -45,  13,  16, -38,  51, -51,  36, -12, -18,  54, -68,  44,   3, -44,  54, -35,   5,  21, -36,  44, -57,  68, -67,  45,   2, -59,  90, -78,  37,   3, -34,  77,-114, 114, -75,   2,  78,-121, 100, -27, -57, 112,-114,  65,   6, -64,  94,-100,  82, -44, -21,  87,-112, 100, -63,   4,  59, -99,  90, -34,},
{  30, -83, 122,-127,  93, -35, -21,  69,-106, 125,-110,  59,   7, -68, 109,-118,  83, -14, -57,  99,-112, 107, -81,  40,   3, -34,  56, -83, 102, -91,  40,  23, -69,  89, -90,  81, -63,  30,  10, -41,  53, -42,  11,  22, -42,  44, -27,   5,  10, -17,  20, -21,  15,  -5, -12,  36, -46,  34, -10, -10,  19, -15,   5,   0,},
{  -5,  10,  -8,   4,   4, -13,  16, -15,  15, -12,  11, -15,  19, -18,  10,   7, -26,  34, -23,   5,  14, -30,  35, -36,  28, -14,  10, -11,   2,  18, -33,  37, -23,   1,  22, -55,  96,-130, 132, -89,   9,  79,-130, 127, -88,  34,  15, -56,  95,-116,  94, -37, -29,  89,-129, 133,-104,  60,   0, -64, 111,-141, 119, -44,},
{  20, -53,  84,-102, 100, -81,  53, -21, -17,  56, -87, 104, -97,  62, -11, -43,  90,-115, 105, -69,  27,  18, -68, 104,-119, 118,-108,  87, -51,  -4,  59, -91,  97, -88,  75, -62,  42,  -9, -27,  54, -65,  55, -23, -18,  45, -53,  46, -33,  21,  -1, -23,  43, -54,  56, -44,  19,   4, -17,  27, -35,  44, -53,  47, -19,},
{  26, -75, 122,-154, 159,-143, 126,-109,  82, -44,  -4,  51, -86, 102, -93,  66, -27, -19,  59, -79,  85, -91,  96, -91,  80, -66,  47, -29,  11,  10, -32,  47, -49,  43, -32,  19,  -3, -17,  34, -43,  44, -36,  13,  16, -34,  38, -33,  25,  -8, -21,  49, -65,  69, -65,  52, -35,  22, -10,  -4,  14, -25,  36, -32,  12,},
{  -9,  23, -32,  40, -47,  54, -63,  73, -83,  90, -88,  76, -51,  19,   8, -28,  46, -58,  55, -39,  17,   8, -28,  40, -49,  58, -66,  74, -88,  95, -74,  37,  -1, -24,  38, -51,  65, -82,  97, -94,  66, -14, -48, 100,-122, 109, -83,  56, -18, -25,  53, -62,  66, -67,  55, -30,  -2,  36, -66,  95,-118, 122, -90,  32,},
{   7, -22,  37, -53,  66, -76,  86, -96, 103,-109, 116,-120, 114, -95,  61, -23, -17,  50, -73,  85, -87,  84, -78,  68, -55,  38, -18,  -5,  32, -67,  95,-104, 101, -93,  86, -79,  71, -62,  47, -28,   7,  13, -27,  37, -39,  31, -21,  11,   7, -31,  47, -52,  53, -54,  50, -36,  20,  -2, -19,  49, -74,  77, -55,  20,},
{  -7,  20, -35,  49, -57,  58, -56,  55, -54,  51, -44,  35, -21,   2,  17, -35,  55, -75,  84, -82,  78, -78,  82, -86,  88, -92, 102,-118, 148,-178, 174,-139, 103, -81,  67, -51,  33, -17,   1,  15, -26,  29, -28,  22,  -9,  -4,  14, -26,  33, -38,  44, -46,  43, -38,  32, -24,  17,  -6,  -5,  12, -15,  17, -14,   5,},
{   3,  -7,  10, -10,   4,   2,  -6,  12, -21,  32, -50,  72, -93, 111,-122, 126,-124, 118,-101,  76, -53,  36, -23,  14,  -6,  -2,   6, -11,  21, -32,  32, -23,  12,  -1,  -7,  19, -39,  64, -90, 111,-120, 113, -93,  60, -16, -23,  45, -57,  62, -55,  38, -16,  -7,  27, -45,  63, -80,  87, -95, 107,-109,  95, -64,  23,},
{  -1,   2,   2,  -5,   6,  -8,  16, -26,  35, -46,  64, -81,  93,-103, 109,-114, 121,-121, 104, -79,  55, -35,  22, -12,   3,   8, -18,  30, -42,  53, -59,  57, -55,  58, -63,  68, -69,  73, -82,  87, -82,  69, -50,  20,  23, -62,  86,-106, 116,-109,  87, -61,  35, -13,  -7,  27, -42,  50, -60,  69, -69,  62, -45,  17,},
{   4, -11,  16, -18,  19, -16,  12, -10,  11, -15,  20, -20,  18, -16,  14, -11,  16, -25,  31, -34,  36, -33,  31, -29,  22, -15,  15, -18,  18, -19,  19, -17,  19, -25,  27, -28,  30, -40,  62, -89, 108,-113, 104, -86,  66, -46,  25,  -2, -27,  61, -91, 107,-115, 124,-130, 134,-138, 138,-136, 126,-106,  83, -56,  21,},
{  -4,   6,  -6,  13, -20,  23, -23,  24, -27,  29, -30,  32, -36,  39, -40,  40, -38,  37, -34,  31, -32,  32, -32,  30, -22,  13, -11,  15, -15,  11,  -6,   3,  -4,   2,   9, -19,  32, -52,  74, -91, 105,-118, 126,-131, 136,-138, 145,-159, 162,-144, 115, -87,  63, -51,  46, -40,  33, -26,  19, -13,   5,   5, -10,   5,},
},

{
{  11,  14,  17,  20,  22,  24,  26,  27,  29,  31,  32,  34,  37,  39,  40,  42,  43,  45,  46,  47,  49,  51,  54,  57,  59,  60,  62,  63,  64,  66,  67,  69,  69,  70,  71,  72,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  81,  82,  82,  82,  83,  83,  83,  83,  82,  82,  82,  82,  81,  79,  78,  78,  77,  75,},
{ -37, -45, -52, -57, -61, -65, -68, -71, -74, -77, -81, -83, -86, -87, -87, -87, -86, -86, -87, -86, -84, -81, -81, -79, -75, -72, -69, -65, -61, -57, -52, -47, -42, -37, -32, -25, -18, -11,  -5,   2,   9,  15,  22,  28,  35,  40,  45,  49,  54,  58,  61,  64,  67,  69,  72,  73,  75,  75,  76,  76,  76,  75,  73,  70,},
{  44,  53,  62,  68,  74,  78,  82,  83,  85,  85,  85,  85,  82,  76,  68,  60,  52,  43,  35,  26,  15,   3,  -8, -19, -30, -39, -48, -57, -65, -71, -77, -81, -85, -88, -90, -89, -88, -86, -83, -80, -76, -69, -62, -53, -43, -32, -21, -10,   1,  11,  21,  29,  37,  45,  53,  60,  66,  71,  75,  78,  80,  80,  79,  77,},
{  53,  63,  72,  77,  79,  80,  79,  76,  71,  65,  57,  48,  37,  24,  10,  -4, -18, -31, -43, -54, -65, -74, -80, -84, -86, -84, -82, -78, -72, -64, -53, -40, -26, -13,   1,  16,  31,  45,  58,  69,  79,  86,  92,  95,  95,  92,  86,  78,  68,  57,  44,  29,  14,   0, -16, -31, -47, -60, -70, -79, -85, -87, -87, -83,},
{  74,  89,  98, 103, 103,  98,  86,  70,  51,  30,   8, -12, -33, -54, -72, -86, -97,-101,-101, -97, -90, -78, -63, -45, -25,  -5,  15,  32,  48,  60,  69,  76,  81,  82,  78,  71,  60,  47,  32,  16,  -2, -18, -34, -47, -60, -68, -73, -74, -73, -69, -62, -52, -42, -30, -15,   1,  16,  29,  42,  54,  63,  69,  71,  70,},
{  72,  82,  85,  81,  72,  58,  39,  16,  -7, -31, -50, -65, -76, -83, -84, -81, -73, -59, -41, -20,   5,  29,  53,  73,  89,  96,  97,  90,  76,  58,  34,   9, -17, -38, -59, -76, -89, -94, -92, -84, -73, -57, -37, -15,   9,  31,  48,  65,  77,  84,  86,  83,  75,  64,  49,  30,   9, -12, -33, -53, -66, -74, -78, -77,},
{  67,  74,  74,  67,  55,  37,  14,  -9, -32, -54, -70, -79, -79, -72, -57, -37, -12,  13,  36,  56,  72,  83,  85,  79,  64,  42,  14, -15, -40, -60, -73, -81, -83, -79, -67, -50, -29,  -6,  22,  49,  70,  86,  96,  95,  82,  62,  37,   8, -21, -49, -75, -93,-102,-101, -92, -74, -48, -21,   9,  38,  67,  87,  97,  96,},
{  81,  88,  85,  71,  51,  25,  -7, -39, -66, -84, -93, -91, -76, -52, -21,  13,  44,  69,  87,  93,  87,  70,  43,   9, -26, -55, -77, -90, -90, -77, -57, -32,  -1,  31,  58,  79,  88,  88,  75,  51,  21,  -9, -38, -63, -79, -84, -81, -66, -44, -16,  17,  45,  68,  84,  89,  82,  66,  44,  17, -12, -40, -63, -77, -80,},
{  71,  77,  71,  55,  28,  -6, -40, -67, -85, -87, -76, -54, -22,  16,  52,  76,  85,  83,  69,  41,   8, -27, -58, -81, -94, -90, -66, -32,   7,  40,  68,  86,  91,  84,  63,  28, -10, -45, -74, -89, -91, -79, -55, -22,  17,  52,  75,  86,  83,  72,  49,  17, -18, -51, -75, -88, -90, -77, -49, -12,  28,  60,  80,  87,},
{  86,  93,  80,  50,   9, -34, -75,-102,-104, -82, -43,   4,  48,  80,  93,  89,  67,  33,  -4, -39, -68, -86, -86, -68, -36,   3,  40,  68,  81,  77,  58,  28,  -8, -40, -66, -81, -78, -58, -24,  13,  48,  74,  86,  81,  58,  21, -18, -53, -79, -89, -82, -59, -26,  12,  47,  76,  91,  87,  64,  30,  -8, -45, -72, -84,},
{ -79, -81, -62, -24,  22,  61,  83,  87,  71,  40,  -2, -45, -77, -89, -76, -43,   1,  41,  69,  79,  72,  49,  10, -32, -65, -81, -75, -50, -14,  21,  50,  71,  79,  72,  48,   4, -45, -81, -93, -83, -54, -14,  31,  68,  89,  92,  69,  26, -22, -62, -85, -91, -75, -38,  10,  56,  91, 105,  88,  51,   2, -47, -84,-101,},
{ -69, -70, -52, -16,  27,  64,  85,  83,  56,  11, -40, -80, -98, -81, -36,  17,  63,  88,  89,  65,  22, -27, -72, -95, -87, -52,  -3,  49,  86,  99,  83,  44, -12, -68,-107,-105, -70, -22,  32,  76,  96,  88,  54,   8, -40, -80, -93, -73, -32,  10,  46,  70,  74,  59,  27, -11, -51, -78, -74, -48, -11,  25,  54,  67,},
{ -80, -72, -39,   9,  61,  91,  81,  41, -12, -59, -81, -76, -46,  -5,  36,  71,  84,  64,  23, -22, -59, -78, -75, -50,  -1,  54,  95,  94,  56,   3, -46, -82, -95, -79, -32,  29,  83, 108,  87,  34, -23, -69, -92, -85, -45,  10,  59,  90,  90,  56,   6, -44, -80, -88, -66, -24,  27,  69,  85,  74,  36, -17, -61, -81,},
{-122, -95, -29,  40,  87,  95,  69,  25, -30, -73, -82, -62, -19,  29,  63,  78,  66,  27, -25, -67, -85, -69, -17,  43,  77,  76,  52,   9, -43, -83, -84, -49,   8,  61,  85,  71,  28, -25, -69, -86, -61, -15,  38,  77,  84,  55,   2, -52, -91, -91, -51,   4,  57,  90,  87,  50,  -3, -58, -95, -88, -49,   9,  63,  89,},
{-119, -96, -28,  57, 113, 114,  58, -26, -88,-103, -73, -13,  56,  98,  89,  38, -34, -83, -90, -60,  -5,  56,  93,  84,  35, -30, -80, -90, -61,  -8,  45,  80,  80,  47, -12, -70, -88, -63, -12,  40,  78,  80,  41, -16, -70, -85, -56,  -5,  45,  73,  66,  32, -10, -52, -69, -51,  -9,  33,  58,  50,  25,  -4, -35, -49,},
{ -86, -54,   1,  55,  84,  66,   0, -68, -89, -55,   3,  60,  85,  60,   2, -53, -88, -73, -23,  35,  84,  91,  44, -20, -75, -93, -57,   8,  64,  90,  67,   8, -54, -85, -73, -27,  33,  76,  85,  46, -21, -74, -86, -55,  13,  75,  97,  62,  -8, -65, -87, -68, -21,  32,  73,  80,  49,  -2, -63,-104, -86, -21,  56, 112,},
{ -93, -51,  20,  75,  82,  35, -31, -76, -69, -20,  38,  74,  65,  18, -39, -74, -71, -28,  33,  74,  69,  30, -23, -60, -68, -33,  24,  66,  71,  33, -24, -71, -72, -30,  25,  70,  71,  30, -35, -80, -70, -20,  45,  89,  77,  17, -53, -94, -83, -22,  59, 105,  91,  28, -49,-100,-101, -50,  39, 118, 116,  41, -53,-119,},
{ -88, -46,  29,  84,  85,  25, -63,-113, -76,  15,  91, 109,  46, -48,-108, -96, -25,  59, 108,  89,  13, -69,-102, -65,  11,  71,  85,  45, -23, -75, -74, -29,  35,  72,  61,   9, -45, -69, -46,   6,  51,  69,  43, -12, -68, -81, -39,  29,  79,  76,  28, -34, -78, -70, -14,  50,  81,  54, -14, -70, -78, -35,  29,  79,},
{ -93, -37,  50,  92,  61, -13, -76, -84, -21,  61,  92,  50, -30, -89, -78,  -7,  63,  87,  52, -23, -89, -87, -14,  66,  94,  61, -21, -89, -91, -34,  56, 108,  78,  -4, -84, -98, -40,  38,  83,  72,  15, -54, -85, -58,  11,  72,  76,  30, -39, -75, -53,   0,  49,  68,  37, -21, -71, -69,  -6,  65,  86,  45, -29, -88,},
{ -61, -21,  40,  70,  38, -27, -74, -63,  18,  83,  69,   1, -66, -88, -33,  56,  99,  55, -28, -88, -84, -10,  74,  97,  39, -52,-102, -67,  28,  96,  84,   8, -74, -97, -42,  48,  92,  65,  -9, -78, -84, -21,  53,  85,  47, -25, -80, -73,  -4,  67,  84,  36, -37, -84, -69,  -2,  74,  87,  29, -44, -87, -64,  15,  88,},
{-101, -24,  73, 105,  34, -67,-102, -41,  58, 103,  47, -50, -93, -51,  36,  95,  60, -27, -84, -72,   4,  79,  82,  11, -75, -93, -22,  76, 103,  38, -64,-109, -53,  43,  95,  56, -30, -82, -62,  11,  79,  82,  10, -70, -86, -33,  40,  77,  54,  -7, -63, -70, -16,  49,  72,  34, -38, -67, -32,  25,  58,  41,  -8, -56,},
{ -31,   3,  28,  23,  -6, -26, -17,   1,  19,  23,   6, -13, -21, -19,   0,  22,  20,   2, -19, -24,   8,  28,  13, -17, -39, -19,  28,  51,  29, -23, -59, -46,  14,  70,  59, -27, -84, -58,  26,  86,  66, -13, -81, -83,  12, 103,  88, -22,-127,-104,  28, 136, 123, -11,-140,-139,  -2, 134, 128,  -4,-124,-112,   6, 121,},
{ -92, -10,  74,  79,   5, -72, -76,   3,  78,  71,  -7, -84, -73,  18,  94,  71, -33, -96, -63,  27,  91,  70, -25, -90, -55,  26,  88,  63, -41,-103, -58,  48, 114,  60, -65,-117, -38,  77, 102,  28, -71,-105, -29,  78,  95,  17, -70, -84,  -4,  67,  65,   4, -62, -64,   0,  57,  51,  -7, -40, -23,   8,  22,   8, -15,},
{ 152,  -8,-131,-103,  40, 133,  60, -85,-118, -10, 102,  93, -12,-100, -71,  41, 104,  45, -58, -97, -38,  68,  91,  12, -70, -70,  11,  78,  46, -25, -70, -51,  36,  79,  31, -51, -73,  -7,  62,  54,  -5, -58, -48,  18,  61,  38, -28, -66, -25,  35,  53,  20, -40, -48,   6,  53,  35, -27, -61, -22,  46,  54,   4, -48,},
{  60, -17, -48, -18,  27,  44,   2, -46, -39,  13,  55,  33, -33, -60, -20,  55,  66, -19, -76, -46,  46,  89,  13, -69, -64,   9,  89,  48, -59, -81, -14,  71,  81, -16, -92, -42,  64,  89,   1,-101, -81,  36, 105,  53, -59,-106, -13, 106,  83, -47,-117, -51,  81, 108,   9, -87, -88,  16, 101,  51, -54, -94, -21,  92,},
{  47, -16, -46, -17,  39,  45, -11, -59, -22,  47,  54,  -7, -66, -44,  42,  82,  11, -70, -67,  13,  79,  48, -46, -84, -12,  83,  70, -37, -84, -39,  62,  99, -16,-112, -45,  84,  99, -21,-108, -56,  62, 106,   9,-106, -64,  57,  99,   8,-102, -62,  54,  95,  15, -90, -64,  44,  90,  22, -79, -74,  35,  88,  26, -76,},
{-127,  51, 131,  20,-106, -81,  56, 116,   0,-116, -60,  74, 105,  -6,-103, -66,  58,  99,   9, -92, -68,  45, 109,  26, -99, -78,  44,  94,   8, -86, -48,  59,  80, -16, -79, -29,  56,  56, -27, -70, -10,  63,  45, -35, -51,   4,  38,  22, -36, -32,  26,  46,  -2, -44, -33,  23,  56,  18, -52, -50,  23,  60,  18, -54,},
{ -59,  26,  54,  -6, -41, -14,  37,  40, -31, -60,   2,  59,  29, -50, -40,  26,  48,  11, -51, -49,  27,  66,  17, -70, -49,  55,  74, -20, -83, -20,  61,  59, -33, -77,   5,  93,  26, -82, -65,  40,  88,  16, -91, -63,  82,  98, -45,-117, -11, 105,  63, -67,-104,  18, 112,  36,-100, -84,  61, 115,  -2,-129, -58, 116,},
{  96, -42, -91,   4,  83,  26, -71, -54,  50,  74, -15, -85, -24,  82,  63, -59, -98,  16, 105,  37, -95, -81,  76, 104, -43,-108,   3, 103,  41, -78, -72,  44,  88, -11, -89, -23,  72,  55, -58, -79,  40,  84,  -7, -79, -24,  71,  44, -46, -65,  22,  74,  -5, -70, -17,  55,  38, -37, -53,  22,  76,   1, -86, -39,  76,},
{  95, -84, -90,  68, 104, -33,-114,   0, 104,  22, -81, -52,  64,  80, -39, -98,  17,  95,  14, -82, -40,  53,  66, -28, -82,  -4,  84,  27, -73, -47,  50,  66, -25, -78,  10,  81,  11, -83, -26,  75,  44, -56, -57,  21,  70,   8, -76, -32,  71,  45, -61, -57,  49,  76, -32, -84,  15,  86,   9, -94, -23,  84,  57, -82,},
{ -88,  78,  75, -66, -86,  48, 107, -38,-114,  21, 113,   6,-106, -32,  88,  56, -77, -66,  52,  79, -35, -86,  23,  87, -14, -82,  -2,  76,  17, -66, -30,  60,  36, -58, -45,  54,  56, -41, -63,  30,  65, -15, -74,   1,  76,  17, -77, -32,  67,  48, -51, -67,  31,  82,  -6, -89, -20,  88,  45, -83, -54,  75,  67, -77,},
{ -92, 102,  49, -94, -42,  90,  38,-103, -23, 108,  20, -98, -29,  75,  43, -56, -63,  49,  77, -35, -94,  21, 100, -10,-101,  -1, 100,  20,-102, -32,  89,  46, -86, -52,  82,  55, -71, -60,  61,  55, -51, -58,  33,  76, -20, -86,  10,  89,  -5, -79,  -7,  79,   7, -65, -10,  56,  16, -43, -39,  39,  54, -45, -60,  59,},
{  74, -88, -34,  83,  30, -90, -27, 105,  10,-107,   2,  97,  -6, -92,   8,  90,  -5, -88,  -5,  91,   0, -86,   4,  88,   0, -85, -13,  89,   6, -84,   2,  77,   0, -80,  -3,  91,  -3, -92,   0,  88,  14, -76, -33,  75,  30, -77, -24,  73,  34, -64, -47,  58,  61, -59, -75,  61,  81, -64, -72,  71,  63, -70, -68,  71,},
{ -70,  95,  24, -99,   4,  95, -27, -93,  59,  67, -62, -60,  67,  49, -63, -38,  62,  29, -52, -39,  61,  36, -70, -22,  83,   8, -87,  -4,  83,   1, -82, -12,  96,   6, -93,  11,  88, -20, -85,  11,  85,   3, -90, -11,  89,   2, -82,   8,  87, -16, -87,  18,  82,  -9, -81,  -2,  96, -12,-105,  34, 104, -52,-100,  81,},
{  66,-107,   6, 107, -52, -79,  86,  49,-114,  -2, 103, -23, -98,  53,  90, -78, -71,  87,  67, -96, -62, 108,  33,-110,   7, 100, -42, -77,  66,  52, -69, -39,  71,  22, -82,   7,  81, -17, -80,  29,  77, -48, -61,  48,  53, -58, -37,  69,  22, -72,  -9,  69,   0, -65,   1,  64, -14, -51,  37,  29, -43,  -4,  26, -11,},
{ -27,  46,   0, -51,  19,  52, -40, -36,  51,  15, -47,  -5,  36,  11, -38, -20,  57,  15, -53, -22,  61,  18, -79,   5,  88, -35, -82,  52,  78, -63, -67,  61,  71, -79, -54,  96,  28,-104,  -3, 103, -11, -99,  16, 107, -40,-112,  84,  76,-110, -38, 105,  38,-106, -31, 114,  13,-119,   6, 102, -14, -86,  30,  78, -60,},
{ -67, 127, -50, -97, 105,  46,-126,  13, 119, -73, -80, 107,  32,-118,  18, 112, -70, -65,  78,  55,-120,  -1, 122, -60, -71,  76,  33, -74,  -9,  66,   1, -70,  20,  62, -45, -35,  50,  19, -48, -16,  55,   6, -70,  22,  63, -49, -39,  61,  21, -61,   0,  49, -16, -29,  12,  26, -20, -22,  41,  19, -72,  11,  74, -49,},
{  28, -50,  24,  27, -41,  -3,  46, -36, -22,  71, -24, -72,  60,  62,-103, -13, 104, -30, -89,  50,  87, -95, -32, 111, -37, -79,  81,  16, -78,  13,  63, -20, -64,  55,  30, -77,  20,  74, -70, -40,  82,  13, -69, -14,  84,  -6, -97,  58,  74,-102, -17,  98,  -8, -94,  13, 107, -61, -75,  98,  21,-100,  17,  89, -59,},
{ -59, 114, -60, -53,  88,  -7, -69,  56,  10, -66,  41,  54, -84,   0,  87, -55, -63,  98,  19,-120,  61,  65, -96,   1,  92, -48, -64,  74,  23, -84,  17,  76, -54, -47,  71,   9, -75,  38,  45, -46, -27,  44,  27, -65,   3,  74, -53, -50,  90, -11, -74,  38,  57, -51, -53,  85,  18,-107,  50,  82,-107,  -8, 100, -60,},
{ -35,  73, -52, -20,  72, -31, -61,  88, -18, -76,  77,  38,-102,  23,  79, -80, -10,  85, -46, -57,  85,  -7, -70,  57,  28, -78,  30,  71,-109,   4, 116, -76, -55,  98, -21, -66,  55,  21, -36, -39,  70,  25,-106,  50,  71,-111,  22,  83, -79, -11,  55,   4, -57,  11,  75, -84, -19, 112, -74, -41,  86, -13, -60,  38,},
{  42, -77,  42,  23, -47,  24,   5, -31,  34,   5, -36,  15,  19, -16, -13,  24,  -1, -37,  30,  32, -75,  47,  39,-111,  85,  37,-128,  75,  55,-102,  19,  76, -59, -39,  76,   0, -74,  26,  98,-117, -19, 148,-106, -53, 135, -53, -53,  51,   7, -13, -45,  75, -11, -87, 109, -11,-111, 106,   7, -83,  49,  31, -60,  27,},
{  11, -14,   8,  -6,   3,  -4,   9,  -8,  -2,  13, -16,   0,  32, -33, -14,  55, -47,   0,  47, -63,  26,  24, -26,  11, -22,  37, -20, -46, 106, -72, -57, 157,-113, -45, 154,-114, -32, 148,-126,  -2, 107, -91,   6,  39, -18,  10, -52,  56,  20,-108, 118,  -6,-131, 133,  -4, -97,  79,  -2, -38,  14,  17, -14, -10,  11,},
{ -24,  60, -60,  -6,  72, -50, -26,  65, -43,  -7,  33, -21,   6, -10,   7,  18, -34,  28,  -3, -48,  88, -60, -32, 106,-101,  25,  75,-124,  59,  70,-114,  21,  76, -67,   0,  42, -20, -43,  48,  42,-100,  32,  74, -97,  28,  80,-123,  34,  88,-105,   1,  93, -59, -50,  86,  -3, -93,  79,  31,-116,  84,  29, -91,  47,},
{  27, -61,  55, -13, -36,  43, -10, -35,  49,  -4, -47,  31,  39, -71,   3,  98,-108,   0, 127,-120,   4,  83,-102,  48,  40, -72,  30,  22, -24, -19,  36,   8, -78,  81,   6,-108, 123, -14, -98,  93,  -3, -69,  63,   1, -39,  21,  25, -42,  -6,  59, -57,  -9,  82, -81,  -6,  98,-101,   5,  97,-118,  45,  67,-113,  51,},
{ -61, 135,-113,  -2, 102, -88,  -7,  88, -88,  11,  68, -66,  -9,  80, -65, -33,  93, -44, -45,  82, -40, -32,  68, -34, -36,  66, -29, -42,  88, -46, -51, 100, -66, -15,  81, -88,  34,  39, -58,  20,  21, -18, -22,  32,  11, -58,  59,   0, -87, 111, -30, -83, 120, -51, -51, 101, -64, -18,  74, -68,  16,  42, -69,  35,},
{  24, -64,  76, -31, -34,  59, -31, -22,  49, -29, -14,  38, -21, -15,  28,  -6, -28,  20,  35, -70,  50,  10, -73,  92, -48, -36, 106, -97,   3,  95,-101,  12,  86,-116,  54,  58,-123,  67,  49,-101,  46,  59,-101,  19,  94,-109,  22,  82,-126,  71,  46,-120,  82,  22, -96,  75,   5, -54,  53, -25, -10,  31, -25,   7,},
{ -28,  89,-123,  74,  30,-108,  90,   4, -89, 105, -32, -78, 116, -39, -73, 114, -65, -27,  86, -56, -21,  69, -52, -18,  73, -51, -11,  56, -44,  -8,  41, -37,   4,  31, -42,  20,  25, -63,  50,  20, -89,  87,  -7, -83, 106, -33, -69, 115, -73, -30, 113, -87, -18,  88, -75,   5,  56, -58,  19,  17, -36,  33,  -8,  -5,},
{  12, -24,   7,  34, -63,  36,  37, -86,  66,  -1, -59,  73, -29, -46,  91, -59, -26,  93, -79, -12,  91, -99,  43,  46,-105,  87, -17, -60, 103, -70, -13,  83,-116,  93,  -9, -82, 126,-103,  23,  82,-129,  74,  21, -87,  96, -48, -24,  67, -73,  46,  12, -55,  41,  -4, -12,   0,  17,  -2, -42,  64, -21, -45,  68, -32,},
{ -15,  55, -83,  56,  19, -87,  93, -25, -60, 104, -74, -22,  95, -73, -18,  89, -88,  30,  46, -86,  65, -10, -39,  61, -50,   8,  36, -47,  29,   1, -26,  19,  12, -23,  10,  10, -28,  24,  15, -60,  59,   4, -74,  88, -27, -67, 117, -87,  -3, 103,-134,  63,  40,-101, 104, -51, -24,  84,-107,  77,   6, -91, 106, -46,},
{  36,-101, 154,-149,  69,  45,-131, 143, -78, -34, 125,-120,  24,  82,-112,  59,  26, -94, 100, -41, -27,  55, -49,  33, -15,  -7,  29, -42,  31,  -5, -21,  39, -44,  34,  -4, -34,  61, -62,  23,  39, -74,  55,  -2, -40,  51, -39,  15,   6, -23,  32, -23,   8,   2, -14,  32, -45,  27,  23, -72,  76, -12, -75, 103, -47,},
{ -20,  58, -90,  93, -55,  -5,  55, -79,  82, -55,  -9,  70, -89,  63,  -6, -51,  88, -88,  33,  58,-123, 121, -67, -12,  93,-132, 108, -37, -49, 103, -99,  46,  21, -71,  90, -75,  30,  23, -71,  88, -57,   2,  46, -65,  43,   4, -35,  36, -26,  18, -14,   8,  16, -47,  57, -29, -17,  60, -80,  47,  26, -78,  75, -31,},
{  21, -61,  91, -97,  76, -32, -14,  51, -77,  78, -42, -19,  75, -94,  67,  -2, -68, 102, -86,  34,  20, -59,  78, -77,  52,   0, -58,  89, -87,  51,   9, -68, 110,-120,  99, -49, -27, 100,-130, 104, -38, -37,  80, -74,  43,   3, -48,  65, -55,  28,  10, -49,  67, -50,  16,  21, -46,  53, -45,  17,  26, -58,  56, -23,},
{  -7,  21, -32,  35, -31,  16,   8, -31,  49, -55,  40,  -6, -34,  62, -65,  39,   6, -49,  60, -41,  15,   8, -22,  37, -60,  75, -54,   5,  52, -95,  99, -59,   0,  52, -85, 100, -93,  57,  -8, -40,  85,-106,  88, -38, -22,  75,-107,  94, -48, -11,  76,-113, 100, -50, -19,  79,-111, 105, -59, -16,  89,-129, 107, -40,},
{  11, -39,  77,-110, 119, -93,  36,  31, -95, 138,-136,  76,  12, -87, 125,-116,  79, -31, -20,  60, -81,  77, -46,  -1,  47, -74,  74, -55,  29,   7, -50,  81, -96,  94, -71,  35,   9, -55,  83, -81,  62, -32,  -1,  24, -43,  52, -44,  18,  14, -37,  47, -41,  18,   9, -32,  43, -43,  29,   1, -39,  64, -63,  42, -14,},
{   8, -27,  56, -92, 119,-120,  93, -50,   2,  49, -95, 115,-104,  63,  -7, -47,  91,-107,  86, -47,  10,  13, -31,  55, -67,  50, -19, -11,  43, -75,  90, -73,  38,  -1, -29,  50, -65,  71, -68,  50, -13, -34,  73, -86,  74, -43,  10,  17, -40,  55, -56,  42, -13, -31,  70, -79,  62, -31, -10,  56, -99, 119, -94,  35,},
{  -6,  20, -42,  71,-102, 121,-117,  93, -64,  29,  15, -54,  75, -69,  42,  -9, -23,  51, -69,  68, -54,  38, -21,   1,  26, -53,  62, -53,  36, -13, -17,  43, -61,  69, -74,  75, -66,  43, -12, -24,  62, -84,  81, -51,   8,  39, -76,  88, -86,  77, -51,   9,  36, -71,  85, -69,  30,  15, -61, 103,-132, 133, -94,  33,},
{  11, -31,  58, -88, 115,-126, 120,-110, 105, -97,  69, -17, -40,  84,-108, 108,-103,  88, -48,  -5,  50, -77,  91, -96,  91, -73,  41,  -4, -39,  81,-101,  92, -72,  57, -44,  26,   3, -35,  55, -59,  58, -53,  35,  -6, -15,  26, -35,  36, -27,  11,   6, -21,  30, -35,  29, -13,  -8,  27, -41,  51, -59,  59, -42,  15,},
{  -2,   6, -10,  14, -20,  23, -24,  24, -26,  31, -33,  28, -14,  -3,  18, -37,  66, -91,  92, -76,  59, -41,  19,   6, -35,  57, -66,  68, -64,  48, -17, -24,  67,-103, 124,-139, 151,-151, 140,-123, 101, -68,  19,  31, -67,  88, -88,  66, -34,   3,  26, -44,  51, -49,  39, -20,   0,  22, -52,  73, -76,  64, -37,  11,},
{   0,   2, -11,  23, -40,  56, -66,  72, -84, 106,-126, 130,-122, 107, -84,  52, -18,  -8,  28, -39,  36, -27,  15,   4, -30,  56, -75,  91,-106, 115,-111,  91, -71,  62, -61,  58, -53,  57, -66,  76, -78,  77, -76,  71, -59,  43, -23,   7,   1,  -6,   8,  -4, -11,  33, -53,  63, -72,  78, -70,  55, -43,  36, -24,   8,},
{   6, -16,  25, -35,  43, -49,  49, -48,  49, -51,  54, -54,  49, -40,  30, -15,  -6,  31, -48,  56, -69,  82, -77,  62, -42,  13,  18, -39,  55, -65,  62, -46,  36, -34,  29, -20,   6,  13, -38,  61, -86, 115,-131, 122, -98,  64, -21, -23,  63, -88,  98, -93,  68, -35,  -5,  49, -85, 105,-114, 120,-121, 104, -67,  23,},
{   4, -11,  19, -30,  40, -43,  40, -35,  31, -25,  18, -10,  -1,  14, -23,  33, -46,  64, -78,  83, -88, 100,-106,  99, -90,  84, -72,  56, -38,  22,  -9,  -1,   5,  -8,  12, -15,  14,  -9,   5,   0, -16,  36, -57,  79,-101, 120,-135, 140,-141, 139,-126, 102, -67,  31,   4, -34,  58, -70,  69, -55,  44, -41,  31, -12,},
{   3,  -7,   7,   4, -22,  34, -45,  57, -69,  85,-103, 119,-129, 127,-116, 102, -92,  85, -65,  33,  -5, -15,  37, -57,  80,-101, 116,-128, 134,-130, 116, -92,  72, -61,  53, -49,  46, -41,  28, -10,  -7,  22, -35,  44, -49,  48, -40,  28, -17,   8,   3,  -8,   7,  -6,  10, -16,  21, -21,  16, -16,  20, -18,   9,  -2,},
{  -1,   2,  -3,   5,  -7,   6,  -2,  -2,   5,  -6,   7, -10,  17, -22,  27, -33,  43, -56,  64, -64,  60, -59,  57, -49,  35, -22,  10,   0,  -7,  12, -18,  23, -31,  34, -35,  38, -42,  47, -52,  60, -71,  83, -95,  97, -89,  76, -57,  35,  -3, -34,  69,-104, 137,-161, 167,-155, 139,-123,  98, -72,  49, -34,  21,  -7,},
{   0,   1,  -4,   3,   2,  -7,  11, -16,  22, -31,  40, -46,  52, -61,  67, -70,  81, -97, 102,-100, 102,-105, 105,-101,  92, -83,  73, -64,  54, -44,  36, -23,   6,   6, -11,  15, -21,  30, -38,  44, -46,  49, -56,  66, -73,  77, -79,  83, -93, 103,-108, 106,-103, 102, -96,  82, -66,  52, -38,  24, -11,   4,   0,  -1,},
},

{
{   7,   9,  11,  13,  15,  18,  20,  23,  25,  28,  30,  34,  37,  39,  41,  44,  46,  48,  50,  51,  54,  56,  58,  59,  61,  65,  67,  68,  69,  70,  72,  73,  74,  76,  77,  78,  78,  79,  80,  80,  81,  82,  83,  82,  83,  83,  82,  82,  82,  81,  80,  80,  79,  78,  77,  76,  75,  73,  71,  70,  68,  67,  64,  62,},
{ -11, -16, -21, -25, -30, -36, -42, -46, -51, -56, -61, -66, -70, -75, -78, -80, -82, -83, -85, -87, -86, -86, -84, -83, -82, -77, -75, -72, -68, -64, -59, -54, -49, -44, -38, -31, -25, -19, -12,  -5,   1,   8,  17,  23,  32,  40,  46,  52,  58,  63,  69,  73,  77,  80,  84,  86,  89,  90,  91,  91,  90,  87,  84,  80,},
{  18,  28,  38,  47,  55,  63,  70,  76,  81,  85,  89,  91,  91,  89,  87,  83,  77,  72,  65,  56,  47,  37,  27,  16,   6,  -6, -17, -27, -36, -46, -56, -65, -72, -78, -84, -87, -89, -89, -88, -86, -83, -78, -72, -65, -54, -45, -36, -26, -15,  -4,   7,  18,  28,  37,  47,  57,  66,  74,  79,  82,  84,  83,  82,  78,},
{  21,  32,  45,  56,  65,  73,  79,  84,  86,  86,  84,  78,  69,  59,  48,  34,  20,   6,  -8, -23, -37, -51, -63, -72, -80, -86, -90, -90, -87, -82, -74, -64, -52, -38, -24,  -9,   6,  22,  37,  51,  63,  74,  82,  87,  89,  89,  87,  82,  75,  65,  52,  38,  23,   7,  -9, -26, -42, -56, -68, -78, -86, -90, -89, -85,},
{  30,  46,  62,  73,  82,  88,  91,  89,  83,  73,  60,  43,  24,   5, -15, -34, -51, -65, -76, -84, -90, -91, -87, -78, -66, -49, -31, -13,   7,  26,  44,  61,  73,  83,  88,  89,  85,  77,  66,  51,  35,  17,  -2, -21, -42, -60, -73, -83, -89, -89, -85, -78, -66, -52, -35, -14,   6,  26,  44,  59,  71,  79,  82,  81,},
{  33,  50,  66,  78,  85,  86,  81,  72,  58,  39,  17,  -6, -29, -50, -68, -79, -86, -87, -83, -74, -57, -35, -10,  14,  36,  59,  76,  86,  90,  88,  80,  67,  49,  27,   4, -20, -43, -62, -77, -87, -91, -88, -79, -64, -42, -15,  10,  35,  57,  74,  86,  91,  89,  81,  67,  46,  22,  -3, -28, -49, -66, -78, -83, -81,},
{  37,  56,  72,  82,  85,  81,  68,  48,  24,  -3, -30, -54, -72, -85, -88, -83, -71, -51, -26,   1,  27,  52,  72,  85,  90,  86,  72,  50,  24,  -4, -32, -58, -76, -88, -91, -85, -70, -47, -20,   8,  36,  61,  81,  93,  92,  82,  66,  43,  15, -16, -45, -68, -85, -93, -92, -79, -57, -31,  -3,  26,  52,  70,  79,  81,},
{  43,  65,  80,  87,  86,  72,  48,  19, -14, -44, -70, -86, -90, -81, -65, -42, -12,  18,  49,  73,  85,  87,  78,  60,  34,  -1, -33, -58, -77, -85, -84, -71, -48, -19,  12,  40,  65,  83,  89,  82,  64,  36,   2, -29, -60, -81, -90, -87, -72, -47, -15,  18,  49,  76,  92,  95,  84,  61,  30,  -4, -38, -66, -83, -88,},
{  45,  67,  80,  81,  71,  48,  19, -14, -44, -67, -81, -82, -70, -46, -16,  16,  47,  72,  85,  82,  69,  42,   8, -27, -59, -82, -89, -83, -62, -30,   7,  43,  71,  88,  88,  73,  47,  12, -25, -58, -82, -93, -88, -67, -28,  15,  51,  78,  93,  89,  71,  40,   2, -38, -70, -90, -95, -84, -57, -21,  20,  60,  86,  95,},
{  53,  78,  91,  87,  68,  34,  -9, -50, -81, -96, -89, -66, -31,  10,  47,  74,  88,  86,  66,  32, -11, -54, -84, -96, -88, -60, -20,  22,  60,  84,  91,  81,  53,  14, -27, -61, -84, -90, -75, -45,  -5,  34,  68,  86,  81,  60,  30,  -7, -43, -68, -79, -74, -51, -16,  24,  58,  78,  83,  70,  38,  -3, -43, -74, -91,},
{  59,  85,  93,  80,  49,   4, -44, -81,-100, -94, -63, -16,  36,  78,  97,  91,  63,  21, -27, -67, -88, -88, -67, -30,  13,  57,  83,  85,  66,  32,  -9, -49, -76, -86, -76, -44,   0,  44,  74,  85,  75,  46,   4, -41, -72, -80, -72, -45,  -8,  32,  64,  81,  76,  49,  10, -34, -68, -85, -80, -54, -13,  33,  71,  93,},
{ -61, -84, -85, -63, -21,  28,  70,  92,  86,  55,   4, -48, -83, -91, -73, -37,  13,  58,  92,  95,  64,  14, -37, -73, -88, -80, -44,   5,  51,  80,  87,  69,  28, -22, -65, -88, -83, -51,  -8,  35,  69,  83,  72,  42,  -8, -56, -81, -82, -58, -17,  28,  65,  83,  77,  49,   3, -47, -85, -92, -70, -27,  25,  70,  97,},
{ -64, -85, -81, -48,   3,  53,  86,  85,  55,   8, -43, -80, -85, -58, -14,  35,  72,  85,  69,  29, -22, -67, -88, -77, -41,  15,  67,  90,  82,  44,  -9, -60, -86, -83, -50,  -1,  51,  86,  90,  63,  15, -39, -81, -95, -71, -20,  33,  75,  88,  72,  32, -20, -62, -85, -78, -44,   9,  63,  91,  86,  49, -12, -67, -94,},
{ -79, -98, -81, -30,  32,  83, 103,  77,  21, -42, -87, -96, -66,  -7,  54,  93,  94,  56,  -8, -68, -94, -78, -30,  25,  65,  83,  66,  22, -30, -69, -84, -65, -18,  39,  80,  86,  57,   2, -51, -80, -78, -48,   3,  52,  84,  80,  42, -15, -63, -84, -72, -31,  20,  64,  81,  64,  21, -35, -75, -82, -56,  -1,  53,  83,},
{ -72, -89, -66, -10,  50,  85,  79,  38, -20, -68, -84, -58,  -5,  52,  83,  72,  28, -30, -73, -79, -51,   5,  56,  80,  69,  21, -36, -74, -80, -49,   4,  61,  87,  75,  28, -32, -78, -89, -58,  -1,  60,  91,  79,  30, -39, -87, -92, -53,   8,  65,  95,  80,  30, -36, -86, -95, -57,  12,  72,  96,  77,  18, -52,-100,},
{ -63, -73, -49,   1,  47,  72,  61,  16, -37, -70, -65, -25,  32,  73,  75,  37, -23, -72, -87, -54,  12,  72,  91,  61,   0, -67, -98, -71,  -8,  58,  94,  84,  28, -42, -90, -90, -43,  30,  84,  95,  55, -18, -84,-104, -55,  23,  79,  95,  61,  -4, -65, -94, -75, -16,  51,  89,  77,  25, -39, -85, -80, -25,  40,  88,},
{ -90,-101, -52,  29,  92, 102,  50, -31, -92, -99, -43,  42, 101,  98,  36, -51,-107, -98, -25,  59, 100,  82,  18, -53, -88, -74, -20,  47,  88,  73,  17, -52, -85, -70, -11,  55,  86,  66,  10, -53, -86, -68,  -6,  64,  84,  49,  -2, -55, -75, -48,   6,  55,  73,  46, -10, -55, -64, -36,  13,  53,  58,  24, -22, -56,},
{ -75, -78, -29,  33,  75,  69,  12, -50, -76, -52,   7,  64,  76,  40, -25, -77, -75, -25,  49,  89,  62,  -8, -69, -82, -40,  35,  85,  74,  15, -52, -90, -66,   6,  71,  87,  46, -25, -84, -79, -26,  43,  90,  75,   9, -78,-109, -54,  32,  96,  94,  31, -54,-100, -76,  -5,  64,  92,  59, -12, -75, -92, -43,  35,  92,},
{-109,-100, -18,  76, 112,  63, -37,-102, -86,  -7,  77, 102,  48, -39,-101, -77,   1,  76,  93,  39, -40, -89, -67,   6,  64,  79,  42, -27, -79, -71,  -9,  58,  81,  46, -24, -73, -69, -14,  50,  77,  51, -15, -71, -78, -17,  58,  80,  45, -22, -72, -72, -19,  47,  79,  52, -12, -67, -73, -23,  46,  81,  52, -19, -77,},
{ -94, -83,  -3,  82,  97,  32, -64, -99, -49,  38,  97,  71, -20, -95, -85,   1,  79,  91,  29, -59, -93, -45,  38,  88,  61, -20, -82, -71,   1,  67,  81,  28, -46, -81, -49,  19,  68,  64,  10, -49, -68, -32,  31,  67,  49,  -5, -58, -70, -29,  44,  85,  59, -13, -82, -88, -20,  68,  98,  45, -41, -96, -72,  16,  93,},
{ -62, -47,  13,  66,  57,  -3, -68, -66,   2,  65,  69,   8, -59, -77, -21,  62,  87,  32, -66,-100, -33,  68, 103,  48, -52,-115, -71,  41, 112,  85, -18,-104, -88,  -4,  85,  95,  17, -73, -91, -29,  55,  92,  49, -44, -88, -48,  20,  71,  62,  -3, -66, -74, -15,  63,  79,  27, -45, -78, -45,  26,  78,  63,  -8, -77,},
{ -42, -31,  12,  48,  33, -19, -50, -31,  24,  53,  27, -28, -50, -22,  29,  47,  17, -24, -55, -31,  38,  70,  32, -36, -77, -44,  34,  82,  54, -29, -86, -63,  22,  93,  75, -19, -94, -79,   7,  87,  86,   2, -89, -93,  10, 110,  88, -18,-110, -96,  10, 109, 103, -11,-110,-101,  10, 107,  89,  -5, -94, -96,  -1, 104,},
{-112, -70,  44, 112,  63, -49,-115, -47,  67, 106,  27, -76, -91, -14,  75,  78,  15, -64, -78,  -8,  66,  68,  -1, -67, -62,   9,  78,  58, -23, -81, -49,  36,  82,  38, -49, -83, -27,  62,  82,  19, -62, -85, -19,  76,  79,  -4, -73, -67,   9,  75,  65, -16, -85, -60,  31,  88,  45, -46, -81, -28,  58,  77,   8, -74,},
{ -88, -46,  46,  85,  30, -55, -83, -11,  68,  74,  -4, -78, -62,  22,  81,  43, -37, -80, -22,  63,  65,  -9, -65, -48,  30,  77,  34, -50, -77, -17,  61,  73,  -3, -76, -59,  28,  79,  39, -39, -74, -33,  49,  75,  18, -73, -82,  16, 100,  70, -47,-110, -50,  71, 110,  16, -90, -86,  15, 101,  69, -53,-115, -28, 104,},
{ -78, -33,  50,  72,  17, -67, -76,  12,  93,  60, -59, -99, -15,  96,  78, -35,-104, -50,  76, 105,   0,-108, -80,  48, 114,  34, -81, -89,  13,  87,  64, -35, -92, -38,  63,  82,   6, -77, -63,  23,  79,  40, -50, -70,  -1,  63,  49, -31, -68, -15,  56,  59, -12, -73, -40,  50,  68,   6, -64, -60,  27,  88,  27, -78,},
{  65,  20, -50, -57,   7,  74,  38, -50, -70,   5,  76,  46, -44, -87,  -2,  83,  54, -42, -92, -17,  89,  82, -36,-110, -38,  93,  86, -35, -98, -38,  71,  90, -16, -95, -53,  59,  90,   4, -83, -66,  35,  96,  29, -83, -66,  31,  83,  22, -72, -63,  26,  78,  32, -67, -69,  29,  81,  29, -64, -80,  20,  97,  31, -83,},
{ 144,  36,-123,-107,  67, 138,   3,-126, -83,  83, 128, -17,-134, -45,  98,  84, -28,-103, -26,  86,  56, -49, -85,   3,  88,  36, -61, -58,  24,  66,  12, -56, -39,  27,  60,   9, -55, -46,  25,  66,  17, -60, -61,  41,  64, -10, -42, -27,  27,  43,   2, -38, -32,  20,  46,   6, -38, -35,  20,  49,   1, -50, -20,  43,},
{ -79, -11,  72,  46, -57, -61,  40,  66,  -8, -74, -14,  65,  39, -55, -51,  26,  58,  12, -66, -36,  55,  63, -27, -80, -10,  82,  42, -57, -74,  27,  76,  20, -69, -58,  40,  82,   7, -87, -53,  55,  88,  -1, -92, -55,  80,  92, -37,-109, -28,  93,  75, -42,-101,  -9,  95,  53, -68,-101,  35, 117,  12,-110, -54,  92,},
{ 115,   0,-113, -37,  98,  59, -79, -74,  52,  86, -28, -92,   0,  97,  26, -74, -61,  48,  80, -28, -76,  -8,  76,  42, -64, -65,  42,  74, -12, -77, -21,  70,  46, -47, -68,   9,  81,  33, -70, -69,  36,  83,   6, -86, -30,  81,  46, -54, -69,  23,  80,   9, -79, -40,  56,  62, -23, -88,  -4,  99,  34, -95, -64,  88,},
{  84,  -6, -88, -15,  83,  32, -79, -45,  73,  60, -69, -64,  55,  71, -37, -74,   6,  80,  20, -76, -39,  67,  61, -56, -82,  38,  92, -16, -97, -17,  95,  53, -82, -82,  62,  93, -30,-106,  -3, 104,  39, -81, -69,  50,  92, -27,-101,   1,  94,  21, -72, -42,  45,  61, -23, -67,  -1,  73,  24, -72, -42,  63,  55, -61,},
{ -81,  20,  88, -19, -81,   9,  84,  -2, -89,  -9,  94,  12, -87, -16,  69,  29, -55, -52,  53,  59, -42, -78,  38,  91, -23,-107,  10, 108,  19,-104, -45,  97,  62, -80, -86,  63, 103, -45,-102,  14, 102,  15, -99, -33,  90,  51, -77, -65,  56,  73, -34, -72,  16,  73,   0, -75,  -8,  71,  15, -56, -27,  43,  39, -43,},
{  76, -22, -75,  17,  69,  -9, -75,  11,  79, -15, -71,  13,  70, -18, -63,  13,  58,  -4, -67,  -3,  78,  11, -78, -28,  71,  37, -66, -39,  68,  41, -67, -49,  65,  59, -66, -62,  54,  70, -37, -77,  22,  80,  -6, -94,  14, 101,  -8,-105, -15, 105,  38,-111, -50, 109,  61,-100, -77,  96,  80, -84, -79,  72,  75, -72,},
{  95, -35, -91,  37,  92, -50, -96,  75,  86, -81, -79,  80,  67, -74, -59,  59,  69, -52, -68,  48,  66, -47, -70,  45,  79, -66, -63,  68,  58, -66, -55,  61,  48, -54, -45,  53,  43, -44, -48,  35,  56, -31, -57,  30,  51, -30, -52,  27,  65, -24, -70,  15,  88, -22, -98,  26, 108, -27,-115,  44,  98, -56, -75,  64,},
{ -74,  36,  69, -49, -61,  63,  53, -77, -38,  84,  21, -95,  -1, 102, -11,-103,  12, 105, -22,-102,  41,  92, -54, -84,  59,  73, -61, -57,  49,  53, -42, -48,  40,  48, -36, -49,  36,  54, -39, -60,  42,  56, -49, -51,  74,  42, -94, -29,  95,  18, -92, -12,  86,   6, -79, -15,  90,  19,-112,   7, 111, -37, -99,  72,},
{  70, -43, -71,  78,  40, -91,  -2,  78, -16, -79,  45,  65, -64, -56,  80,  49, -90, -32,  80,  24, -69, -22,  78,  10, -95,  27,  77, -39, -74,  36,  86, -54, -71,  54,  68, -57, -63,  67,  44, -64, -39,  66,  37, -79, -15,  98, -28, -90,  40,  82, -42, -81,  44,  80, -62, -69,  86,  54,-109, -18, 100, -13, -81,  49,},
{ -93,  68,  79,-103, -40, 114, -10, -93,  47,  68, -76, -37,  88,   0, -85,  32,  76, -48, -63,  54,  51, -66, -26,  72,  -8, -55,  18,  45, -23, -53,  40,  49, -48, -45,  57,  45, -71, -35,  74,  31, -80, -20,  83,   0, -89,  38,  71, -51, -65,  64,  58, -80, -41,  96,  10,-109,  43,  93, -96, -38, 106, -17, -88,  60,},
{ -10,   1,  20, -11, -31,  38,  22, -58,   6,  59, -43, -39,  73,   4, -83,  35,  73, -53, -72,  73,  61,-108, -13, 117, -41, -93,  80,  45, -91,   1,  86, -32, -74,  50,  62, -70, -40,  82,  20, -85, -11,  92, -11, -97,  65,  75,-106, -28, 110, -14,-100,  54,  72, -82, -37,  99, -14, -93,  71,  50, -89,   5,  67, -41,},
{  83, -72, -49, 106, -21, -86,  71,  43, -91,  -1,  96, -54, -66,  90,  23, -97,  13,  92, -54, -75,  97,  36,-117,   9, 112, -63, -73,  97,  35,-114,   6, 117, -54, -94,  87,  58,-101, -17,  94,  -3, -88,  16,  84, -42, -54,  62,  16, -61,   5,  51, -22, -29,  30,  -2, -15,  18,  -6, -18,  24,   9, -35,  11,  25, -19,},
{ -79,  71,  42, -97,  18,  81, -66, -30,  81, -25, -59,  66,  14, -86,  40,  72, -84, -27,  93, -17, -79,  64,  37, -84,  16,  75, -67, -28,  90, -21, -81,  64,  32, -69,  10,  61, -47, -43,  72,  21, -84,  12,  85, -64, -41,  83,   1, -89,  34,  87, -83, -45, 109, -20, -97,  91,  26,-108,  55,  58, -81,   3,  60, -37,},
{  48, -47, -13,  57, -34, -29,  53,  -8, -51,  44,  30, -70,  12,  59, -40, -39,  65,  -4, -51,  32,  21, -47,  17,  47, -61,  -1,  71, -58, -42,  93, -18, -85,  71,  45, -99,  10,  91, -69, -51,  99,  -2, -90,  46,  58, -86,  -4,  97, -60, -66, 108,   6,-128,  89,  76,-151,  37, 110,-112,  -3,  87, -61, -22,  68, -36,},
{ -97, 102,  24,-120,  70,  53,-111,  47,  67, -99,  13,  86, -85, -14, 102, -69, -54, 108, -18, -96,  81,  35,-102,  45,  63,-104,  30,  82, -90, -22,  98, -30, -72,  62,  31, -75,  16,  61, -46, -35,  65, -13, -45,  50,  -4, -49,  48,  10, -50,  25,  38, -60,   4,  65, -58, -17,  70, -49, -15,  65, -52, -20,  74, -41,},
{  29, -37,   5,  37, -32, -20,  55, -30, -36,  65, -23, -47,  62,   7, -79,  54,  46,-103,  56,  58,-129,  76,  42,-105,  62,  26, -75,  49,  37, -79,  20,  69, -78, -17,  99, -51, -78, 124, -18,-117, 108,  36,-126,  68,  43,-101,  61,  36, -83,  37,  58, -87,   8,  79, -69, -14,  72, -50, -17,  61, -41, -16,  50, -26,},
{ -82, 105, -15, -97, 123, -32, -93, 129, -37, -89, 125, -40, -82, 133, -49, -96, 137, -26, -96, 100, -10, -80,  86,   3, -85,  70,  16, -82,  55,  34, -85,  45,  35, -74,  29,  54, -76,  10,  67, -61, -23,  79, -47, -23,  45, -15, -19,  27,  -2, -29,  30,  -2, -25,  32, -15, -19,  37, -21, -13,  33, -23,  -7,  27, -15,},
{ -42,  55, -12, -51,  72, -27, -39,  62, -31, -19,  43, -26, -14,  51, -44, -15,  61, -46,   1,  28, -37,  20,  11, -24,  22, -17,  -3,  32, -28, -13,  35,  -8, -29,  23,  17, -39,  16,  36, -50,  -9,  64, -42, -38,  95, -70, -31, 126,-108, -24, 142,-119, -22, 138,-129,  13, 120,-151,  49,  91,-141,  67,  57,-119,  62,},
{  79,-116,  58,  45,-114,  92,  -9, -72,  89, -30, -52,  82, -40, -38,  89, -64, -19,  71, -44, -14,  49, -43, -13,  70, -54, -18,  74, -66,  -5,  73, -76,   1,  79, -87,   4,  96,-113,  24,  89,-115,  27,  87,-115,  37,  61, -96,  49,  35, -80,  57,  12, -68,  62,  -9, -41,  62, -46,  -5,  53, -63,  26,  32, -60,  31,},
{ -28,  44, -21, -23,  52, -39,  -7,  49, -51,  15,  33, -57,  36,  16, -62,  65,  -8, -62,  78, -21, -66, 105, -64, -35, 115,-113,  26,  91,-134,  64,  56,-127,  95,  18,-117, 118, -14,-110, 132, -26, -98, 117, -32, -63,  90, -39, -33,  70, -51,  -8,  56, -54,  14,  26, -42,  32, -12,  -9,  30, -34,  12,  22, -37,  20,},
{ -69, 110, -73, -13,  93,-111,  59,  30, -96,  87, -17, -61,  96, -56, -37, 115,-107,  17,  77,-117,  82,  18,-107, 114, -41, -51, 114,-109,  23,  86,-124,  57,  50,-106,  73,   9, -75,  85, -38, -34,  71, -46,  -9,  42, -30,   7,  -4,   3,   6, -13,  13,  -1, -17,  25, -18,   4,   8, -18,  20, -11,  -8,  24, -24,  10,},
{ -31,  53, -41,   8,  22, -31,  22,  -1, -20,  24,  -8,  -9,  24, -30,  21,   7, -41,  56, -36,  -8,  56, -70,  33,  26, -63,  71, -51, -13,  86, -93,  23,  64,-108,  69,  23, -89,  90, -39, -30,  75, -74,  32,  27, -81,  99, -38, -74, 131, -90, -18, 112,-120,  38,  64,-115,  99, -29, -59, 108, -89,   9,  88,-125,  59,},
{  59,-103,  87, -21, -55,  90, -73,  16,  54, -92,  65,   5, -67,  88, -63,   5,  59, -88,  57,   5, -60,  82, -53,  -9,  56, -71,  58, -15, -35,  61, -53,  24,   7, -42,  58, -34, -19,  72, -85,  39,  41, -94,  77, -12, -57, 100, -86,  18,  61,-103,  78,  -7, -59,  92, -82,  35,  22, -71,  94, -64, -17,  99,-118,  53,},
{  69,-120, 104, -43, -27,  77, -96,  71,  -2, -68,  96, -76,  15,  57,-103,  94, -23, -66, 117,-107,  39,  41, -87,  91, -55,  -3,  60, -92,  74, -20, -36,  77, -86,  45,  26, -78,  82, -57,  11,  45, -78,  67, -20, -37,  79, -83,  41,  14, -51,  58, -33, -10,  44, -57,  46, -13, -24,  53, -64,  37,  20, -67,  70, -30,},
{ -45,  84, -83,  47,  11, -66, 102,-105,  61,  15, -88, 123,-102,  37,  42,-101, 115, -77,   8,  58, -98, 100, -64,   8,  52, -96,  99, -53, -19,  80,-100,  76, -26, -30,  72, -82,  50,   3, -53,  82, -72,  17,  52, -87,  75, -33, -15,  53, -63,  46, -10, -26,  45, -44,  29,   0, -33,  53, -49,  17,  22, -46,  43, -17,},
{  29, -61,  68, -48,  10,  34, -69,  82, -63,  15,  39, -77,  84, -69,  39,   6, -54,  75, -72,  55, -10, -44,  71, -70,  48, -15, -23,  59, -79,  65, -23, -34,  81, -99,  77, -15, -58, 106,-106,  58,  14, -78, 105, -89,  43,  20, -79, 106, -88,  27,  48, -96, 101, -72,  22,  35, -74,  84, -64,  14,  42, -82,  79, -32,},
{ -42,  80, -86,  69, -43,   4,  44, -88, 106, -87,  38,  28, -90, 118,-106,  61,   4, -58,  74, -68,  47,  -6, -34,  59, -75,  74, -46,   0,  44, -78,  86, -68,  28,  23, -70,  95, -85,  49,  -2, -53,  95, -93,  51,  -1, -31,  59, -87,  80, -37, -19,  68, -90,  79, -45,  -9,  62, -79,  64, -40,   7,  37, -76,  73, -30,},
{  41, -86, 108,-109,  88, -53,   2,  56,-100, 110, -89,  48,   6, -56,  86, -91,  68, -31,   1,  22, -45,  56, -47,  30,  -6, -24,  56, -74,  68, -44,   8,  38, -77,  85, -67,  33,  12, -56,  82, -81,  59, -27, -10,  46, -80,  90, -56,  -3,  59, -92,  94, -70,  26,  20, -63,  94, -90,  55, -14, -29,  73, -98,  82, -31,},
{ -47,  97,-119, 122,-110,  79, -31, -20,  62, -91, 105,-101,  75, -35,  -3,  37, -62,  67, -60,  54, -31,  -5,  36, -56,  65, -62,  45, -15, -21,  54, -74,  74, -54,  16,  23, -49,  56, -52,  40, -16, -15,  42, -58,  58, -45,  14,  31, -70,  88, -83,  53, -11, -29,  62, -86,  94, -76,  40,   6, -61, 105,-121,  94, -34,},
{ -35,  72, -90, 100,-108, 109,-103,  82, -42,  -7,  51, -86, 101,-100,  93, -76,  38,   3, -43,  86,-113, 113, -96,  63, -14, -37,  76, -99, 104, -89,  56, -13, -32,  63, -74,  69, -47,  25,  -5, -21,  45, -52,  42, -27,   7,  18, -40,  51, -51,  49, -36,  11,  19, -46,  58, -51,  33, -11, -14,  44, -66,  72, -57,  21,},
{  10, -18,  22, -28,  35, -39,  43, -50,  49, -39,  29, -10, -15,  36, -55,  71, -75,  66, -51,  25,  15, -52,  78, -90,  86, -75,  56, -19, -31,  78,-104, 119,-125, 102, -58,   8,  41, -80, 104,-104,  84, -53,   6,  39, -62,  72, -80,  77, -59,  29,  13, -57,  87, -98,  92, -71,  38,   3, -41,  70, -89,  92, -67,  23,},
{ -20,  45, -64,  80, -88,  90, -90,  88, -82,  72, -60,  42, -15,  -9,  26, -42,  59, -71,  73, -61,  36,  -8, -14,  34, -56,  75, -87,  90, -85,  71, -47,  15,  20, -50,  67, -72,  62, -42,  14,  19, -55,  88,-113, 119,-114,  99, -67,  29,   8, -44,  67, -77,  72, -56,  30,   2, -32,  56, -75,  87, -87,  72, -42,  12,},
{   5,  -7,   3,   7, -18,  30, -43,  57, -68,  76, -85,  95,-104, 110,-114, 122,-130, 130,-124, 109, -88,  73, -62,  51, -38,  27, -23,  23, -24,  30, -39,  49, -60,  71, -78,  73, -67,  69, -73,  66, -56,  43, -32,  22, -12,   8,  -6,   9, -22,  38, -50,  59, -63,  66, -68,  71, -72,  67, -60,  49, -33,  18,  -7,   2,},
{  18, -39,  57, -74,  86, -94, 104,-115, 115,-104,  91, -79,  63, -42,  19,   7, -36,  59, -72,  81, -86,  86, -81,  66, -44,  23,  -4, -15,  33, -48,  54, -53,  54, -46,  29, -15,  -1,  25, -53,  74, -86,  90, -85,  69, -46,  17,  17, -49,  73, -85,  85, -77,  64, -47,  21,  12, -38,  54, -70,  84, -88,  76, -49,  16,},
{ -10,  18, -21,  27, -35,  34, -33,  39, -47,  52, -57,  58, -51,  40, -31,  21,  -6,  -9,  32, -58,  77, -99, 121,-131, 129,-127, 130,-125, 107, -86,  65, -40,  12,  12, -32,  50, -61,  69, -77,  76, -65,  44, -16, -11,  31, -50,  71, -87,  90, -90,  86, -73,  53, -34,  13,  11, -30,  41, -52,  63, -61,  50, -34,  12,},
{   0,  -2,   5, -11,  19, -23,  26, -33,  38, -43,  53, -65,  74, -79,  82, -83,  82, -84,  84, -74,  57, -40,  20,   3, -23,  40, -59,  79, -91,  99,-103, 113,-130, 136,-135, 130,-117,  98, -78,  54, -25,  -3,  23, -39,  48, -51,  48, -40,  30, -16,   0,  17, -28,  35, -42,  50, -56,  57, -56,  53, -46,  38, -25,   8,},
{   4,  -7,  10, -13,  13, -15,  20, -24,  26, -27,  28, -26,  23, -21,  21, -24,  27, -28,  27, -32,  38, -42,  45, -48,  44, -41,  42, -43,  43, -45,  47, -51,  56, -56,  55, -55,  56, -55,  52, -46,  36, -23,   7,   9, -20,  29, -46,  67, -84,  97,-108, 117,-124, 129,-134, 139,-140, 135,-128, 118, -97,  69, -42,  14,},
{   6, -11,  13, -18,  20, -20,  22, -25,  26, -25,  22, -21,  21, -20,  19, -20,  21, -24,  29, -33,  34, -33,  40, -50,  51, -45,  43, -44,  40, -31,  18,  -1, -18,  33, -41,  50, -62,  80,-100, 112,-122, 130,-133, 133,-133, 135,-132, 125,-112,  95, -75,  51, -26,   5,  13, -32,  49, -61,  68, -73,  74, -68,  51, -19,},
},

{
{  24,  27,  29,  31,  33,  35,  37,  39,  41,  43,  45,  46,  48,  49,  52,  53,  53,  55,  57,  59,  59,  60,  61,  63,  65,  66,  66,  67,  69,  70,  71,  72,  73,  72,  72,  74,  75,  75,  75,  76,  76,  76,  76,  75,  75,  75,  75,  75,  75,  75,  76,  76,  75,  74,  73,  72,  71,  70,  70,  70,  69,  68,  67,  65,},
{ -52, -58, -63, -67, -71, -74, -78, -80, -83, -86, -88, -89, -90, -90, -90, -89, -88, -86, -85, -84, -80, -77, -74, -70, -64, -58, -51, -45, -40, -35, -27, -20, -13,  -6,   2,   9,  15,  20,  24,  29,  34,  37,  41,  44,  48,  52,  54,  56,  59,  61,  64,  66,  68,  70,  72,  73,  74,  73,  72,  70,  69,  68,  66,  64,},
{  55,  60,  64,  67,  70,  72,  74,  74,  74,  72,  70,  67,  62,  56,  49,  42,  33,  24,  15,   6,  -4, -16, -27, -38, -50, -61, -72, -80, -86, -91, -95, -97, -98, -98, -96, -92, -87, -80, -73, -66, -58, -48, -38, -29, -20, -12,  -1,   9,  17,  26,  34,  41,  50,  56,  62,  68,  72,  76,  80,  82,  83,  85,  83,  80,},
{  71,  79,  83,  87,  88,  87,  84,  78,  71,  60,  47,  33,  19,   4, -10, -24, -40, -52, -62, -70, -78, -86, -91, -92, -90, -85, -77, -69, -57, -44, -31, -14,   1,  16,  31,  44,  58,  68,  76,  82,  87,  89,  87,  83,  78,  71,  64,  55,  44,  31,  19,   8,  -5, -18, -31, -43, -54, -63, -69, -75, -78, -79, -78, -75,},
{  73,  83,  87,  86,  82,  75,  63,  48,  33,  16,  -2, -20, -36, -50, -63, -73, -82, -86, -84, -80, -74, -64, -51, -34, -15,   6,  28,  47,  63,  75,  80,  87,  88,  84,  74,  59,  46,  30,  12,  -7, -25, -41, -56, -69, -79, -85, -87, -87, -86, -80, -70, -58, -45, -31, -13,   7,  26,  40,  57,  74,  86,  92,  91,  86,},
{  72,  83,  85,  80,  71,  59,  42,  22,  -1, -26, -51, -69, -82, -89, -91, -90, -82, -66, -47, -26,  -4,  21,  44,  69,  89,  97,  96,  89,  76,  57,  33,   8, -19, -44, -64, -80, -88, -91, -89, -80, -65, -47, -26,  -4,  18,  38,  55,  70,  80,  86,  87,  82,  71,  58,  40,  21,   2, -18, -34, -48, -61, -68, -71, -70,},
{  75,  80,  78,  70,  53,  32,   8, -19, -47, -72, -90, -99, -98, -87, -66, -40, -11,  21,  49,  70,  85,  95, 100,  89,  66,  36,   0, -33, -59, -78, -87, -91, -85, -68, -46, -23,   1,  28,  53,  72,  84,  87,  84,  75,  57,  34,   7, -19, -43, -62, -74, -79, -80, -75, -65, -50, -31, -11,  12,  35,  54,  68,  74,  73,},
{  74,  79,  73,  59,  38,  13, -15, -43, -63, -76, -83, -78, -63, -40, -10,  21,  46,  70,  82,  83,  76,  56,  27,  -7, -41, -67, -85, -89, -79, -58, -31,  -1,  34,  65,  87,  96,  88,  70,  44,  15, -20, -51, -75, -89, -93, -87, -72, -49, -19,  12,  44,  66,  84,  95,  96,  83,  62,  38,   7, -25, -52, -76, -90, -92,},
{  64,  70,  62,  42,  15, -14, -43, -65, -78, -77, -69, -50, -22,  11,  42,  63,  74,  77,  68,  48,  22, -12, -45, -74, -94, -92, -69, -31,  12,  45,  69,  84,  87,  75,  44,   2, -35, -66, -85, -89, -78, -60, -31,   3,  39,  71,  91,  97,  87,  68,  39,   3, -35, -69, -93,-104,-101, -80, -42,  -3,  39,  71,  88,  94,},
{  87,  90,  73,  42,   3, -33, -66, -88, -92, -78, -50, -15,  20,  53,  76,  88,  83,  58,  24, -14, -55, -87, -95, -84, -52,  -8,  40,  78,  92,  85,  60,  25, -19, -57, -81, -86, -74, -45, -10,  26,  56,  74,  80,  72,  51,  19, -17, -48, -72, -81, -74, -55, -26,   8,  44,  75,  91,  89,  73,  39,  -6, -55, -90,-102,},
{ -89, -89, -68, -30,  20,  60,  91, 101,  88,  55,   8, -41, -76, -96, -90, -61, -18,  29,  67,  87,  86,  67,  28, -23, -69, -91, -89, -61, -14,  33,  65,  82,  80,  61,  28, -20, -60, -83, -85, -67, -33,   6,  46,  77,  88,  78,  44,   2, -39, -66, -77, -75, -53, -19,  21,  56,  73,  82,  72,  42,   3, -41, -71, -83,},
{ -79, -79, -61, -22,  36,  84, 105,  90,  50,  -4, -53, -86, -94, -76, -37,  16,  67,  94,  94,  65,  19, -31, -79,-105, -98, -44,  32,  83,  99,  85,  47,  -4, -61,-106,-108, -66,  -7,  52,  91,  97,  76,  35, -19, -67, -91, -84, -57, -11,  38,  68,  71,  56,  26,  -8, -32, -48, -49, -34, -12,   8,  20,  16,  17,  21,},
{ -43, -39, -23,   9,  47,  65,  49,  13, -21, -44, -44, -31, -12,   9,  24,  45,  51,  30,  -6, -32, -46, -50, -33,  -8,  28,  60,  79,  59,  -1, -51, -79, -86, -62, -11,  48,  81,  92,  76,  32, -24, -73,-102,-102, -71,  -6,  63, 112, 124,  96,  40, -26, -86,-121,-117, -77,  -7,  65, 108, 117,  89,  31, -31, -80,-102,},
{-111, -84, -27,  31,  81,  92,  79,  46,  -5, -56, -85, -84, -53,  -6,  42,  86, 101,  77,  24, -38, -94,-111, -70,   5,  60,  78,  70,  29, -26, -65, -70, -43,   6,  57,  77,  52,   8, -33, -60, -64, -41,  -5,  36,  67,  75,  48,  -5, -56, -86, -80, -42,   9,  55,  84,  85,  51,   2, -58,-106,-100, -53,  16,  72,  95,},
{-111, -90, -20,  59, 109, 110,  62, -18, -81,-100, -75, -25,  33,  81,  92,  60,   3, -51, -82, -80, -43,  18,  71,  86,  57,   7, -55, -85, -70, -32,  23,  74,  90,  63,  -7, -72, -86, -61, -18,  30,  67,  72,  54,  11, -43, -77, -74, -39,  17,  63,  79,  59,  16, -42, -76, -71, -33,  21,  69,  86,  60,   6, -53, -86,},
{-102, -64,   6,  67, 100,  85,  21, -48, -85, -79, -35,  18,  58,  79,  61,   5, -57, -89, -69, -12,  53,  88,  66,  21, -43, -87, -78, -23,  45,  84,  81,  31, -34, -80, -81, -37,  21,  66,  73,  42,  -5, -49, -70, -52,  -6,  42,  78,  64,  19, -32, -80, -85, -44,  18,  76,  98,  70,  10, -68,-118, -97, -20,  62, 109,},
{ -63, -42,   5,  45,  66,  47,  -1, -45, -60, -48,  -2,  41,  64,  54,  14, -35, -78, -73, -20,  43,  91,  83,  16, -67,-109, -68,  26,  99,  99,  42, -37,-102, -96, -40,  37,  96,  91,  36, -40, -88, -83, -32,  37,  87,  81,  34, -29, -78, -86, -43,  23,  76,  90,  52, -13, -71, -96, -59,  18,  85, 103,  38, -42, -89,},
{ -81, -48,  24,  80,  99,  49, -51,-116,-102, -10,  94, 119,  72, -17, -99,-107, -61,   8,  85, 110,  69, -13, -98, -96, -29,  37, 106,  85, -25, -91, -93, -26,  74,  94,  37, -31, -67, -54,  -9,  11,  34,  48,  34,  14, -38, -74, -44,   4,  65,  78,  23, -34, -69, -62,   5,  58,  60,  21, -33, -50, -29,  -5,  17,  36,},
{  74,  -6, -33, -43,  -5,  25, -19,  -8,  19,  41,  48, -48, -94, -51,  37, 131,  83, -62,-120, -81,  34, 133,  52, -50, -85, -48,  82, 105, -29, -84, -69,   8, 113,  56, -53, -72, -48,  45,  92,   0, -50, -50, -18,  69,  40, -34, -36, -17,  49,  66, -28, -69, -58,   2,  99,  94, -26, -88, -80,   0, 104,  64, -28, -64,},
{  50,  37, -18, -60, -56, -15,  42,  71,  50, -19, -74, -65, -19,  46,  83,  50, -16, -60, -77, -31,  47,  79,  53, -12, -74, -80, -15,  84, 110,  40, -75,-133, -72,  67, 137,  76, -28,-111,-103, -16,  61,  93,  65,  -2, -47, -66, -54,  -6,  43,  61,  48,  -2, -55, -77, -42,  31,  86,  76,   6, -68, -93, -51,  30,  89,},
{  47,  35, -24, -71, -66,  -6,  83, 100,  18, -83,-121, -41,  76, 127,  66, -60,-120, -60,  30,  86,  71,  -6, -60, -58, -13,  38,  61,  33, -22, -61, -54,  -3,  46,  85,  40, -62, -85, -58,  15,  93,  86,  10, -66,-103, -49,  42,  99,  81, -13, -91, -84, -26,  41,  88,  60,  -6, -57, -72, -22,  42,  58,  43, -16, -61,},
{  -2,  -3,  12,  11, -19, -39,  -4,  55,  57,  -6, -72, -74, -15,  62,  93,  32, -73, -94, -17,  76, 107,  15,-107,-104,  26, 115,  71, -56,-109, -46,  69, 111,  18,-110, -95,  51, 123,  66, -58,-127, -77,  47, 121,  81, -40,-111, -53,  40,  74,  31, -26, -41, -14,  -3,  -3,   1,  14,  21,  11, -13, -28, -12,  11,  19,},
{ 171,  36,-118,-153, -51,  94, 139,  50, -85,-126, -41,  59,  92,  46, -37, -63, -24,  10,  30,  20, -11,  -5,  -1, -12,  -3,  12,  19,   1, -36, -26,  10,  35,  45,  -1, -57, -43,  11,  59,  67,  -7, -75, -86, -28,  76, 118,  45, -64,-120, -54,  55,  87,  54, -20, -77, -54,   6,  57,  57,   4, -48, -55, -24,  17,  55,},
{  81,   4, -65, -70, -10,  69,  73,   1, -68, -70,   3,  60,  53,   5, -58, -52,  13,  49,  49,  -1, -70, -54,   9,  65,  63, -16, -83, -54,  30,  84,  51, -38, -74, -50,  30,  95,  38, -38, -66, -51,  11,  57,  61,  26, -45, -90, -50,  42, 111,  79, -40,-126, -89,  36, 124,  95, -33,-116, -83,  16, 102,  96, -18, -98,},
{ 128, -22,-108, -66,  35,  98,  51, -57, -96, -29,  68,  85,   0, -84, -65,  32,  85,  27, -58, -67,   0,  65,  60, -34, -77,  -2,  67,  40, -40, -63,   2,  56,  26, -40, -27,  42,  44, -20, -75, -56,  29, 107,  61, -64,-113, -35,  90, 114,  -2,-122,-100,  31, 119,  65, -46, -78, -20,  45,  42,  -8, -41, -39,  12,  42,},
{ -17,   6,  23,   2, -24,  -8,  11,  21,   3, -18, -17,  -8,  23,  33, -18, -46,  -3,  34,  32,  -7, -46, -29,  31,  47,   2, -55, -27,  39,  44,  11, -54, -57,  33,  84,  -3, -84, -26,  63,  71, -20, -78, -51,  41, 100,  22, -97, -79,  46, 118,  30,-107,-110,  29, 148,  58,-101,-132,  14, 148,  82, -85,-151, -21, 136,},
{ -74,  14,  80,  54, -62, -87,   4,  80,  59, -50, -88,   5,  80,  53, -47, -98,  10,  86,  48, -37,-102, -28, 123,  91, -89,-113,  21, 105,  21, -76, -53,  40,  66,  14, -73, -65,  57,  88,  -1, -77, -62,  35,  87,  13, -60, -48,  16,  65,   4, -51,  -5,  38,  22, -36, -60,  17,  90,  35, -78, -78,  30, 100,  30, -93,},
{-112,  38, 111,  30, -73, -63,  39,  81,  -4, -78, -43,  60,  78, -23, -69, -25,  54,  61, -28, -86, -26,  72,  88, -54,-110,  25, 107,  21, -93, -65,  51,  89, -15, -84, -11,  90,  37, -73, -75,  30,  80,  23, -63, -57,  51,  71, -34, -71,  -7,  65,  58, -54, -93,  27,  85,  21, -64, -58,  37,  75,  -5, -93, -28,  80,},
{  93, -29,-107, -21, 107,  54, -80, -92,  35, 122,   6,-127, -46, 107,  82, -51,-110,  -3, 101,  48, -79, -98,  72, 116, -61,-101,  37,  94,  -6, -90, -29,  81,  52, -82, -40,  54,  43,  -5, -64, -24,  52,  37, -18, -51,  -5,  64,   6, -52, -22,  32,  47, -14, -66,  -2,  56,  18, -36, -31,  25,  46,  -9, -63, -11,  48,},
{ -66,  55,  66, -47, -76,  17,  69,  24, -64, -42,  56,  44, -33, -61,  18,  66,  -5, -61,  -2,  47,  20, -27, -57,  34,  73, -21, -78,   5,  79,  12, -87, -37,  87,  55, -86, -73,  68, 103, -27,-126, -12, 110,  52, -69, -89,  31, 117, -11,-118,  -5,  92,  32, -68, -72,  65,  80, -45, -81,  12,  87,  24, -97, -52,  86,},
{ -96,  71,  97, -54, -94,  16,  82,  31, -88, -42,  97,  33, -67, -56,  34,  93, -32,-102,  30,  72,  15, -68, -65, 116,  58,-134, -23, 105,   8, -69, -40,  66,  80,-130, -26, 146, -37, -78,   5,  43,  42, -42, -60,  39,  41, -11, -35, -20,  63,  22, -82, -12,  70,  29, -45, -62,  27,  62, -16, -37,   5,  31,  11, -30,},
{  54, -66, -30,  68,   2, -20, -21,   4,  48, -31, -28,  44, -26,   7,  10, -57,  58,  24, -56,  -4,  25,  38, -29, -75,  90,  30,-123,  34, 105, -25, -96, -22, 113,  45,-157,   7, 134, -29, -88,   0,  68,  42, -54, -83,  60,  72, -28, -68,  -9,  89,  20,-122,   2, 113, -19, -62, -25,  48,  67, -65, -80, 103,  60, -86,},
{ -52,  70,  10, -29, -52,  20, 103, -67, -80,  62,  31,  30, -44, -89,  82,  56, -50, -26, -24,  60,  33,-109,  68,  61,-140,  46,  79, -54,  -2, -18,  33,  34,-118,  85,  64,-158,  61,  78, -72,   5,  -4,   0,  46, -57,   8,  70, -67, -21,  29,   6,  59, -64, -79, 116,  20, -80,  12,  36,  15, -39, -56,  74,  77, -91,},
{  92,-101, -65, 108,  19, -63, -23,  27,  83, -87, -75, 147,  -2,-112,  25,  46,  42, -70, -82, 131,  35,-131,  38,  49, -12,  -7, -67,  68,  52,-110,  24,  76, -66,  13,   7,   1,  23, -68,  14, 100, -66, -50,  43,  -4,  31, -31, -40,  69,  -9, -54,  55,  -7,  -8,   8, -57,  66,  55,-114, -13,  99, -15, -49, -22,  43,},
{   8,  37, -75, -12, 115, -75, -36,  57, -12,  18, -60, -18, 132, -64, -67,  58, -10,  52, -52, -78, 140, -27, -90,  87, -16,  -9,  19, -92, 105,  41,-149,  52,  80, -71,  21, -33,   8,  71, -81,   4,  58, -46,   4,  -1, -15,  75, -68, -48, 121, -44, -45,  50, -33,   7,  34, -75,  77,  13,-112,  70,  53, -73,  10,  11,},
{  51, -49, -18,   9,  71, -32,-109,  87,  68, -76, -42,  21,  88, -38,-113, 107,  55,-123,  -1,  77,  12, -55, -41,  94,   4,-105,  82,  35,-128,  50,  93, -61, -46,  40,  32, -31, -40,  52,  28, -95,  25,  79, -29, -28, -41,  38,  87,-101, -54, 133, -17, -81,  27,  46,  10, -64, -29,  85,   1, -62,  33,  14, -17,   1,},
{ -52,  61,  40, -67, -55, 109,  20,-126,  40,  93, -52, -66,  40,  89, -72, -95, 122,  53,-138, -18, 135, -13,-113,  45, 100, -89, -48, 112, -45, -89,  98,  39,-103,  54,  13, -59,  38,  21, -22, -24,   2,  51, -17, -54,  53,  25, -58,   4,  26,  -3, -18, -11,  38,  15, -61,  16,  52, -40, -34,  27,  42, -18, -57,  44,},
{  -5,  -9,  24,  -8, -21,  27,  -4, -16,   3,  13,   5,  -9,  -6,  -8,   5,  35, -13, -65,  45,  66, -70, -42,  82,  -1, -74,  47,  48, -59, -52,  95,  27,-133,  47, 124,-147, -28, 153, -39, -98,  19,  86,   5, -99,  -6,  96, -15, -76,  25,  72, -13, -84,   2, 105, -30, -88,  31,  76,  -6, -99,  19, 114, -63, -95,  84,},
{  18, -16, -23,  36,  13, -45,   1,  50, -20, -65,  65,  56,-105, -16, 118, -34, -95,  50,  86, -62, -75,  98,   7, -84,  53,  18, -61,  47,  11, -59,  29,  52, -69, -18, 101, -62, -54,  86,  17, -75,  -7,  57,  19, -67, -15,  84,  -7, -93,  41,  88, -64, -78,  74,  71,-101, -41, 142, -30,-123,  72,  62, -51, -58,  54,},
{ -74,  97,  31,-105,  -9, 121, -37,-115, 102,  49,-108,   3,  71, -19, -35,  15,  40, -46, -17,  69, -51, -23,  88, -57, -56, 109, -36, -64,  74,  11, -74,  11,  76, -65, -10,  84, -67, -53,  88,  29, -99,   4,  80, -11, -64,  17,  58, -26, -63,  66,  48,-114,  11, 106, -57, -66,  72,  31, -81,   8,  57, -21, -43,  33,},
{ -29,  34,  14, -43,   7,  51, -43, -37,  71,  -2, -72,  26,  63, -43, -50,  78,   3, -88,  46,  58, -77,   5,  64, -64,   4,  52, -50,  -7,  58, -35, -35,  49,  -4, -49,  73, -27, -60,  74,  31,-101,   6, 107, -39,-109,  86,  78,-121, -28, 133, -28,-117,  76,  79,-117,   3, 113, -82, -53, 106, -17, -80,  52,  37, -38,},
{ -55,  97,  -5,-120,  71,  87,-105, -33, 114, -30, -98,  97,  47,-132,  44,  93,-126,  24, 110, -84, -51,  94, -41, -34,  81, -43, -45,  73, -10, -58,  38,  29, -52,  39,  -1, -69,  86,   4, -86,  33,  77, -61, -57,  86,  31,-110,  33,  65, -48, -29,  43,  17, -55,  20,  33, -37, -11,  29,   1, -17,   1,  11,   2,  -7,},
{ -63, 109, -33, -66,  62,  15, -43,  -3,  45, -28, -36,  78, -42, -45,  90, -48, -44,  91, -25, -62,  66, -18, -42,  78, -39, -49,  92, -26, -74,  72,  16, -72,  41,  35, -68,  35,  20, -47,  18,  43, -51, -21,  66,  -9, -58,  34,  54, -87,   9,  86, -80, -38, 115, -43, -90, 115,  22,-141,  67,  94,-126,   5, 107, -70,},
{  33, -63,  30,  38, -51, -24,  76, -18, -89, 120, -16,-121, 119,  17,-127,  93,  29, -98,  70,  15, -85,  74,  -2, -42,  38, -23,  -4,   6,  24, -14, -36,  57, -36,  -8,  42, -64,  58,  15, -85,  50,  61, -97,   6,  86, -62, -36,  89, -45, -44,  78, -21, -55,  59,  22, -95,  54,  74,-121,  21,  96,-102,  -5,  80, -45,},
{ -38,  66, -15, -58,  55,  21, -72,  45,  19, -71,  67,  -1, -64,  68, -16, -61, 107, -59, -51,  96, -58,   8,  39, -50,  32, -31,  26,   7, -67,  68,   5, -58,  61, -42,  24,  14, -72,  90, -17, -84,  86,  18, -79,  24,  56, -44, -49,  87,  -8, -94,  90,  18, -94,  92, -32, -61, 121, -79, -46, 138,-110, -21, 120, -72,},
{  59,-101,  21,  83, -89,  -6, 103, -99,   1, 102,-113,  13,  90,-102,  22,  85,-106,  23,  62, -53, -18,  57, -43,  -8,  65, -78,  26,  53,-106,  65,  55,-119,  76,  12, -68,  86, -65, -15,  78, -33, -64,  63,  36, -85,  24,  61, -68,  17,  45, -82,  51,  32, -86,  61,   6, -57,  63, -26, -23,  52, -37,  -8,  37, -20,},
{  41, -72,  23,  48, -70,  16,  48, -66,  35,  22, -60,  51,  -3, -44,  55, -19, -46,  71, -19, -48,  69, -30, -39,  79, -64,   0,  70, -80,  26,  53, -83,  30,  47, -94,  80,   0, -90, 108, -17,-100,  98,  45,-134,  37, 109,-121,  11,  96,-117,  46,  55,-107,  72,  25, -96,  74,   0, -45,  54, -44,  14,  30, -58,  32,},
{ -55, 120, -91, -29, 124, -89, -30, 119,-115,  23,  75,-104,  59,  19, -82,  94, -55,  -9,  62, -65,  17,  24, -30,  22, -11,  11, -17,   5,  22, -30,  13,   3, -14,  32, -41,  25,  10, -58,  64,  12, -95,  83,  15,-100, 104, -19, -86, 116, -57, -39, 115,-115,  32,  66,-103,  62,   8, -49,  59, -46,   6,  36, -44,  18,},
{  46,-105, 108, -40, -65, 126, -98,   5,  86,-116,  68,  11, -61,  66, -39,   7,  24, -61,  81, -47, -23,  80, -92,  56,  10, -73, 107, -79,  -6,  77, -89,  47,   4, -36,  49, -59,  57, -21, -43,  87, -53, -46, 106, -61, -32,  87, -69,   7,  50, -80,  63,  -5, -43,  65, -61,  25,  22, -56,  69, -50,  -1,  57, -84,  44,},
{ -60, 142,-152,  69,  48,-104,  91, -42, -11,  48, -62,  50, -10, -25,  35, -42,  57, -29, -42,  80, -67,  25,  24, -62,  80, -65,  19,  46, -87,  67, -12, -41,  69, -64,  41, -16, -23,  72, -86,  39,  44, -94,  60,  23, -70,  54, -12, -16,  26, -27,   9,  13, -24,  33, -41,  40, -18, -34,  92,-104,  28, 100,-174,  94,},
{ -17,  51, -65,  35,  24, -61,  62, -39,  -7,  48, -64,  59, -39,  -2,  45, -70,  81, -64,  13,  53, -97,  97, -59,   6,  64,-119, 119, -73,  -4,  82,-124, 107, -49, -13,  44, -66,  96, -97,  43,  45,-116, 109, -27, -72, 113, -70,   3,  40, -57,  56, -37,   7,  22, -55,  73, -44, -21,  66, -72,  43,  16, -52,  53, -26,},
{  20, -49,  61, -42,   3,  27, -33,  34, -26,  -3,  29, -41,  49, -47,  33,  -7, -24,  45, -46,  20,  17, -46,  60, -70,  63, -15, -49,  94,-104,  63,   7, -62, 100,-116, 106, -75,  12,  60,-111, 112, -44, -48,  94, -76,  24,  34, -76,  83, -60,  14,  59,-116, 121, -85,  24,  47, -91,  88, -57,   3,  56, -96, 102, -46,},
{ -29,  89,-129, 117, -61, -13,  74,-112, 114, -71,   7,  44, -73,  91, -83,  33,  36, -88,  97, -54, -19,  81,-100,  85, -64,  37,   0, -24,  49, -72,  69, -40,   1,  31, -39,  47, -66,  66, -43,  -4,  56, -75,  46,  -3, -33,  54, -58,  44, -10, -30,  70, -91,  80, -57,  21,  30, -68,  75, -62,  31,  27, -85,  93, -39,},
{  26, -64,  99,-108,  81, -33, -15,  55, -79,  83, -65,  24,  33, -80, 105, -93,  50,  -3, -46,  84, -96,  82, -52,   8,  31, -52,  59, -58,  56, -40,  -4,  50, -83,  88, -80,  75, -56,  14,  36, -67,  71, -43, -10,  58, -85,  81, -48,  -6,  63, -97, 103, -87,  47,   9, -61,  80, -68,  36,  12, -54,  79, -81,  59, -22,},
{ -21,  55, -82,  86, -65,  24,  23, -59,  79, -87,  74, -44,  10,  24, -52,  81, -93,  76, -41,  -2,  50, -91, 110,-114, 107, -77,  38,  -2, -42,  82,-100,  91, -59,  24,   0, -22,  51, -75,  81, -63,  24,  24, -61,  67, -51,  14,  33, -63,  75, -72,  56, -23, -16,  62, -94,  90, -58,  27,   0, -43,  80, -99,  89, -36,},
{  -7,  26, -48,  54, -43,  31, -25,  21, -13,   5,   8, -36,  60, -70,  79, -81,  71, -49,  24,   5, -36,  69,-102, 105, -99,  99, -81,  49, -14, -27,  65, -87, 100,-110, 109,-101,  90, -66,  25,  18, -61,  93, -93,  59, -15, -29,  69, -90,  89, -68,  25,  26, -60,  78, -81,  62, -29,  -7,  35, -54,  71, -81,  71, -30,},
{ -18,  56,-103, 131,-138, 132, -98,  44,  16, -69, 117,-151, 155,-127,  82, -23, -40,  86,-108, 112, -96,  70, -44,  17,  11, -43,  62, -64,  60, -47,  23,   3, -24,  34, -37,  40, -37,  32, -34,  26, -15,   8,  -4,   4,  -9,  20, -29,  33, -34,  33, -21,  -4,  28, -42,  45, -36,  16,   0, -10,  30, -46,  50, -41,  17,},
{  -7,  22, -39,  48, -52,  58, -55,  48, -46,  40, -26,   4,  27, -51,  59, -59,  66, -77,  83, -82,  63, -35,  10,  13, -33,  47, -61,  78, -87,  86, -72,  52, -25,  -7,  25, -43,  78,-114, 130,-118,  82, -32, -30,  85,-112, 113,-103,  80, -37, -12,  44, -58,  63, -59,  46, -17, -14,  39, -63,  88,-102,  94, -70,  28,},
{  12, -35,  61, -77,  89,-103, 101, -90,  76, -60,  41, -13, -21,  49, -70,  87,-108, 132,-139, 123, -99,  80, -65,  55, -42,  23, -11,   6,   3, -18,  31, -38,  48, -58,  61, -76,  94, -89,  66, -39,   8,  25, -52,  59, -52,  39, -17,  -7,  29, -48,  64, -74,  71, -61,  52, -35,   9,   8, -26,  55, -78,  79, -61,  24,},
{   0,  -1,   7, -11,  16, -25,  29, -27,  24, -22,  25, -30,  36, -43,  49, -53,  60, -68,  75, -80,  78, -82,  91, -94, 104,-121, 129,-135, 146,-152, 142,-116,  88, -67,  59, -62,  65, -68,  62, -47,  31, -15,   3,   3,  -2,  -5,  10, -13,  17, -28,  45, -55,  53, -52,  55, -55,  53, -44,  31, -18,   4,   7, -11,   4,},
{   1,  -1,   1,   0,  -3,   4,  -2,   2,  -5,   5,   0,  -4,   3,  -7,  13, -21,  28, -37,  48, -47,  37, -35,  33, -21,  10,  -6,  -3,  15, -21,  25, -27,  30, -34,  35, -34,  43, -68,  94,-110, 118,-123, 119, -98,  50,  13, -68, 114,-138, 145,-147, 137,-117,  91, -69,  49, -17, -19,  40, -56,  73, -90,  95, -74,  29,},
{  -2,   5, -12,  25, -40,  55, -68,  78, -81,  81, -83,  84, -76,  66, -58,  51, -44,  39, -29,  13,  -1,  -7,  14, -25,  40, -58,  68, -73,  78, -79,  75, -66,  53, -41,  32, -26,  14,   3, -18,  29, -37,  41, -40,  41, -40,  35, -27,  19, -18,  22, -31,  49, -69,  91,-118, 137,-149, 150,-139, 119, -89,  65, -47,  19,},
{ -13,  32, -55,  80,-101, 117,-123, 123,-119, 118,-118, 116,-106,  90, -77,  66, -60,  62, -62,  51, -33,  18,  -7,   4,  -2,  -3,   5,  -3,   2,  -5,   8, -12,  18, -26,  27, -25,  27, -32,  39, -44,  48, -51,  48, -43,  43, -47,  55, -61,  63, -66,  70, -73,  74, -77,  86, -88,  82, -74,  64, -58,  50, -37,  24, -10,},
{   0,   2,  -4,   6,  -8,   9,  -9,  11, -13,  19, -38,  50, -46,  40, -35,  29, -18,   3,   8, -10,   9, -13,  18, -20,  20, -12,   7, -10,  14, -13,   8,  -6,   3,   5,  -6,   6, -15,  39, -72, 104,-131, 157,-174, 174,-162, 146,-124,  99, -73,  52, -27,   1,  16, -31,  48, -67,  87, -94,  89, -76,  62, -55,  46, -19,},
},

{
{  19,  21,  23,  24,  27,  30,  32,  35,  37,  39,  41,  43,  45,  47,  49,  51,  53,  55,  57,  59,  60,  62,  63,  65,  67,  68,  70,  71,  72,  72,  73,  74,  74,  75,  76,  76,  77,  77,  77,  78,  78,  78,  78,  78,  78,  78,  78,  77,  77,  76,  75,  75,  74,  73,  72,  71,  69,  68,  67,  66,  64,  63,  62,  60,},
{ -46, -51, -56, -60, -64, -68, -71, -74, -78, -81, -84, -86, -89, -90, -91, -91, -90, -88, -86, -83, -81, -77, -74, -69, -65, -60, -54, -49, -44, -39, -33, -26, -20, -13,  -7,  -1,   6,  12,  17,  23,  28,  33,  37,  42,  47,  51,  54,  58,  61,  64,  67,  69,  72,  73,  75,  76,  76,  76,  76,  75,  75,  73,  72,  69,},
{ -58, -66, -73, -79, -83, -87, -89, -88, -87, -85, -82, -78, -72, -65, -55, -45, -35, -24, -13,  -3,   8,  19,  30,  40,  49,  57,  65,  71,  76,  81,  84,  86,  87,  87,  85,  83,  79,  74,  68,  62,  55,  47,  39,  30,  21,  13,   5,  -4, -12, -20, -28, -36, -45, -53, -60, -66, -72, -76, -80, -82, -83, -83, -81, -78,},
{ -65, -73, -79, -83, -84, -83, -80, -75, -67, -58, -46, -32, -18,  -3,  13,  27,  40,  52,  63,  71,  79,  84,  86,  87,  85,  81,  74,  64,  54,  42,  28,  13,  -2, -17, -31, -46, -59, -69, -78, -85, -89, -91, -91, -87, -81, -73, -65, -56, -46, -33, -21,  -8,   4,  16,  28,  40,  53,  64,  73,  80,  85,  88,  88,  87,},
{ -72, -80, -84, -83, -79, -72, -63, -50, -34, -16,   1,  18,  36,  52,  68,  80,  88,  90,  90,  86,  77,  63,  47,  28,   8, -12, -32, -51, -68, -81, -90, -95, -95, -91, -84, -72, -55, -36, -17,   3,  23,  40,  55,  67,  76,  82,  86,  86,  82,  75,  66,  55,  41,  26,  11,  -6, -22, -37, -52, -65, -73, -78, -79, -75,},
{ -73, -81, -83, -79, -70, -58, -41, -21,   1,  24,  46,  65,  78,  86,  88,  84,  75,  60,  41,  19,  -4, -28, -51, -71, -86, -93, -93, -86, -73, -55, -33, -10,  15,  40,  63,  82,  93,  96,  94,  84,  70,  51,  31,  10, -12, -34, -54, -71, -84, -89, -90, -85, -75, -60, -42, -21,  -2,  17,  35,  51,  64,  73,  77,  75,},
{ -85, -92, -89, -78, -59, -35,  -8,  20,  46,  69,  85,  92,  90,  80,  63,  40,  13, -15, -42, -65, -82, -91, -89, -78, -60, -35,  -8,  20,  46,  67,  81,  87,  83,  72,  56,  33,   6, -22, -47, -66, -78, -83, -82, -75, -61, -41, -18,   8,  35,  58,  75,  85,  90,  87,  76,  58,  36,  10, -17, -41, -60, -75, -82, -82,},
{ -96,-101, -92, -71, -40,  -6,  29,  61,  86, 100, 101,  87,  62,  30,  -7, -41, -68, -85, -93, -89, -73, -46, -12,  24,  54,  77,  88,  87,  73,  49,  21,  -9, -36, -59, -75, -82, -78, -62, -37, -10,  17,  41,  60,  73,  78,  74,  63,  45,  21,  -5, -32, -56, -73, -84, -85, -78, -63, -40, -12,  18,  45,  67,  80,  84,},
{ -84, -84, -69, -41,  -6,  29,  59,  79,  87,  81,  60,  29,  -7, -41, -71, -88, -88, -74, -48, -13,  25,  60,  85,  96,  90,  69,  36,  -4, -42, -71, -89, -91, -76, -50, -17,  21,  54,  80,  91,  87,  69,  41,   7, -29, -61, -84, -95, -89, -71, -43, -12,  20,  49,  73,  85,  85,  74,  54,  28,  -3, -34, -60, -74, -79,},
{ -83, -81, -60, -25,  17,  52,  76,  85,  77,  54,  20, -19, -53, -77, -83, -71, -45, -10,  29,  64,  86,  90,  73,  39,  -4, -46, -77, -94, -90, -66, -29,  13,  54,  82,  94,  86,  60,  21, -19, -56, -79, -89, -83, -63, -31,   9,  46,  76,  93,  91,  72,  39,  -1, -39, -69, -87, -91, -78, -52, -16,  22,  54,  75,  83,},
{ -85, -81, -57, -17,  28,  65,  86,  83,  63,  31, -11, -51, -80, -88, -73, -37,   7,  49,  78,  88,  75,  41,  -8, -55, -85, -91, -70, -29,  17,  58,  85,  90,  69,  34,  -7, -48, -78, -90, -77, -47,  -7,  34,  68,  89,  89,  67,  30, -16, -58, -85, -93, -80, -51, -10,  34,  70,  88,  88,  71,  38,  -2, -44, -75, -90,},
{  95,  82,  42, -10, -60, -91, -95, -72, -28,  24,  71,  98,  94,  62,   6, -51, -88, -99, -80, -37,  16,  65,  93,  92,  62,  14, -38, -78, -91, -76, -38,  11,  56,  84,  86,  61,  17, -30, -66, -83, -78, -52, -10,  35,  71,  87,  79,  47,   3, -40, -71, -82, -69, -38,   3,  41,  69,  77,  67,  39,   2, -34, -60, -73,},
{  94,  72,  26, -28, -71, -87, -72, -34,  16,  60,  81,  71,  37,  -9, -53, -78, -75, -47,   1,  48,  79,  81,  50,   5, -43, -78, -81, -57, -13,  37,  75,  87,  64,  19, -34, -76, -89, -72, -31,  22,  67,  91,  85,  50,  -2, -57, -94, -99, -69, -19,  36,  80,  98,  86,  45, -10, -61, -93, -97, -69, -19,  34,  73,  90,},
{  98,  72,  18, -43, -88, -94, -61,  -6,  49,  81,  83,  53,   1, -52, -83, -81, -45,  13,  64,  88,  72,  26, -30, -74, -84, -59,  -8,  47,  79,  77,  46,  -4, -54, -85, -82, -42,  18,  70,  93,  76,  31, -27, -74, -94, -78, -32,  27,  75,  93,  78,  34, -20, -66, -89, -77, -35,  16,  59,  88,  83,  47, -10, -63, -92,},
{  86,  59,   2, -56, -85, -73, -24,  32,  73,  79,  50,  -2, -53, -80, -71, -31,  23,  66,  79,  56,   5, -52, -82, -68, -21,  34,  72,  79,  48,  -5, -57, -88, -75, -29,  36,  84,  91,  55,  -8, -65, -91, -79, -33,  27,  78,  97,  69,   4, -63, -97, -86, -37,  27,  77,  95,  71,  17, -42, -91,-101, -68,  -4,  65, 106,},
{  88,  56,  -5, -62, -89, -70, -14,  50,  88,  81,  31, -38, -91, -96, -44,  32,  90, 102,  60, -16, -87,-108, -62,  16,  79,  99,  62,  -9, -71, -95, -71,  -9,  56,  92,  79,  24, -44, -84, -79, -33,  27,  71,  77,  46,  -6, -57, -78, -55,  -5,  44,  71,  62,  21, -33, -71, -71, -36,  14,  63,  85,  64,  11, -48, -86,},
{  74,  43, -15, -67, -79, -44,  21,  75,  85,  43, -26, -82, -90, -41,  36,  90,  90,  30, -49, -96, -81, -12,  59,  88,  64,   4, -59, -86, -64,  -6,  56,  90,  69,   6, -62, -93, -66,  -3,  64,  93,  65,   4, -61, -92, -69,  -8,  59,  90,  67,   6, -59, -91, -71, -12,  53,  87,  72,  22, -45, -88, -80, -25,  43,  91,},
{ 106,  45, -46,-105, -89, -17,  66, 105,  72, -10, -84,-100, -45,  38,  93,  85,  22, -54, -91, -70,  -5,  62,  84,  46, -16, -64, -70, -30,  32,  73,  65,  10, -56, -84, -52,  17,  75,  81,  28, -42, -81, -68, -12,  51,  80,  61,   2, -58, -81, -51,  13,  71,  84,  43, -26, -78, -81, -41,  34,  88,  84,  30, -41, -88,},
{ 104,  33, -57, -96, -63,  15,  78,  80,  24, -50, -83, -50,  15,  67,  67,  16, -42, -70, -46,  10,  61,  67,  19, -44, -72, -49,  12,  68,  75,  24, -47, -88, -59,  14,  80,  84,  24, -56, -96, -63,  13,  80,  91,  39, -37, -91, -83, -16,  64,  99,  61, -18, -85, -96, -39,  47,  96,  85,   8, -77,-102, -54,  32,  98,},
{ 102,  15, -71, -96, -34,  54,  81,  49, -20, -68, -54, -16,  34,  62,  38,  -5, -49, -64, -16,  46,  73,  46, -40, -96, -63,  20, 102, 101, -10,-106,-109, -18,  96, 112,  33, -62,-106, -50,  45,  77,  57,  -2, -55, -55, -30,  10,  52,  54,  23, -30, -77, -59,  11,  80,  89,  22, -71,-102, -47,  45, 103,  70, -25, -91,},
{  -2,  29,   6, -29, -55, -28,  64,  88,  27, -70,-122, -25,  93, 117,  34,-111,-131,  -1, 107, 120,   8,-132, -98,  28, 102,  88, -33,-113, -46,  31,  73,  44, -50, -47,  13,  33,  24, -36, -65,  18,  63,  51,   0, -85, -64,  28,  71,  64, -23, -96, -37,  37,  69,  39, -46, -71,  -5,  37,  43,  16, -47, -43,  12,  33,},
{ -71,  -3,  57,  64,   6, -56, -56, -13,  40,  55,  14, -24, -42, -30,  12,  37,  37,  11, -39, -55, -22,  29,  72,  49, -30, -83, -67,  19, 102,  84, -10,-101,-110,  -6, 110, 116,  14,-108,-125, -12,  99, 119,  34, -93,-118, -30,  71, 105,  43, -59, -92, -51,  31,  87,  60, -13, -62, -64, -14,  46,  61,  39, -11, -66,},
{ 138,   4,-120,-119,  12, 128, 103, -24,-126,-105,  29, 127,  94, -33,-131, -88,  47, 126,  76, -49,-122, -62,  56, 107,  50, -55, -94, -33,  48,  76,  25, -48, -52, -11,  29,  33,  -6, -21,  -3,   6,  11,  -3, -14,   4,  11,   2, -10, -17,   8,  34,  11, -25, -39,  -6,  40,  46,  -2, -45, -41,   0,  40,  41,  -1, -42,},
{   7,   2,  -7,  -7,  -1,   9,  13,  -4, -18,  -9,  15,  29,   2, -28, -23,   3,  33,  29, -19, -51, -24,  35,  65,  17, -55, -71, -11,  68,  77,  -4, -83, -74,  15,  93,  76, -26,-100, -74,  27, 113,  81, -34,-114, -84,  29, 122,  87, -37,-128, -94,  42, 133,  93, -35,-124, -90,  25, 108,  81, -17, -94, -79,   7,  87,},
{-106,  19, 102,  61, -52, -94, -26,  68,  83,   2, -77, -63,  20,  75,  40, -48, -64,   5,  54,  39, -31, -57,   4,  50,  34, -26, -65,  -6,  59,  46, -19, -70, -32,  52,  70,  -1, -78, -51,  41,  82,  27, -72, -88,  14,  90,  67, -36,-107, -41,  72, 101,  13,-103, -87,  31, 110,  69, -59,-115, -30,  66, 101,  22,-101,},
{ -90,  27,  90,  34, -66, -74,  12,  82,  46, -50, -82,  -4,  82,  56, -49, -90,  -8,  83,  68, -37, -92, -17,  79,  65, -31, -93, -26,  75,  64, -19, -81, -37,  69,  73, -19, -81, -35,  67,  77, -24, -92, -46,  59,  94,   7, -86, -68,  41,  93,  17, -76, -73,  27,  96,  31, -65, -81,   4,  88,  51, -46, -86, -22,  82,},
{ -94,  43, 100,   9, -99, -54,  64,  92,  -5, -99, -55,  70,  94, -14,-100, -41,  74,  77, -26, -91, -26,  78,  66, -45, -84,   0,  80,  50, -44, -83,  -9,  85,  49, -49, -76,   0,  83,  44, -57, -72,   3,  78,  47, -56, -65,  12,  70,  29, -61, -55,  30,  71,  19, -74, -59,  46,  83,  13, -81, -69,  38, 100,  25, -87,},
{  87, -52, -90,   9,  90,  30, -69, -63,  40,  76,   0, -72, -37,  53,  62, -25, -70, -10,  63,  48, -47, -76,  19,  84,  21, -73, -57,  48,  74,  -7, -78, -36,  66,  69, -39, -86,   2,  89,  39, -70, -77,  27,  96,  20, -85, -60,  53,  87,  -8, -99, -34,  82,  70, -52, -98,  14, 102,  42, -73, -89,  20, 109,  40, -94,},
{  71, -48, -74,  27,  81,  -2, -74, -33,  57,  60, -33, -78,   0,  80,  36, -73, -64,  50,  79,  -8, -89, -31,  94,  52, -80, -72,  55,  91, -19, -98, -25,  91,  56, -63, -76,  27,  88,   7, -86, -40,  62,  70, -24, -89, -12,  91,  46, -75, -79,  48,  96, -15, -97, -17,  83,  51, -65, -73,  37,  84,  -1, -84, -41,  75,},
{ -72,  53,  72, -33, -79,   9,  82,  22, -79, -47,  66,  64, -46, -78,  24,  90,   2, -92, -33,  83,  59, -72, -77,  58,  92, -35, -97,   6,  87,  25, -79, -50,  68,  68, -46, -80,  21,  91,   1, -93, -29,  79,  58, -52, -81,  31,  89, -13, -89, -10,  82,  37, -76, -56,  57,  74, -36, -87,  11,  90,  19, -87, -47,  75,},
{ -68,  57,  68, -53, -74,  40,  81, -18, -85,  -6,  87,  22, -80, -40,  70,  60, -58, -76,  44,  89, -29,-104,  19, 114,  -4,-116,  -8, 108,  21, -91, -43,  81,  59, -71, -75,  58,  81, -39, -78,  20,  82,  -9, -84,  -4,  78,  21, -72, -37,  63,  55, -57, -66,  45,  72, -31, -80,  15,  87,   2, -82, -21,  73,  45, -65,},
{  46, -39, -50,  44,  45, -38, -42,  18,  59, -10, -67,   5,  64,   5, -68, -11,  70,  16, -71, -21,  79,  15, -74, -21,  64,  38, -70, -41,  71,  40, -63, -48,  54,  66, -48, -79,  46,  82, -46, -76,  33,  82, -18, -99,  11, 109,   1,-110, -18, 115,  28,-118, -34, 118,  39,-120, -42, 109,  50, -92, -62,  89,  73, -89,},
{ -87,  92,  70, -94, -70,  95,  70, -86, -77,  79,  84, -74, -83,  63,  81, -52, -81,  44,  82, -37, -85,  45,  83, -58, -77,  60,  68, -56, -62,  53,  59, -52, -53,  44,  58, -44, -66,  49,  65, -52, -60,  46,  65, -40, -68,  36,  66, -34, -66,  40,  61, -42, -56,  47,  55, -56, -56,  59,  55, -56, -55,  56,  55, -59,},
{ -61,  70,  42, -86, -19,  86,   7, -74, -18,  79,  28, -95, -16,  93,   4, -77, -12,  72,  24, -87, -11, 107, -35, -91,  60,  66, -61, -55,  57,  60, -68, -55,  83,  37, -80, -26,  70,  40, -80, -48,  93,  42, -92, -27,  76,  21, -64, -23,  69,  24, -91,  -2,  95, -22, -84,  29,  83, -33, -92,  50,  90, -65, -82,  77,},
{  56, -70, -37,  94,   4, -92,  23,  73, -20, -76,  18,  91, -38, -85,  59,  60, -55, -58,  56,  67, -90, -50, 133, -14,-117,  52,  77, -59, -50,  48,  55, -58, -57,  80,  38, -90, -12,  83,  -2, -73,   5,  75, -10, -84,  28,  85, -54, -63,  62,  45, -55, -50,  55,  53, -83, -27, 108, -14,-100,  41,  70, -43, -55,  49,},
{ -63,  86,  23,-105,  32,  85, -66, -50,  61,  44, -58, -55,  84,  34,-105,  16,  83, -37, -63,  44,  63, -70, -45,  98,   2, -99,  43,  72, -71, -36,  66,  24, -59, -30,  68,  27, -84,  -4,  87, -27, -65,  38,  50, -30, -64,  51,  64, -86, -28, 102, -29, -74,  54,  47, -59, -52,  81,  42,-106,  -4, 113, -45, -92,  74,},
{  46, -93,  43,  31, -12, -15, -55,  94,   3, -91,  77, -25, -29,  55, -39,  31,  -4, -84, 109,  -2, -98, 109, -66, -33, 114, -81,  -1,  69,-108,  43,  71, -87,  35,   1, -42,  73, -38, -28,  78, -92,  17,  89, -82,  13,  11, -41,  72, -25, -47,  77, -66,  -7,  90, -74,  -6,  77,-109,  40,  83,-110,  35,  38, -61,  28,},
{  53, -77, -15,  90, -34, -67,  55,  45, -61, -33,  69,  20, -83,   8,  87, -49, -63,  74,  33, -80,  -6,  90, -47, -67,  89,  24,-106,  32,  83, -66, -51,  81,  16, -78,   7,  73, -23, -70,  48,  65, -77, -44,  88,  19, -80,  -1,  78, -21, -84,  65,  66,-108,  -8, 113, -54, -82,  87,  42,-102,   9,  79, -30, -57,  43,},
{ -36,  65, -11, -72,  82,   4, -80,  46,  34, -31, -27,  18,  55, -62, -27, 100, -54, -60,  80,   8, -59,  12,  38,  -7, -52,  34,  53, -90,   1, 100, -62, -58,  91,  -3, -64,  21,  44, -12, -62,  33,  71, -83, -31, 117, -45, -82,  75,  33, -57, -30,  64,  22,-100,  36,  99,-122, -16, 143, -93, -66, 125, -21, -78,  50,},
{ -14,   8,  27, -29, -38,  98, -41, -87, 117, -11, -81,  56,  13, -11, -40,  20,  82,-123,   1, 152,-146,  -6, 140,-138,   9, 110, -97,  -2,  53, -19, -11, -16,  32,  20, -80,  50,  55,-101,  28,  60, -62,   5,  29, -20,  15, -30,  22,  34, -78,  38,  50, -91,  38,  59, -98,  53,  26, -77,  56,  12, -49,  23,   6,  -4,},
{ -64, 119, -44, -92, 122, -10, -83,  47,  34, -28, -45,  49,  55,-116,  23, 116,-117, -25, 125, -67, -54,  89, -22, -44,  32,  28, -42, -20,  75, -24, -85,  92,  24,-110,  57,  57, -94,  15,  65, -38, -34,  30,  35, -38, -44,  87, -10, -92,  81,  26, -86,  37,  38, -47,  -1,  30,   1, -43,  23,  52, -74, -12,  90, -54,},
{  13, -20,  -6,  27,   5, -53,  36,  40, -73,  18,  54, -66,   9,  44, -37,  -6,  22,   4, -29,   5,  50, -64,   9,  74, -97,   9, 113,-120, -18, 140, -88, -74, 135, -25, -97,  74,  46, -94,  -2,  99, -47, -79,  92,  27,-112,  50,  79,-113,   8, 104, -87, -29,  97, -43, -49,  66,   2, -59,  34,  32, -50,  -4,  51, -31,},
{ -48,  90, -37, -58,  78,   1, -67,  33,  51, -68,  -7,  77, -45, -46,  73,  -7, -62,  51,  27, -77,  35,  45, -74,  35,  36, -81,  40,  62, -95,   2,  96, -70, -46, 103, -21, -86,  76,  34, -99,  34,  72, -78, -15,  76, -23, -68,  70,  25,-102,  67,  50,-115,  52,  65,-109,  41,  70,-102,  21,  76, -83,  -3,  73, -44,},
{  35, -55,   1,  59, -40, -46,  86, -13, -95, 104,  12,-126, 104,  34,-129,  80,  40, -93,  39,  41, -66,  30,  17, -36,  29, -11,  -9,  22, -13,  -4,  -1,  13,  11, -51,  47,  13, -78,  86, -10, -96, 116, -18, -94, 111, -27, -73,  76,  -2, -35,   9,  22, -12, -31,  57, -23, -57, 100, -50, -55, 131, -85, -59, 133, -65,},
{ -59, 119, -67, -66, 145, -69, -89, 148, -39,-115, 140, -12,-123, 120,   6,-120, 105,  15,-102,  78,   6, -59,  63, -30, -14,  31, -22,   6,  -3,  10,  -8, -18,  49, -35, -30,  76, -53, -22,  82, -70, -10,  77, -58, -11,  54, -36, -21,  47,  -9, -35,  28,  16, -42,  30,  13, -55,  51,  -7, -37,  58, -34, -29,  63, -31,},
{  12, -29,  26,  -1, -28,  38, -19, -16,  42, -34,  -6,  40, -38,   4,  33, -50,  30,  12, -40,  48, -42,  11,  45, -86,  67,   8, -88, 118, -60, -56, 119, -74, -16,  81, -73,  -3,  56, -27, -33,  37,  19, -73,  50,  54,-126,  72,  59,-149, 119,   6,-119, 127, -37, -66, 115, -86,   2,  66, -88,  72,  -8, -71,  91, -40,},
{  32, -68,  57,   0, -57,  58,  -5, -51,  55,  -1, -46,  35,  10, -31,  10,  26, -33,  -3,  49, -56,   6,  60, -87,  48,  34, -97,  79,  17,-111, 105,   3,-109, 112,  -9, -96,  99,  -6, -84,  84,   2, -86,  77,  24,-100,  58,  54,-109,  63,  36,-108,  84,  16, -91,  76,   0, -74,  86, -15, -70,  96, -46, -36,  73, -36,},
{ -40,  91, -76,  -4,  85, -99,  29,  77,-120,  46,  74,-118,  48,  64,-115,  59,  48,-109,  78,  14, -84,  85, -33, -32,  68, -48, -12,  50, -26, -25,  45,  -9, -58,  73,  -4, -82, 104, -37, -73, 137, -84, -40, 118, -98,   5,  86, -93,  24,  43, -66,  46,   6, -41,  32, -16,   9,   0,  -5,  14, -28,  20,  13, -26,  10,},
{ -46, 110,-119,  64,  17, -79,  93, -51, -21,  72, -74,  36,  16, -52,  57, -34,  -5,  43, -56,  34,  -3, -16,  35, -59,  60, -24, -26,  74, -92,  49,  28, -80,  89, -44, -37,  91, -82,  34,  18, -63,  75, -43,  -8,  56, -82,  62,  -4, -51,  75, -64,  21,  33, -69,  78, -58,  12,  42, -82, 103, -77, -21, 132,-156,  69,},
{  30, -78,  83, -31, -46, 100, -90,  11,  84,-118,  66,  34,-102,  91, -20, -59,  95, -68,   7,  47, -68,  62, -41,  10,  26, -54,  63, -43,  -7,  65, -92,  59,  21,-105, 121, -46, -63, 130,-112,  24,  77,-118,  77,   9, -77,  87, -40, -24,  60, -54,  28,   8, -41,  48, -30,  -5,  41, -54,  42, -14, -20,  41, -37,  14,},
{  38,-101, 122, -83,   9,  59, -95,  83, -24, -45,  86, -82,  40,  20, -70,  88, -70,  18,  50, -99, 102, -69,  22,  30, -81,  99, -71,   9,  56, -89,  72, -17, -38,  66, -57,  16,  32, -49,  41, -27,   8,  15, -33,  46, -44,  17,  23, -57,  70, -55,  11,  46, -87,  88, -41, -37,  98,-111,  84, -19, -59, 101, -92,  39,},
{ -32,  82,-106,  90, -47,  -5,  48, -66,  53, -17, -27,  66, -82,  60,  -9, -43,  77, -89,  73, -19, -44,  85, -99,  80, -30, -34,  79, -90,  67, -17, -37,  71, -79,  57, -16, -35,  74, -77,  47,   3, -53,  82, -75,  35,  22, -75,  94, -65,  11,  49, -98, 110, -76,  14,  52, -94,  95, -63,  17,  29, -66,  83, -71,  29,},
{ -28,  76,-100,  89, -56,  14,  29, -60,  68, -51,  15,  30, -70,  81, -59,  13,  39, -75,  87, -72,  36,  10, -54,  88, -95,  69, -20, -31,  68, -79,  60, -20, -18,  51, -72,  67, -37,  -9,  53, -78,  75, -41, -14,  65, -93,  85, -44, -13,  64, -94,  90, -56,   8,  45, -90, 111, -99,  63, -18, -35,  88,-111,  90, -34,},
{  23, -66,  99, -99,  73, -37,  -4,  42, -68,  71, -47,   5,  41, -76,  93, -81,  34,  34, -81,  93, -85,  66, -35,  -7,  49, -82,  95, -81,  47,  -2, -41,  67, -74,  58, -19, -29,  63, -73,  63, -35,  -9,  53, -76,  71, -43,  -3,  54, -89,  95, -76,  44,   6, -63, 101,-104,  75, -31, -12,  48, -77,  93, -93,  73, -29,},
{ -25,  74,-113, 127,-123, 102, -56,  -6,  62, -94,  95, -71,  32,  12, -55,  89,-103,  87, -45,  -6,  45, -67,  77, -78,  65, -34,  -1,  40, -74,  88, -74,  39,   5, -47,  73, -77,  58, -19, -23,  59, -81,  83, -59,  18,  22, -49,  63, -63,  50, -22, -16,  51, -75,  87, -78,  53, -24,  -2,  25, -44,  60, -72,  62, -25,},
{  10, -33,  55, -68,  78, -84,  76, -50,  10,  29, -59,  70, -60,  31,   8, -43,  69, -91, 103, -93,  68, -39,  10,  19, -44,  61, -66,  57, -32,  -7,  48, -81, 102,-100,  72, -26, -19,  50, -65,  64, -48,  17,  26, -72, 103,-110,  95, -63,  23,  15, -46,  66, -69,  57, -33,  -1,  34, -62,  85,-106, 119,-109,  78, -29,},
{  -9,  28, -47,  61, -73,  84, -85,  72, -47,  14,  20, -49,  65, -62,  39,  -5, -34,  72,-102, 117,-122, 124,-128, 127,-109,  71, -28, -13,  52, -82,  90, -76,  47,  -4, -41,  75, -92,  91, -76,  51, -21, -10,  40, -58,  55, -34,   5,  23, -47,  67, -81,  85, -79,  66, -45,  20,   0, -13,  23, -28,  27, -27,  21,  -8,},
{  -8,  23, -35,  37, -28,  11,  11, -32,  50, -59,  61, -58,  48, -29,   6,  13, -24,  25, -15,   5,   4, -17,  35, -55,  73, -84,  85, -82,  78, -64,  40, -16,  -9,  34, -43,  33, -10, -22,  47, -66,  79, -84,  78, -60,  31,  -1, -19,  26, -21,   7,  12, -33,  57, -87, 114,-135, 149,-150, 143,-138, 126,-105,  70, -24,},
{  10, -32,  54, -68,  83, -98, 101, -91,  70, -38,  -6,  52, -94, 120,-131, 128,-115,  91, -60,  29,  -6, -13,  33, -51,  63, -67,  64, -53,  38, -17, -11,  40, -65,  82, -90,  88, -74,  49, -21,  -4,  27, -47,  56, -50,  32,  -9, -13,  36, -57,  79, -96, 103, -98,  85, -70,  56, -44,  32, -22,  13,  -6,   1,   0,   0,},
{  -5,  10, -10,   7,  -4,  -1,  12, -27,  43, -57,  65, -67,  63, -56,  46, -38,  33, -28,  24, -22,  26, -32,  40, -55,  72, -87,  99,-109, 116,-114,  99, -77,  53, -24,  -6,  30, -45,  46, -35,  22,  -4, -22,  54, -86, 110,-123, 126,-121, 107, -88,  67, -45,  19,   7, -31,  53, -69,  79, -83,  86, -84,  75, -55,  20,},
{   3, -11,  26, -43,  63, -85,  98, -97,  83, -62,  31,   5, -37,  61, -77,  88, -95,  98, -96,  92, -85,  75, -64,  57, -47,  32, -15,  -3,  24, -43,  56, -61,  62, -56,  44, -26,   1,  30, -62,  89,-109, 116,-106,  82, -49,  13,  19, -47,  69, -85,  97,-102,  97, -84,  68, -51,  40, -32,  22, -11,   0,   8, -10,   4,},
{   0,  -6,  17, -32,  52, -74,  94,-106, 111,-111, 105, -94,  77, -53,  29,  -9,  -8,  25, -40,  47, -51,  56, -62,  67, -68,  66, -59,  50, -41,  28, -10, -12,  38, -67,  94,-114, 127,-133, 133,-128, 118,-101,  81, -59,  37, -19,   4,   6, -15,  21, -30,  37, -37,  34, -31,  27, -26,  27, -26,  25, -22,  18, -15,   6,},
{  -2,   7, -12,  16, -22,  30, -40,  52, -58,  60, -63,  67, -70,  69, -65,  62, -55,  47, -41,  38, -37,  37, -40,  46, -51,  54, -61,  71, -84,  92, -90,  84, -78,  69, -54,  37, -18,  -5,  31, -53,  71, -87, 101,-118, 128,-129, 127,-122, 111, -98,  88, -76,  61, -46,  32, -22,  15, -10,   7,  -3,  -1,   2,  -4,   3,},
{  -4,  10, -16,  23, -36,  54, -72,  86, -96, 106,-114, 118,-114, 102, -88,  75, -63,  51, -37,  23, -10,  -2,  13, -25,  37, -45,  53, -66,  82, -95, 104,-112, 121,-126, 124,-120, 111, -97,  83, -69,  54, -39,  25, -11,  -2,  11, -12,   9,  -8,   8,  -9,  10,  -9,  10, -11,  10, -10,  10, -10,  11,  -9,   8,  -5,   1,},
},

{
{  17,  20,  22,  24,  26,  28,  30,  32,  34,  36,  38,  40,  42,  44,  46,  47,  49,  51,  52,  54,  55,  57,  58,  60,  62,  64,  65,  66,  67,  68,  70,  71,  72,  72,  73,  74,  75,  75,  76,  76,  77,  77,  78,  78,  78,  79,  79,  79,  79,  79,  79,  79,  78,  78,  77,  76,  76,  75,  74,  74,  73,  72,  71,  69,},
{ -44, -50, -56, -60, -64, -68, -72, -75, -78, -82, -84, -86, -88, -88, -89, -88, -88, -87, -87, -85, -82, -79, -77, -74, -70, -65, -61, -56, -52, -47, -41, -35, -29, -23, -17, -10,  -3,   3,   8,  14,  20,  25,  30,  36,  41,  46,  50,  54,  58,  61,  64,  67,  69,  71,  73,  75,  76,  76,  76,  75,  75,  73,  71,  69,},
{  55,  63,  69,  74,  78,  82,  84,  85,  85,  83,  81,  78,  73,  67,  58,  50,  41,  31,  22,  13,   2,  -9, -20, -30, -41, -50, -59, -67, -74, -80, -85, -88, -90, -91, -91, -89, -86, -81, -76, -71, -65, -58, -50, -41, -31, -22, -12,  -1,   8,  17,  26,  34,  42,  50,  57,  63,  69,  74,  77,  79,  80,  81,  79,  77,},
{  65,  74,  80,  84,  86,  86,  83,  78,  71,  61,  50,  37,  23,   9,  -6, -21, -35, -47, -58, -68, -76, -83, -87, -89, -88, -83, -78, -70, -61, -50, -37, -22,  -7,   7,  22,  36,  50,  61,  71,  79,  85,  89,  90,  89,  85,  79,  72,  64,  53,  41,  28,  15,   2, -11, -25, -38, -51, -61, -70, -76, -80, -82, -82, -78,},
{  74,  83,  88,  89,  87,  80,  69,  54,  37,  18,  -1, -20, -38, -55, -70, -82, -90, -93, -92, -87, -79, -67, -51, -33, -13,   7,  28,  46,  61,  73,  81,  87,  89,  87,  80,  68,  55,  39,  21,   2, -16, -33, -49, -62, -73, -81, -85, -85, -82, -77, -67, -56, -43, -28, -12,   6,  23,  37,  51,  64,  73,  78,  79,  76,},
{  74,  81,  84,  80,  71,  58,  39,  18,  -5, -29, -50, -67, -79, -86, -88, -84, -75, -60, -41, -19,   5,  30,  52,  72,  88,  95,  94,  86,  73,  55,  33,   9, -16, -38, -59, -76, -87, -92, -91, -83, -71, -54, -34, -12,  12,  34,  53,  69,  82,  88,  89,  85,  75,  62,  44,  25,   4, -16, -35, -52, -65, -74, -78, -76,},
{  77,  83,  82,  73,  58,  37,  11, -17, -44, -68, -84, -93, -91, -81, -62, -38, -10,  19,  44,  66,  81,  90,  91,  81,  63,  38,   7, -23, -49, -70, -82, -88, -86, -75, -57, -35, -10,  16,  42,  63,  78,  86,  87,  80,  65,  44,  19,  -8, -33, -56, -74, -85, -90, -87, -77, -60, -37, -13,  12,  37,  59,  76,  84,  83,},
{  79,  84,  78,  63,  41,  13, -18, -48, -70, -84, -88, -81, -64, -38,  -7,  26,  53,  75,  86,  86,  76,  55,  27,  -7, -39, -65, -82, -88, -82, -64, -40, -11,  20,  50,  73,  87,  89,  80,  60,  33,   1, -30, -58, -79, -89, -88, -78, -58, -32,  -2,  30,  57,  78,  92,  95,  86,  67,  42,  11, -20, -49, -73, -86, -89,},
{  80,  82,  72,  51,  20, -15, -49, -75, -89, -87, -73, -47, -14,  23,  57,  79,  88,  84,  67,  37,   2, -35, -66, -88, -96, -86, -58, -21,  19,  51,  75,  87,  86,  72,  44,   7, -28, -60, -82, -89, -82, -64, -36,  -3,  33,  64,  84,  89,  80,  62,  35,   2, -33, -64, -83, -90, -85, -67, -37,  -3,  32,  62,  78,  83,},
{  86,  87,  70,  40,   0, -39, -73, -92, -92, -72, -37,   5,  44,  74,  87,  84,  63,  30,  -9, -44, -73, -89, -84, -61, -24,  17,  56,  81,  88,  76,  49,  13, -27, -61, -83, -87, -73, -45,  -9,  29,  61,  81,  87,  77,  50,  13, -25, -59, -82, -89, -78, -53, -18,  20,  55,  81,  92,  87,  64,  29, -13, -54, -81, -91,},
{ -84, -83, -60, -22,  24,  64,  89,  92,  74,  39,  -7, -52, -85, -95, -80, -43,   4,  47,  79,  89,  78,  49,   4, -42, -77, -90, -78, -46,  -4,  36,  66,  81,  78,  57,  24, -21, -61, -85, -87, -68, -33,   8,  49,  79,  89,  79,  49,   6, -37, -69, -84, -82, -61, -23,  21,  60,  84,  91,  77,  44,   1, -44, -77, -92,},
{ -81, -75, -48,  -7,  39,  76,  92,  80,  44,  -7, -56, -89, -94, -70, -23,  32,  75,  94,  86,  51,   1, -49, -85, -95, -77, -33,  24,  70,  92,  87,  58,  13, -39, -83,-102, -84, -42,  10,  57,  84,  89,  69,  28, -18, -60, -86, -88, -61, -17,  26,  61,  79,  76,  51,  15, -26, -61, -80, -75, -47,  -7,  29,  59,  74,},
{ -83, -70, -34,  15,  62,  87,  77,  39, -11, -55, -74, -68, -40,  -1,  37,  68,  78,  58,  15, -31, -65, -78, -64, -30,  17,  65,  92,  80,  33, -21, -65, -89, -83, -48,   6,  57,  90,  94,  62,   8, -46, -85, -96, -78, -28,  31,  79, 100,  87,  46,  -8, -58, -91, -94, -66, -17,  39,  83,  97,  81,  35, -24, -70, -92,},
{-102, -76, -21,  38,  80,  87,  64,  22, -30, -71, -82, -61, -18,  32,  69,  84,  67,  22, -32, -74, -89, -66, -10,  51,  83,  77,  42, -11, -59, -85, -75, -33,  25,  76,  92,  67,  14, -44, -83, -88, -57,  -7,  46,  84,  88,  55,  -1, -58, -92, -88, -49,   5,  55,  88,  86,  51,  -1, -57, -95, -90, -51,  10,  64,  89,},
{-108, -82, -17,  56, 103, 103,  53, -21, -80, -97, -71, -16,  47,  90,  89,  45, -21, -73, -88, -65, -13,  47,  85,  80,  39, -19, -75, -92, -64, -10,  47,  86,  85,  47, -17, -77, -92, -63, -10,  44,  80,  81,  45, -10, -64, -88, -68, -16,  41,  76,  76,  43,  -5, -54, -78, -64, -21,  31,  70,  74,  47,   1, -47, -73,},
{ -97, -61,   3,  63,  95,  76,  12, -57, -89, -71, -19,  42,  81,  77,  31, -32, -80, -85, -45,  17,  75,  92,  54,  -7, -64, -91, -66,  -2,  60,  89,  72,  15, -48, -84, -76, -29,  31,  75,  80,  43, -18, -67, -82, -55,   5,  63,  91,  64,   4, -51, -84, -75, -31,  28,  75,  86,  54,  -2, -65,-103, -84, -17,  56, 103,},
{ -85, -47,  16,  69,  82,  43, -21, -71, -76, -38,  24,  74,  80,  36, -29, -78, -85, -40,  31,  80,  83,  38, -28, -75, -80, -34,  37,  83,  76,  25, -39, -85, -78, -26,  40,  85,  77,  22, -47, -87, -72, -18,  50,  91,  74,  16, -51, -91, -80, -23,  51,  96,  89,  31, -44, -93, -90, -38,  38,  97,  95,  30, -44, -94,},
{ -90, -44,  31,  83,  84,  28, -53,-102, -78,   3,  80, 105,  56, -29, -97, -99, -39,  46, 102,  92,  21, -63,-102, -66,   9,  68,  89,  47, -30, -80, -78, -24,  50,  88,  62,  -7, -64, -79, -40,  20,  64,  73,  35, -24, -72, -77, -28,  38,  80,  69,  14, -46, -80, -63,  -1,  61,  82,  47, -21, -72, -76, -28,  32,  74,},
{ -89, -37,  44,  89,  68,  -2, -72, -91, -38,  49,  97,  69,  -7, -82, -95, -33,  51,  94,  71,  -7, -85, -96, -27,  60,  96,  65, -21, -94, -88, -23,  66, 107,  63, -27, -94, -86, -17,  61,  88,  54, -12, -71, -79, -34,  34,  81,  64,   5, -56, -74, -38,  19,  62,  65,  22, -37, -71, -58,   2,  63,  76,  36, -26, -74,},
{ -63, -22,  38,  71,  44, -21, -71, -66,   5,  72,  74,  19, -54, -88, -43,  38,  88,  61, -13, -76, -81, -18,  65,  91,  36, -49,-101, -62,  40, 100,  77,  -8, -91, -98, -18,  75,  95,  41, -40, -85, -66,   5,  69,  76,  29, -38, -82, -63,   5,  71,  86,  36, -38, -90, -77,  -4,  79,  97,  34, -51, -97, -66,  23,  91,},
{ -95, -30,  67, 102,  49, -48,-111, -65,  42, 111,  78, -34,-105, -79,  18, 105,  85, -16, -93, -89,  -2,  91,  87,   7, -77, -91,  -7,  84,  81,  14, -68, -87, -16,  54,  69,  24, -43, -61, -30,  17,  57,  53,   1, -44, -63, -29,  30,  62,  49,  -3, -61, -67, -18,  52,  84,  40, -49, -86, -46,  32,  90,  58, -19, -78,},
{ -42,  11,  31,  24, -13, -39,  -2,  22,  22,   1, -31,  -6,  20,  16,  -4, -35, -19,  32,  34,   2, -27, -44,  13,  46,  16, -20, -54, -20,  63,  58,  -7, -65, -77,  22, 105,  53, -41,-108, -70,  64, 121,  66, -48,-143, -69,  79, 124,  54, -85,-135, -29,  87, 121,  40, -93,-127, -20,  89, 102,  16, -93, -86,   8,  80,},
{  29,   8, -26, -33, -12,  20,  45,  23, -25, -54, -33,  31,  61,  30, -32, -68, -28,  43,  69,  30, -48, -85, -21,  65,  81,  19, -82, -96,   7,  91,  88,   0,-104, -97,  20, 116,  86, -33,-115, -75,  28, 104,  86, -22,-102, -88,   6, 100,  95,   4, -79,-100, -28,  72,  95,  37, -46, -90, -47,  33,  74,  60,  -8, -78,},
{ 153,   2,-129,-123,  10, 138, 101, -47,-134, -75,  69, 125,  51, -70,-118, -30,  90,  94,   7, -86, -97,  15, 101,  68, -33, -94, -45,  55,  73,  27, -47, -81,  -8,  65,  54, -10, -65, -40,  32,  48,  24, -23, -48, -12,  33,  37,   2, -39, -29,  16,  27,  14, -14, -21,   5,  26,   6, -15, -27, -12,  27,  29,  -5, -22,},
{  85, -18, -75, -46,  32,  75,  24, -53, -68,  -2,  69,  52, -27, -75, -34,  58,  73,  -8, -73, -51,  39,  83,  14, -69, -60,  24,  88,  30, -67, -71,   5,  74,  55, -44, -77,   0,  68,  52, -29, -90, -41,  64,  89,  11, -82, -82,  28, 107,  52, -70,-112, -21,  98,  97, -14,-102, -73,  43, 100,  34, -65, -99, -12,  99,},
{  65, -16, -67, -32,  44,  61,  -2, -63, -37,  38,  62,   9, -60, -55,  30,  80,  19, -62, -66,  11,  79,  44, -54, -81,   2,  86,  46, -51, -66, -13,  64,  65, -37, -92,  -8,  90,  57, -53, -91, -10,  80,  77, -27,-105, -36,  79,  90, -23,-109, -43,  73,  97,   0,-110, -61,  65, 101,   8, -98, -74,  50, 109,  25, -97,},
{ -99,  29, 107,  41, -83, -88,  32, 101,  29, -88, -78,  45, 101,  16, -85, -75,  43,  95,  15, -82, -73,  39, 113,  23,-105, -68,  61,  90, -13, -93, -35,  71,  73, -24, -86, -25,  72,  59, -37, -76, -14,  70,  59, -40, -67,   0,  54,  36, -41, -52,  19,  57,  17, -48, -56,  21,  76,  30, -65, -69,  25,  83,  26, -74,},
{ -78,  31,  80,  10, -62, -41,  44,  65, -20, -76, -16,  67,  52, -45, -63,   8,  62,  32, -50, -65,  18,  78,  26, -78, -58,  59,  75, -21, -82, -19,  65,  58, -40, -80,  12,  95,  22, -88, -62,  50,  88,   5, -90, -56,  75,  91, -42,-105, -10,  96,  60, -63, -96,  20, 100,  27, -87, -72,  54,  99,  -5,-112, -45, 100,},
{  92, -42, -99,  -3,  96,  40, -77, -71,  45,  94,  -9,-103, -30,  93,  68, -60, -96,  17,  98,  28, -89, -73,  81,  95, -60, -97,  32,  97,   3, -86, -43,  67,  71, -46, -79,  14,  78,  22, -74, -53,  54,  70, -17, -78, -15,  80,  30, -61, -51,  37,  69, -16, -75,  -8,  62,  31, -42, -50,  23,  69,   2, -81, -32,  69,},
{  71, -54, -74,  35,  83,  -4, -84, -27,  76,  47, -61, -60,  41,  74, -20, -79,  -2,  75,  25, -62, -48,  41,  74, -30, -86,   9,  90,  13, -87, -33,  76,  57, -59, -79,  50,  92, -29,-103,   0, 103,  33, -86, -61,  53,  82, -21, -96,   0,  96,  19, -81, -45,  61,  75, -48, -86,  27,  93,  -2, -98, -24,  93,  55, -84,},
{ -86,  64,  89, -46,-100,  24, 109,  -3,-110, -18, 109,  36, -95, -59,  74,  82, -54, -93,  30,  95,  -6,-102,  -6, 110,   8,-106, -14,  94,  22, -76, -40,  65,  56, -71, -57,  73,  50, -53, -56,  35,  64, -18, -70,   0,  66,  20, -66, -32,  63,  43, -52, -58,  37,  71, -23, -77,   5,  75,  13, -65, -30,  61,  46, -61,},
{  72, -67, -62,  61,  61, -49, -63,  46,  63, -50, -57,  41,  53, -21, -59,  -1,  71,   6, -73, -15,  81,  21, -86, -29,  91,  23, -91, -24,  94,  32, -90, -46,  88,  60, -96, -57,  92,  52, -75, -56,  61,  65, -40, -85,  32,  93, -21, -99,   9,  99,   7,-103, -12,  94,  21, -86, -34,  77,  48, -67, -61,  74,  65, -76,},
{  89, -91, -66,  83,  71, -83, -80,  97,  74,-106, -58,  91,  53, -76, -50,  68,  52, -67, -54,  65,  56, -70, -54,  80,  55, -85, -51,  81,  37, -71, -31,  61,  38, -71, -28,  86,   5, -81,  -3,  74,  13, -61, -26,  53,  22, -53, -15,  46,  28, -46, -39,  44,  56, -52, -68,  57,  75, -68, -70,  77,  62, -76, -65,  74,},
{ -62,  68,  42, -71, -33,  78,  23, -82, -10,  83,   0, -84,   2,  93,  -7, -98,  13,  97,  -5,-106,  18, 108, -58, -82,  93,  43,-100, -17,  97,   9,-101,  -8, 116, -17,-107,  41,  87, -40, -78,  31,  77, -27, -76,  24,  75, -31, -68,  38,  68, -44, -70,  47,  66, -46, -57,  36,  59, -35, -62,  41,  61, -45, -62,  57,},
{ -42,  59,  16, -63,   6,  51, -17, -47,  32,  40, -47, -28,  68,  -4, -63,  34,  43, -39, -44,  44,  48, -74,  -5,  79, -54, -37,  79, -11, -80,  35,  72, -42, -71,  75,  45, -99,  -1,  88, -20, -77,  19,  78, -19, -86,  29, 100, -70, -76,  90,  50, -86, -49,  83,  51, -95, -47, 127,  17,-141,  21, 127, -56, -98,  79,},
{  64, -83, -36,  97,   9, -99,  23,  90, -53, -74,  67,  70, -85, -60, 105,  37,-109, -24, 110,  21,-138,  25, 137, -85, -92, 115,  28,-102,  10,  80, -14, -75,  17,  83, -51, -63,  75,  32, -71, -15,  62,   7, -55, -14,  63,   6, -69,  14,  66, -24, -53,  11,  54, -14, -60,  30,  58, -37, -36,  32,  15, -20, -16,  19,},
{ -33,  51,   4, -57,  13,  54, -25, -59,  54,  48, -87,  -4,  91, -35, -77,  58,  59, -62, -56,  82,  26,-104,  49,  66, -88,   0,  74, -50, -29,  51,  12, -56,   0,  87, -64, -64, 113,   0,-104,  36,  78, -38, -71,  36,  84, -69, -70, 107,  26,-119,  26, 100, -52, -78,  63,  63, -84, -32,  94,  -3, -85,  26,  75, -55,},
{  64, -95,  -8, 103, -40, -84,  80,  43,-103,  12,  90, -50, -64,  84,  16, -91,  25,  81, -57, -67, 104,   1, -99,  61,  57, -97,   9,  84, -63, -50,  84,  23,-103,  36,  64, -75,  -4,  71, -26, -58,  35,  50, -40, -52,  67,  32, -87,   6,  83, -47, -62,  76,  35, -92,   2,  91, -42, -65,  69,  21, -66,  15,  44, -31,},
{ -53,  92, -17, -81,  56,  51, -67, -27,  84, -23, -73,  72,  29, -85,  20,  66, -59, -20,  59,  -7, -54,  36,  35, -67,  24,  51, -85,  14, 102, -93, -50, 125, -42, -86, 118, -19, -94,  73,  46, -70, -29,  74,  11, -93,  47,  79,-100, -26, 109, -33, -71,  53,  32, -51, -15,  64,  -9, -77,  60,  47, -96,  18,  75, -52,},
{  -5,  38, -59,  18,  36, -66,  62,   4, -70,  43,   9,  -2,  -1, -32,  49, -26, -30,  95, -70, -59, 123, -67,  -6,  67, -99,  55,  23, -92, 116, -29, -95, 106, -30, -34,  70, -71,  19,  38, -78,  90,  -9,-103,  91,   6, -49,  50, -52,  21,  36, -85,  91,   2,-117, 101,  12, -97, 106, -21, -84,  85, -13, -44,  60, -29,},
{  42, -81,  23,  79, -91, -13, 102, -64, -47,  91, -13, -79,  40,  69, -76, -30, 111, -59, -75, 113, -17, -80,  86, -11, -73,  70,   9, -71,  49,  38, -71,  -6,  75, -43, -40,  84, -31, -67,  73,  35, -96,  12,  87, -61, -44,  83,  -8, -66,  32,  50, -48, -42,  87, -13, -91,  92,  23,-106,  55,  51, -79,  10,  51, -32,},
{ -56,  94, -23, -62,  42,  39, -53,  -9,  58, -48, -14,  78, -61, -33,  87, -43, -39,  68,  -5, -52,  27,  23, -40,  23,  21, -59,  33,  45, -80,  10,  75, -63, -25,  94, -72, -21,  87, -55, -38,  80, -20, -65,  62,  22, -66,   8,  67, -51, -39,  83, -11, -92,  87,  38,-136,  80,  79,-148,  41, 112,-124,  -5, 106, -65,},
{ -36,  70, -30, -57, 101, -42, -64, 103, -40, -41,  53, -24,   6,  -4,   0,  28, -60,  40,  36, -90,  71,  -3, -78, 112, -56, -56, 140,-108, -44, 155, -84, -79, 151, -83, -35, 102, -75,  -8,  54, -25, -18,  11,  21,  -7, -46,  62,  -2, -71,  78,  -1, -90,  80,  26, -95,  58,  18, -53,  31,   7, -13,   5, -17,  35, -21,},
{ -26,  45, -25,  -8,   9,  29, -39, -10,  68, -77,  10,  95,-117,   5, 118,-120,  16,  87,-108,  31,  66, -93,  53,   2, -41,  53, -41,  18,   1, -23,  46, -46,   4,  63, -95,  60,  30,-121, 108,  31,-149,  98,  53,-132,  75,  47,-109,  60,  31, -71,  32,  26, -35,  -7,  37, -11, -42,  52,  -4, -54,  50,  22, -68,  36,},
{  41, -72,  25,  47, -66,   9,  63, -74,  11,  70, -89,  14,  86,-103,   9, 106,-117,  13,  93, -93,  16,  42, -54,  26,  11, -13,  -8,  10,  19, -36,  17,  20, -57,  70, -31, -55, 121, -85, -39, 135, -86, -64, 131, -41, -76,  91,  -5, -79,  67,  16, -73,  46,  26, -76,  53,  32, -87,  52,  30, -84,  60,  24, -75,  40,},
{ -49,  88, -35, -57,  86, -20, -66,  87, -28, -56,  86, -25, -58,  74, -14, -63,  70,  -1, -57,  42,  22, -67,  51,  17, -79,  81, -26, -53, 103, -63, -42, 105, -82,   5,  66, -94,  65,  14, -71,  49,  23, -56,   8,  52, -39, -30,  76, -52, -38, 112, -81, -37, 120, -93,  -1,  89,-103,  33,  61,-101,  55,  34, -85,  47,},
{  41, -84,  50,  37, -92,  58,  30, -94,  80,   1, -78,  83, -12, -70,  81,  -8, -78,  82,   3, -87,  93, -29, -58, 109, -84,   0,  81,-100,  45,  42, -83,  43,  24, -63,  53,   3, -57,  49,  12, -55,  27,  46, -72,   0,  91, -91,   7,  80,-108,  55,  47,-111,  78,  15, -84,  84, -28, -35,  67, -60,  16,  44, -71,  35,},
{ -50, 118,-105,  -2, 105,-110,  19,  85,-121,  66,  37,-106,  85,   6, -90,  99, -37, -42,  77, -47, -13,  49, -44,  12,  20, -24,   6,   4,   4,  -9,   0,   8, -15,  22, -20,  -2,  36, -59,  38,  35, -94,  71,  15, -93, 103, -24, -77, 116, -73, -27, 112,-107,  17,  72, -99,  60,   6, -53,  69, -51,   0,  57, -69,  30,},
{  26, -70,  85, -42, -38,  98, -94,  21,  69,-115,  81,   8, -78,  84, -36, -25,  69, -76,  38,  19, -56,  65, -46,   5,  38, -61,  58, -26, -31,  65, -49,   8,  28, -52,  54, -35,   0,  48, -82,  61,  13, -89, 100, -32, -60, 109, -90,  18,  64,-114,  87,  -1, -69,  88, -75,  35,  18, -68,  97, -76,   1,  83,-109,  51,},
{ -52, 126,-142,  73,  36,-107, 113, -63, -13,  76, -98,  63,  11, -69,  72, -37,  -8,  56, -83,  61,  -8, -36,  58, -68,  56, -17, -33,  75, -80,  36,  27, -72,  84, -60,  13,  39, -84, 101, -58, -29,  92, -85,  17,  53, -76,  50,  -4, -29,  37, -27,   5,   9, -13,  19, -32,  42, -24, -28,  82, -88,  16,  87,-126,  61,},
{ -25,  67, -84,  56,   3, -53,  76, -69,  36,  13, -60,  82, -63,  11,  46, -86,  95, -62, -10,  83,-118, 100, -47, -19,  90,-131, 113, -43, -46, 107,-115,  69,   3, -63,  92, -93,  67, -18, -42,  87, -86,  36,  27, -72,  74, -34, -12,  38, -49,  46, -25,  -5,  37, -59,  58, -19, -33,  66, -73,  39,  19, -54,  50, -21,},
{  30, -77,  95, -70,  14,  38, -62,  63, -47,  12,  30, -60,  72, -59,  24,  23, -65,  82, -64,  20,  32, -72,  88, -80,  45,  16, -73, 100, -93,  50,  14, -69,  99,-100,  81, -41, -22,  80,-106,  89, -28, -46,  88, -79,  39,  17, -67,  84, -65,  16,  50, -98,  99, -59,   3,  46, -69,  70, -52,   9,  44, -83,  84, -36,},
{ -26,  71, -95,  82, -40, -13,  57, -83,  86, -63,  17,  32, -70,  88, -74,  28,  33, -79,  82, -46,  -4,  45, -64,  67, -65,  54, -25, -11,  50, -83,  83, -49,   1,  39, -62,  79, -88,  69, -26, -29,  80, -97,  68, -16, -35,  72, -88,  68, -21, -30,  75, -98,  86, -51,  -3,  61, -94,  93, -64,   7,  60,-109, 101, -39,},
{  25, -69, 106,-115,  87, -35, -22,  72,-105, 113, -86,  28,  42, -97, 121,-103,  56,   2, -53,  81, -88,  76, -49,   9,  31, -55,  62, -60,  53, -28, -17,  59, -83,  84, -72,  53, -19, -28,  68, -79,  65, -30, -12,  44, -66,  68, -46,   6,  38, -67,  76, -64,  30,  14, -55,  76, -70,  42,   2, -47,  74, -77,  57, -20,},
{ -20,  54, -83,  93, -79,  44,   0, -40,  70, -88,  88, -69,  38,   0, -35,  63, -81,  77, -48,  11,  25, -50,  62, -70,  65, -40,  13,   9, -40,  71, -84,  69, -32,  -8,  39, -63,  84, -93,  81, -49,  -1,  56, -92,  95, -70,  25,  25, -61,  83, -88,  68, -28, -17,  64, -97,  97, -69,  30,  13, -62, 101,-119,  98, -37,},
{ -12,  33, -57,  75, -85,  80, -58,  22,  17, -54,  83, -91,  75, -40,  -4,  47, -83, 103,-100,  77, -43,   7,  27, -54,  77, -93,  88, -66,  34,   7, -51,  82, -95,  95, -87,  75, -52,  18,  16, -46,  68, -73,  54, -17, -21,  54, -78,  84, -74,  51, -17, -24,  58, -79,  79, -57,  21,  16, -48,  78, -99, 103, -79,  30,},
{  17, -52,  88,-112, 119,-112,  91, -61,  27,  11, -53,  93,-115, 113, -95,  63, -20, -26,  66, -97, 112,-120, 121,-105,  78, -47,  12,  21, -55,  84, -93,  81, -59,  38, -21,  -1,  29, -58,  78, -80,  73, -61,  36,  -2, -20,  30, -36,  35, -23,   1,  23, -39,  42, -39,  30, -18,   6,   8, -22,  27, -29,  32, -25,   9,},
{  -9,  26, -42,  54, -60,  61, -58,  52, -47,  39, -22,  -5,  35, -58,  68, -73,  81, -84,  72, -51,  25,   5, -31,  51, -67,  75, -75,  74, -66,  47, -17, -15,  46, -74,  90,-102, 115,-117, 107, -85,  50,  -2, -50,  91,-107, 103, -87,  59, -20, -20,  48, -63,  66, -61,  45, -18, -11,  37, -65,  88, -98,  89, -59,  20,},
{  15, -40,  64, -80,  86, -88,  81, -70,  57, -40,  19,   4, -30,  53, -66,  74, -86, 100,-100,  90, -82,  76, -67,  58, -46,  30, -15,   5,   6, -17,  22, -21,  18, -15,  11,  -8,  -2,  22, -48,  73, -94, 110,-113,  97, -66,  30,   8, -39,  63, -79,  82, -74,  54, -30,   2,  30, -59,  77, -90, 106,-114, 104, -73,  26,},
{   5, -16,  31, -45,  59, -74,  84, -87,  88, -90,  92, -88,  78, -66,  54, -41,  26, -14,   7,  -5,   9, -18,  32, -47,  63, -81,  95,-108, 123,-137, 137,-119,  97, -81,  76, -76,  75, -73,  67, -56,  45, -34,  25, -19,  16, -14,  14, -18,  25, -35,  48, -58,  62, -66,  68, -65,  62, -57,  45, -28,  12,  -1,  -3,   2,},
{  -7,  20, -33,  46, -58,  64, -59,  54, -50,  43, -32,  18,   0, -20,  37, -53,  72, -92, 105,-105,  97, -97, 101, -97,  89, -85,  78, -66,  54, -43,  26,  -5, -14,  28, -37,  51, -68,  82, -89,  92, -88,  78, -60,  31,   5, -41,  73, -93, 102,-103,  97, -81,  56, -30,   3,  24, -45,  58, -63,  61, -56,  52, -39,  15,},
{   2,  -7,  16, -33,  51, -65,  77, -88,  94,-102, 113,-122, 121,-112, 101, -89,  78, -69,  55, -31,   9,   9, -25,  40, -56,  72, -84,  97,-107, 113,-109,  97, -85,  75, -68,  65, -60,  49, -36,  24, -13,   3,   9, -23,  36, -45,  47, -46,  47, -49,  46, -45,  47, -50,  50, -45,  40, -37,  31, -21,   7,   1,  -2,   0,},
{   1,  -4,  10, -15,  20, -23,  22, -22,  25, -32,  44, -50,  46, -41,  33, -26,  20, -15,  10,  -7,   6,  -4,  -2,   6,  -6,   3,   2,  -3,   2,  -3,   6,  -8,  12, -15,  16, -16,  22, -34,  50, -69,  86,-101, 111,-111, 104, -93,  76, -56,  28,   4, -38,  71, -98, 119,-135, 146,-152, 150,-138, 118, -94,  74, -52,  19,},
{  -7,  16, -25,  36, -46,  53, -55,  55, -56,  58, -57,  58, -58,  56, -54,  51, -52,  57, -58,  54, -49,  43, -40,  41, -38,  30, -27,  30, -31,  27, -20,  10,   1, -12,  17, -20,  27, -39,  54, -66,  77, -89, 100,-109, 117,-123, 129,-133, 131,-128, 120,-107,  93, -82,  75, -63,  48, -34,  23, -14,   5,   5,  -8,   4,},
}
};

extern TMatrixCoeff g_aiKLT64HP[KLT_NUM][64][64] =
{
{
{  72,  82,  91,  99, 108, 117, 126, 133, 141, 147, 154, 162, 170, 178, 186, 193, 199, 206, 213, 219, 224, 229, 235, 243, 251, 257, 261, 265, 270, 274, 278, 282, 286, 290, 294, 297, 300, 302, 304, 306, 308, 309, 310, 311, 312, 313, 313, 313, 313, 312, 313, 313, 311, 308, 305, 302, 300, 297, 294, 291, 288, 285, 280, 275,},
{-186,-208,-230,-248,-266,-281,-295,-308,-321,-332,-341,-347,-351,-352,-353,-351,-349,-346,-342,-335,-324,-312,-302,-289,-272,-255,-238,-220,-203,-182,-160,-137,-111, -85, -59, -30,  -3,  19,  42,  65,  86, 106, 127, 149, 169, 187, 203, 220, 234, 246, 257, 266, 276, 284, 292, 297, 300, 301, 302, 300, 298, 292, 283, 274,},
{ 247, 273, 295, 311, 325, 335, 341, 341, 338, 329, 317, 302, 282, 253, 217, 181, 146, 111,  75,  36,  -5, -48, -90,-132,-171,-206,-244,-277,-304,-327,-345,-355,-363,-366,-362,-352,-338,-316,-295,-272,-248,-221,-190,-156,-118, -80, -39,   3,  40,  74, 107, 139, 172, 202, 233, 256, 278, 295, 307, 315, 318, 320, 314, 302,},
{ 279, 309, 330, 343, 348, 342, 327, 302, 271, 229, 181, 128,  71,  13, -47,-104,-157,-203,-244,-280,-311,-334,-349,-353,-347,-327,-301,-267,-230,-184,-134, -76, -16,  43, 102, 161, 214, 258, 296, 328, 350, 361, 365, 356, 338, 311, 282, 243, 201, 151, 101,  49,  -2, -56,-110,-162,-210,-249,-279,-301,-318,-326,-324,-311,},
{ 304, 332, 346, 343, 328, 297, 248, 188, 122,  51, -21, -91,-160,-222,-277,-320,-351,-363,-359,-341,-310,-261,-201,-127, -50,  30, 115, 189, 253, 302, 336, 358, 367, 357, 328, 277, 220, 152,  77,   0, -76,-145,-207,-258,-305,-337,-351,-348,-335,-310,-270,-221,-166,-104, -37,  34, 101, 159, 214, 262, 297, 315, 316, 301,},
{ 299, 319, 320, 301, 267, 216, 144,  63, -25,-114,-197,-261,-309,-338,-346,-335,-302,-243,-168, -80,  17, 113, 201, 278, 340, 370, 368, 341, 295, 228, 147,  55, -40,-129,-213,-286,-338,-364,-367,-344,-300,-236,-155, -66,  32, 128, 211, 285, 338, 367, 374, 356, 318, 257, 181,  99,  18, -67,-146,-213,-268,-305,-320,-309,},
{ 321, 338, 328, 294, 232, 147,  41, -73,-181,-278,-348,-381,-378,-342,-268,-169, -54,  64, 169, 260, 327, 365, 371, 340, 274, 178,  60, -63,-174,-265,-324,-354,-351,-309,-238,-145, -43,  59, 162, 247, 305, 333, 336, 310, 251, 169,  70, -35,-134,-219,-289,-330,-347,-334,-296,-229,-143, -51,  43, 141, 226, 291, 326, 325,},
{ 311, 321, 293, 235, 146,  37, -85,-199,-280,-329,-344,-315,-247,-150, -26, 105, 217, 305, 349, 349, 310, 227, 115, -23,-154,-258,-323,-348,-333,-265,-169, -53,  74, 192, 288, 349, 360, 326, 250, 141,  12,-114,-230,-316,-356,-351,-312,-232,-128,  -8, 118, 224, 307, 367, 384, 349, 274, 168,  42, -88,-203,-297,-353,-364,},
{ 320, 322, 281, 199,  79, -63,-198,-299,-353,-349,-292,-192, -62,  80, 215, 314, 361, 353, 287, 167,  24,-126,-255,-348,-384,-348,-245,-100,  55, 182, 280, 337, 343, 291, 183,  40,-101,-229,-319,-349,-324,-256,-148, -16, 131, 256, 335, 358, 326, 251, 139,   3,-137,-263,-339,-363,-341,-266,-150, -12, 130, 254, 322, 339,},
{ 349, 346, 275, 155,  -4,-162,-291,-367,-372,-296,-155,  10, 166, 289, 349, 342, 264, 127, -27,-169,-289,-352,-332,-243,-103,  63, 220, 322, 351, 309, 206,  63, -94,-233,-330,-352,-301,-197, -55,  99, 230, 315, 346, 314, 215,  69, -82,-217,-314,-348,-309,-212, -76,  77, 215, 318, 366, 352, 268, 121, -56,-228,-337,-373,},
{-332,-317,-224, -78,  94, 249, 347, 363, 296, 161, -17,-197,-326,-376,-325,-186,  -8, 165, 295, 348, 318, 213,  43,-137,-285,-352,-318,-204, -43, 115, 241, 318, 320, 245, 113, -63,-224,-329,-356,-294,-159,  11, 187, 321, 370, 334, 213,  37,-143,-282,-347,-343,-255, -94,  85, 245, 341, 370, 316, 185,   9,-179,-310,-369,},
{-341,-301,-181, -19, 163, 306, 370, 327, 184, -24,-223,-351,-376,-290,-109, 111, 285, 371, 349, 214,  14,-184,-323,-366,-312,-153,  66, 249, 346, 345, 250,  92,-104,-285,-379,-343,-211, -20, 168, 299, 348, 299, 159, -16,-198,-327,-367,-287,-126,  54, 222, 325, 336, 254, 102, -80,-235,-333,-328,-216, -47, 117, 247, 313,},
{-347,-282,-129,  58, 236, 336, 316, 185,  -5,-189,-286,-285,-194, -46, 116, 252, 311, 261, 105, -86,-242,-321,-280,-156,  30, 234, 358, 332, 178, -31,-217,-336,-348,-232, -25, 185, 338, 381, 284,  85,-139,-315,-385,-339,-162,  78, 286, 397, 360, 212,   3,-207,-346,-377,-280, -92, 129, 317, 394, 339, 160, -91,-286,-378,},
{-344,-255, -78, 117, 274, 323, 254,  99, -97,-264,-317,-246, -90, 100, 265, 344, 289, 110,-119,-305,-372,-282, -62, 180, 330, 341, 214,  -2,-214,-343,-339,-198,  40, 273, 389, 338, 142,-119,-321,-385,-290, -99, 130, 314, 377, 283,  60,-193,-360,-366,-228, -14, 193, 342, 350, 217,  14,-210,-356,-346,-207,  27, 239, 338,},
{-406,-295, -60, 199, 381, 391, 224, -41,-274,-363,-294,-111, 118, 301, 353, 237,   3,-221,-331,-290,-129, 105, 293, 336, 231,  26,-231,-373,-328,-144, 107, 316, 377, 275,  20,-256,-379,-314,-120, 120, 306, 350, 234,  23,-213,-355,-322,-124, 126, 300, 333, 215,  10,-203,-320,-283,-118, 102, 282, 323, 224,  22,-196,-321,},
{-422,-268,   1, 265, 411, 348,  97,-195,-362,-332,-141, 105, 292, 339, 209, -37,-266,-358,-259, -25, 234, 365, 268,  55,-170,-332,-321,-112, 158, 329, 329, 159, -94,-291,-332,-201,  33, 255, 334, 229,   6,-213,-321,-264, -47, 197, 343, 280,  75,-145,-309,-313,-168,  64, 277, 348, 237,  12,-238,-400,-339, -66, 215, 387,},
{-336,-187,  52, 258, 332, 201, -41,-253,-310,-204,  25, 253, 338, 218, -25,-264,-365,-238,  37, 277, 363, 239, -35,-271,-351,-210,  70, 294, 336, 181, -77,-311,-356,-184,  99, 335, 355, 145,-150,-337,-318,-123, 155, 347, 317, 108,-159,-353,-341,-128, 173, 368, 368, 151,-160,-366,-355,-145, 149, 365, 355, 106,-163,-344,},
{-344,-174,  97, 297, 324, 141,-144,-349,-322, -62, 234, 381, 272,  -5,-297,-400,-254,  75, 361, 405, 187,-169,-402,-335, -62, 215, 391, 291, -25,-292,-379,-208, 126, 360, 325,  66,-209,-349,-241,   9, 232, 324, 205, -36,-270,-341,-152, 129, 314, 280,  67,-177,-317,-251,  -2, 249, 322, 185, -82,-294,-305,-112, 127, 294,},
{-385,-162, 174, 378, 323,  32,-288,-417,-234, 153, 413, 367,  67,-303,-449,-250, 120, 392, 381,  74,-298,-437,-201, 175, 375, 318,  -1,-328,-362,-154, 183, 383, 276, -30,-300,-335,-117, 177, 314, 228,  -7,-243,-298,-148, 116, 309, 246,  15,-220,-283,-135,  86, 237, 235,  69,-140,-241,-197,   2, 208, 251, 120, -87,-238,},
{-247, -90, 136, 281, 197, -53,-269,-283, -27, 267, 317, 118,-177,-355,-218, 106, 338, 277,  -3,-276,-343,-111, 234, 365, 164,-159,-382,-268, 126, 370, 316,   7,-335,-396, -95, 272, 377, 180,-145,-337,-273,  12, 277, 319, 123,-166,-345,-255,  30, 302, 356, 134,-172,-372,-308,   5, 340, 386, 115,-213,-389,-258, 100, 360,},
{-361,-120, 241, 396, 220,-149,-422,-297, 101, 414, 353, -58,-374,-349, -19, 362, 379,  14,-326,-374, -84, 312, 366,  90,-256,-370, -93, 287, 336, 111,-222,-347,-118, 162, 284, 147,-135,-257,-167,  32, 235, 250,  36,-184,-281,-133, 133, 277, 206, -36,-267,-280, -58, 251, 354, 139,-221,-364,-191, 141, 388, 251, -83,-330,},
{-198,  21, 148, 154,  -9,-189, -95,  39, 135, 112, -63, -94, -41,  17,  66,  -4, -47,  26,   9, -27,  -3, -34,  67,  71, -47, -86, -98,  22, 214, 108,-122,-217,-164, 166, 354,  88,-213,-349,-170, 280, 417, 160,-240,-510,-173, 381, 461,  97,-408,-520, -30, 444, 499,  66,-480,-542,  -1, 488, 455,  10,-481,-438,  49, 448,},
{  71,  59, -65,-114, -75,  14, 178, 148, -48,-214,-211,  77, 266, 172, -75,-293,-193, 153, 305, 170,-143,-391,-134, 257, 343, 134,-324,-458,  18, 376, 379,  36,-456,-421, 138, 493, 330,-200,-530,-247, 212, 474, 309,-231,-479,-254, 166, 454, 253,-179,-342,-240,  76, 322, 209, -67,-181,-186, -22, 152, 117,  66, -12,-143,},
{ 595,   9,-496,-504, -22, 530, 461,-112,-519,-385, 189, 506, 280,-187,-472,-223, 284, 393, 117,-284,-429,  -5, 393, 300, -86,-382,-237, 194, 322, 159,-164,-355, -84, 255, 234,  -2,-242,-193,  93, 187, 126, -45,-184, -90,  87, 142,  52,-114,-135,  33,  98,  69, -21, -82, -22,  81,  28, -25, -69, -59,  79, 109, -24, -77,},
{ 313, -57,-279,-200,  95, 301, 131,-183,-298, -41, 276, 224, -73,-290,-163, 192, 290,   8,-256,-210, 102, 314,  94,-242,-251,  43, 331, 169,-235,-295, -28, 259, 267, -98,-315, -91, 210, 265,   4,-329,-241, 158, 345, 146,-268,-370,  26, 395, 282,-203,-459,-147, 347, 427,  -8,-435,-330, 173, 428, 171,-273,-449, -57, 442,},
{ 288, -52,-291,-176, 154, 291,  43,-259,-212, 114, 293, 103,-220,-282,  34, 334, 176,-214,-321, -20, 324, 259,-189,-388, -60, 353, 255,-157,-297,-107, 244, 293, -87,-369,-101, 316, 265,-144,-357,-112, 281, 342, -56,-401,-184, 264, 370, -48,-407,-185, 238, 367,  41,-404,-257, 227, 371,  56,-339,-288, 172, 401,  85,-347,},
{-372,  80, 406, 208,-293,-374,  76, 397, 164,-316,-350, 121, 417, 130,-306,-340, 119, 388,  97,-321,-320, 128, 474, 116,-428,-289, 257, 381, -59,-388,-149, 281, 316, -73,-373,-130, 307, 263,-150,-328, -68, 290, 256,-170,-288,  14, 223, 131,-157,-195,  70, 197,  62,-154,-201,  57, 260, 120,-210,-249,  75, 291,  98,-260,},
{-207,  67, 231,  63,-179,-159, 114, 224, -35,-246, -81, 207, 208,-111,-230, -35, 198, 154,-138,-238,  11, 260, 164,-248,-271, 163, 306, -11,-312,-126, 232, 263,-119,-334,   5, 373, 138,-343,-294, 185, 390,  47,-375,-278, 287, 427,-163,-464, -55, 424, 274,-285,-417,  98, 430,  95,-362,-294, 228, 419, -27,-488,-188, 432,},
{ 384,-147,-431, -83, 395, 249,-272,-364, 105, 439,  60,-445,-219, 349, 356,-166,-426, -25, 383, 191,-289,-367, 231, 441,-152,-421,  60, 369,  65,-290,-209, 192, 312,-107,-328, -26, 307, 160,-261,-255, 158, 292,  -1,-293,-130, 289, 161,-209,-202, 117, 252, -41,-261, -39, 215, 105,-129,-177,  55, 242,  28,-276,-116, 236,},
{ 186,-117,-211,  29, 226,  80,-206,-173, 159, 221, -94,-237,  18, 239,  57,-204,-131, 170, 181,-105,-236,   2, 328,  46,-364,-111, 363, 181,-338,-228, 261, 305,-165,-399, 140, 430, -66,-442, -61, 417, 201,-339,-306, 202, 369, -60,-426,   0, 425,  73,-359,-193, 261, 334,-212,-385, 133, 426, -30,-426, -93, 398, 230,-361,},
{-336, 214, 394,-106,-442, -14, 461, 122,-428,-229, 410, 281,-316,-360, 199, 423, -92,-444, -20, 420, 141,-435,-192, 458, 196,-419,-201, 336, 213,-228,-266, 154, 312,-145,-322, 146, 286, -82,-256,  18, 236,  42,-226,-103, 190, 158,-174,-185, 172, 203,-130,-232, 101, 240, -57,-246,   5, 237,  40,-189,-102, 192, 150,-201,},
{ 284,-213,-298, 154, 311, -86,-319,  53, 320, -52,-286,  24, 252,  47,-251,-124, 260, 147,-232,-178, 255, 196,-288,-223, 306, 197,-288,-188, 290, 209,-273,-243, 249, 304,-304,-284, 307, 251,-249,-257, 204, 272,-129,-327, 115, 353,-100,-384,  57, 397,  12,-409, -63, 392, 121,-378,-179, 364, 215,-314,-269, 342, 287,-346,},
{ 354,-313,-339, 259, 380,-232,-425, 280, 408,-325,-339, 265, 305,-187,-283, 138, 291,-127,-288, 112, 302,-158,-302, 226, 314,-277,-290, 285, 244,-265,-231, 219, 267,-247,-232, 309, 146,-311,-117, 292, 125,-236,-146, 190, 128,-199, -84, 173, 122,-171,-163, 154, 241,-181,-292, 217, 303,-260,-268, 285, 236,-279,-248, 276,},
{-250, 234, 220,-221,-232, 243, 226,-277,-195, 311, 145,-307,-163, 359, 146,-406,-115, 413, 151,-459,-114, 533,-106,-456, 313, 283,-403,-120, 408,  45,-406, -24, 428, -43,-410, 105, 369,-111,-336,  95, 321, -96,-300,  86, 308,-135,-274, 167, 248,-185,-245, 198, 209,-195,-158, 166, 134,-146,-121, 137, 112,-111,-140, 134,},
{ -86, 109,  47,-117, -11,  87,   6, -89,  23,  89, -82, -52, 137, -33,-109, 108,  54,-123, -58, 151,  31,-209, 137, 154,-300,  37, 308,-204,-257, 273, 214,-304,-195, 413,  39,-417, 113, 338,-165,-278, 150, 272,-145,-313, 193, 359,-364,-231, 433, 136,-406,-150, 395, 152,-470,-106, 595, -14,-619, 142, 548,-267,-439, 367,},
{-270, 297, 219,-338,-178, 379,  95,-403,  29, 395,-144,-394, 245, 387,-353,-315, 396, 290,-461,-266, 662, -45,-637, 368, 377,-490, -64, 413,-145,-264, 166, 212,-165,-203, 258, 101,-284,   8, 228, -57,-200,  92, 165, -93,-150, 146,  85,-156, -39, 121,   6, -52, -30,  59,  49,-116, -11, 122, -77, -56, 128, -28, -90,  61,},
{ 112,-140, -70, 174,  37,-194,  -2, 235, -92,-250, 241, 132,-307, -10, 329, -93,-295, 136, 300,-227,-236, 399, -66,-338, 270, 140,-334,  92, 251,-206,-170, 280,  71,-411, 261, 274,-463, -11, 407,-105,-319, 110, 310,-130,-343, 283, 249,-404,-107, 454, -47,-438, 163, 394,-290,-287, 402, 113,-400,  31, 320,-100,-260, 188,},
{ 294,-386,-143, 430,  13,-413, 140, 343,-297,-187, 368,  11,-346, 148, 222,-217,-127, 262,  29,-290, 170, 140,-279,  79, 251,-288, -42, 338,-211,-214, 319, 112,-397,  74, 324,-263,-127, 324,  -3,-337,  91, 314,-165,-285, 288, 186,-363, -58, 400,-121,-372, 283, 267,-413, -42, 416,-180,-277, 298,  80,-278,  81, 163,-122,},
{-233, 325,  96,-391,  63, 380,-250,-279, 411,  42,-410, 157, 305,-260,-186, 348,  -5,-349, 184, 256,-342,  22, 270,-266,  25, 229,-300,  29, 375,-287,-233, 397, -17,-390, 374,  62,-412, 219, 275,-345,-122, 387, -44,-361, 236, 233,-334, -65, 330, -80,-269, 190, 143,-239, -12, 256,-136,-200, 263,  54,-267,  96, 143,-112,},
{ -63, 238,-205,-126, 291,-176,   9, 136,-146, -36,  38, 137,  -9,-251, 211,  39,-259, 330, -90,-327, 398,-132, -76, 211,-267, 152,  16,-246, 446,-209,-314, 471,-194,-189, 463,-380, -66, 372,-285,  65, 125,-170,  42, -25, 115,  76,-371, 259, 199,-525, 349, 272,-618, 208, 375,-499, 208, 218,-380, 116, 134,-138,  99, -49,},
{  80, -70,-125, 207, -36,-234, 309,  -8,-331, 246, 103,-241,  30, 190, -81,-213, 244, 118,-383, 121, 288,-376, 173, 210,-468, 234, 248,-493, 283, 250,-454,  34, 344,-229, -77, 239,-109,-190, 167, 251,-359,-110, 449,-214,-237, 416,-145,-245, 252,  40,-136, -47, 169,  -3,-269, 224, 219,-454,  91, 401,-415, -51, 448,-277,},
{-313, 521, -63,-444, 267, 242,-309, -64, 322,-198,-172, 405,-105,-386, 383,  68,-410, 297, 180,-406, 111, 206,-237, 116,  74,-231, 139, 124,-200, -25, 209, -83,-182, 276, -79,-214, 275, -60,-230, 289,  -8,-292, 244,  82,-258,  86, 164,-192, -25, 199, -54,-211, 189, 128,-364, 188, 284,-476,  91, 437,-456, -46, 459,-285,},
{ 135,-214,   3, 268,-279, -35, 351,-286, -90, 285, -96,-113,  42, 139,-135,-127, 345,-178,-221, 387,-203, -72, 293,-333, 100, 221,-428, 285, 250,-562, 194, 401,-572, 220, 274,-470, 222, 197,-337,  98, 204,-208, -41, 154,  18,-197, 110, 137,-275,  98, 285,-375, -15, 410,-372,   0, 364,-365,  14, 280,-261,  24, 140, -83,},
{  90,-128, -37, 180,-148,  -9, 183,-208,  34, 162,-195,  82,  98,-209,  93, 138,-169,  10,  76, -31, -19, -10,  94,-165,  83, 160,-340, 238,  92,-338, 277,  17,-320, 461,-308,-119, 536,-560,  50, 587,-611, -98, 650,-413,-191, 497,-286,-165, 388,-216,-115, 270,-138,-150, 259, -43,-212, 220,  -7,-223, 202,  77,-255, 135,},
{  78,-119,  45,  16,  24,-129, 111,  83,-252, 247, -33,-305, 410, -74,-353, 424,-134,-215, 360,-204, -67, 189,-185, 152, -76, -33, 139,-222, 215, -10,-256, 312,-125,-164, 365,-358, 112, 284,-485, 211, 320,-513, 125, 401,-481,  54, 420,-499,  92, 412,-490,  98, 325,-406, 163, 169,-295, 149,  87,-207, 145,  -7, -84,  53,},
{-226, 349, -39,-302, 251, 139,-395, 229, 169,-438, 318, 169,-506, 276, 248,-524, 266, 232,-437, 183, 242,-432, 277,  88,-380, 383,-146,-160, 355,-263, -49, 273,-260,  95,  87,-194, 182, -56, -77, 106, -40, -41,  23,  46,  -7,-100, 142, -67,-143, 304,-190,-155, 365,-248, -26, 290,-357, 115, 251,-411, 199, 195,-408, 217,},
{ 252,-453, 139, 363,-509, 160, 333,-542, 290, 185,-465, 336,  85,-426, 342, 103,-434, 327,  81,-397, 362, -88,-210, 380,-312,  50, 215,-315, 216,  36,-221, 163,  -9, -73,  77, -25, -26, -10,  78, -41, -89, 179,-111,-125, 326,-247, -32, 271,-339, 171, 155,-373, 256,  37,-242, 288,-154, -78, 237,-251,  88, 183,-322, 164,},
{-227, 493,-332,-159, 497,-388, -43, 450,-519, 221, 231,-500, 356,  68,-406, 410,-140,-175, 330,-232,  -5, 163,-183, 118, -22, -35,  41, -63, 120, -99, -20, 111,-139, 118, -53, -64, 203,-268, 151, 142,-369, 308, -11,-305, 403,-173,-182, 388,-320,   8, 341,-423, 152, 202,-344, 238, -27,-157, 243,-190,   8, 182,-223,  98,},
{  69,-173, 178, -40,-155, 263,-197, -11, 228,-320, 204,  47,-237, 251,-117, -75, 236,-253, 109,  85,-216, 258,-187,   4, 193,-286, 256, -75,-195, 332,-233,   6, 192,-300, 285,-180, -11, 256,-386, 268,  56,-372, 411,-136,-214, 420,-387, 139, 201,-448, 398, -75,-245, 391,-371, 208,  37,-304, 460,-351, -21, 412,-522, 245,},
{-233, 548,-534, 147, 308,-517, 428,-116,-230, 424,-394, 155, 188,-387, 295, -50,-187, 359,-374, 187,  76,-262, 327,-316, 205,  12,-238, 367,-325, 125, 129,-311, 344,-216,  14, 173,-318, 349,-180,-101, 280,-253,  53, 150,-203, 132, -28, -47,  60, -39,  28, -41,  27,  32,-120, 175,-116, -71, 264,-293,  60, 281,-403, 191,},
{-106, 261,-280, 111, 118,-247, 265,-197,  58, 110,-250, 296,-191, -36, 256,-367, 325,-120,-166, 391,-440, 306, -89,-140, 366,-478, 371, -83,-233, 421,-434, 264,  15,-253, 382,-398, 283, -67,-195, 402,-406, 185, 112,-330, 353,-185, -23, 171,-249, 233,-104, -83, 227,-271, 211, -25,-193, 317,-308, 130, 111,-225, 197, -82,},
{ 147,-353, 371,-178,-106, 298,-318, 222, -70,-115, 264,-314, 257,-108, -68, 223,-314, 280,-138, -51, 224,-332, 346,-258,  65, 184,-359, 384,-297, 121, 110,-288, 346,-312, 247,-118,-103, 303,-389, 326,-105,-177, 351,-331, 182,  44,-258, 350,-284,  67, 212,-408, 402,-224,  -3, 183,-267, 286,-244,  80, 163,-365, 387,-167,},
{-110, 278,-311, 176,  25,-186, 268,-274, 207, -74, -93, 231,-290, 258,-134, -60, 258,-353, 267, -64,-140, 280,-320, 288,-223, 124,   6,-124, 246,-336, 286,-120, -73, 227,-311, 364,-361, 233, -19,-212, 394,-426, 288, -58,-166, 330,-400, 307, -85,-142, 324,-405, 338,-177, -50, 292,-412, 372,-220, -14, 259,-416, 368,-139,},
{ 139,-362, 477,-406, 180,  74,-276, 401,-442, 373,-185, -91, 360,-519, 518,-335,  51, 213,-369, 386,-311, 208, -97, -52, 191,-246, 225,-192, 137, -12,-159, 278,-298, 230,-143,  61,  57,-199, 295,-291, 193, -31,-126, 226,-275, 248,-136, -24, 189,-289, 299,-223,  70, 115,-266, 325,-265, 129,  45,-203, 277,-279, 212, -78,},
{-112, 295,-395, 364,-222,  25, 171,-312, 384,-383, 301,-156, -13, 174,-272, 304,-277, 184, -49, -77, 163,-211, 225,-216, 154, -36, -40,  79,-164, 253,-266, 193, -46,-109, 225,-315, 364,-354, 284,-151, -41, 253,-396, 396,-280,  81, 130,-271, 350,-350, 255, -95, -80, 251,-375, 391,-284, 125,  43,-238, 408,-487, 403,-154,},
{ -67, 174,-258, 293,-271, 201, -80, -69, 206,-314, 379,-345, 216, -27,-161, 304,-395, 411,-348, 225, -84, -57, 192,-299, 374,-407, 353,-235,  85, 101,-282, 389,-396, 347,-287, 221,-107, -42, 169,-260, 303,-280, 171,  -7,-131, 233,-306, 317,-258, 154, -23,-122, 243,-312, 303,-209,  66,  81,-214, 319,-382, 392,-306, 116,},
{  76,-217, 331,-373, 339,-258, 145, -19,-108, 221,-315, 369,-332, 226,-107, -34, 214,-378, 479,-520, 498,-454, 405,-310, 185, -58, -63, 170,-269, 332,-310, 213, -81, -40, 128,-222, 326,-420, 469,-435, 351,-249,  98,  72,-167, 195,-201, 176, -99, -25, 149,-225, 239,-215, 164,-106,  56,  16, -97, 140,-155, 159,-117,  40,},
{ -66, 186,-283, 323,-308, 267,-210, 146, -81,  12,  75,-188, 282,-315, 300,-267, 224,-160,  61,  46,-143, 246,-335, 380,-395, 369,-310, 244,-151,  21, 118,-224, 289,-335, 364,-376, 358,-295, 205,-101, -30, 192,-346, 434,-433, 373,-290, 186, -51, -78, 160,-208, 231,-223, 164, -56, -68, 183,-291, 386,-423, 372,-248,  86,},
{  62,-163, 226,-230, 182,-109,  26,  47,-118, 201,-299, 387,-438, 451,-428, 381,-334, 292,-233, 174,-148, 144,-150, 178,-212, 229,-235, 248,-272, 302,-319, 304,-261, 229,-228, 232,-240, 272,-308, 344,-376, 383,-374, 327,-237, 131, -25, -51, 101,-131, 124, -80,   9,  70,-156, 241,-308, 345,-367, 370,-341, 285,-192,  67,},
{  43,-124, 194,-223, 224,-233, 235,-214, 178,-140, 104, -73,  41, -19,  20, -32,  35, -33,  54, -86, 103,-150, 231,-286, 328,-382, 425,-453, 486,-514, 484,-382, 252,-151,  92, -39, -29, 104,-184, 255,-294, 306,-279, 191, -59, -87, 231,-350, 424,-467, 487,-464, 377,-269, 156, -28, -76, 149,-208, 251,-274, 274,-211,  80,},
{ -88, 242,-369, 444,-480, 481,-428, 361,-291, 209,-119,  18,  99,-207, 279,-335, 405,-479, 502,-461, 397,-356, 325,-279, 215,-151,  90, -21, -59, 141,-213, 258,-269, 267,-279, 317,-354, 360,-331, 284,-233, 183,-133,  72,  -2, -58, 104,-126, 119, -95,  56,  -4, -49,  94,-137, 170,-184, 178,-151,  99, -45,  20,  -9,   2,},
{   2, -17,  50, -91, 124,-145, 159,-176, 188,-215, 269,-301, 276,-231, 187,-151, 116, -77,  38,   3, -38,  71,-116, 165,-199, 202,-193, 207,-230, 227,-195, 155,-102,  57, -41,  33,   0, -73, 167,-254, 325,-393, 454,-496, 523,-540, 524,-480, 400,-290, 158, -17,-104, 192,-258, 323,-376, 403,-409, 387,-341, 296,-221,  85,},
{ -23,  61,-107, 166,-215, 252,-286, 313,-315, 307,-310, 320,-308, 269,-238, 216,-183, 151,-111,  45,  25, -74, 108,-143, 189,-239, 277,-316, 350,-371, 369,-337, 296,-269, 254,-251, 239,-219, 206,-206, 207,-196, 176,-136,  81, -28, -21,  72,-147, 222,-285, 349,-397, 430,-442, 446,-445, 423,-380, 303,-201, 130, -88,  34,},
{ -60, 160,-244, 312,-367, 395,-391, 378,-364, 356,-358, 354,-327, 289,-255, 223,-203, 200,-186, 165,-144, 119,-100,  98, -78,  45, -35,  45, -49,  42, -28,   2,  30, -55,  66, -70,  77, -98, 127,-148, 173,-201, 224,-255, 287,-321, 371,-421, 444,-456, 465,-458, 424,-391, 370,-342, 296,-240, 190,-151, 107, -61,  31, -10,},
},

{
{  70,  78,  85,  91,  96, 101, 107, 113, 120, 126, 132, 140, 149, 159, 170, 181, 191, 200, 206, 210, 214, 218, 223, 229, 237, 244, 253, 258, 261, 266, 270, 275, 280, 286, 290, 294, 298, 300, 302, 305, 308, 310, 312, 314, 316, 317, 317, 319, 320, 320, 321, 322, 322, 322, 319, 318, 318, 314, 310, 308, 305, 302, 299, 294,},
{-183,-208,-233,-252,-269,-284,-299,-314,-329,-340,-351,-358,-364,-367,-369,-368,-365,-361,-356,-348,-337,-326,-313,-295,-274,-259,-243,-223,-203,-182,-160,-136,-114, -91, -66, -41, -19,   2,  22,  42,  62,  82, 104, 126, 148, 168, 186, 202, 219, 233, 245, 255, 266, 275, 282, 285, 287, 288, 290, 289, 286, 278, 269, 260,},
{-284,-310,-332,-351,-367,-378,-385,-382,-376,-362,-340,-314,-283,-244,-201,-153,-109, -66, -24,  18,  58,  99, 142, 185, 220, 247, 273, 293, 312, 327, 339, 342, 339, 334, 328, 315, 301, 283, 261, 235, 212, 189, 165, 137, 101,  64,  29,  -8, -43, -77,-109,-139,-169,-197,-224,-245,-262,-277,-287,-291,-292,-292,-289,-281,},
{-300,-330,-353,-365,-366,-354,-331,-298,-254,-199,-140, -78, -14,  55, 128, 196, 251, 294, 330, 354, 368, 372, 365, 347, 319, 282, 239, 187, 133,  76,  15, -50,-109,-165,-215,-256,-291,-314,-332,-343,-344,-336,-325,-304,-279,-251,-220,-181,-140, -96, -50,  -1,  46,  92, 139, 178, 211, 238, 259, 276, 288, 291, 289, 281,},
{-296,-320,-332,-325,-303,-268,-215,-142, -57,  29, 111, 185, 255, 310, 350, 372, 376, 356, 321, 268, 203, 128,  44, -44,-124,-196,-259,-307,-344,-364,-372,-361,-328,-286,-238,-173,-100, -26,  45, 112, 175, 232, 275, 310, 337, 351, 356, 339, 314, 279, 234, 185, 126,  60,  -5, -72,-133,-185,-230,-263,-284,-298,-301,-292,},
{-330,-359,-362,-339,-288,-210,-114,  -6, 109, 212, 294, 349, 377, 373, 338, 277, 201, 112,  18, -82,-182,-268,-335,-373,-386,-361,-295,-217,-136, -49,  37, 128, 213, 279, 325, 354, 355, 333, 296, 236, 166,  88,   3, -83,-167,-237,-288,-324,-344,-343,-323,-285,-236,-172,-100, -31,  40, 109, 171, 218, 255, 280, 291, 286,},
{-345,-371,-353,-294,-199, -73,  67, 200, 302, 367, 389, 370, 309, 213,  95, -27,-138,-231,-299,-343,-354,-320,-258,-167, -58,  63, 193, 292, 351, 375, 364, 318, 245, 149,  46, -61,-169,-251,-312,-344,-345,-318,-264,-187, -99, -10,  73, 153, 226, 280, 321, 339, 327, 291, 236, 161,  74, -13, -94,-172,-239,-283,-302,-301,},
{-396,-391,-331,-227, -86,  75, 228, 339, 389, 385, 327, 216,  75, -70,-202,-299,-355,-355,-306,-212, -85,  49, 167, 272, 336, 352, 315, 229, 117,  -2,-115,-210,-279,-321,-328,-298,-231,-132, -16, 103, 205, 279, 326, 342, 317, 259, 182,  82, -25,-130,-227,-298,-339,-348,-327,-268,-181, -84,  21, 126, 222, 289, 327, 332,},
{-341,-319,-242,-128,  12, 154, 268, 330, 325, 256, 147,   9,-138,-255,-317,-318,-271,-174, -48,  92, 229, 322, 362, 343, 260, 124, -51,-218,-330,-373,-356,-278,-154, -15, 124, 247, 325, 357, 346, 274, 165,  39, -95,-219,-312,-361,-367,-307,-208, -86,  42, 171, 286, 367, 389, 359, 288, 181,  51, -77,-194,-281,-328,-341,},
{-333,-319,-224, -76,  92, 245, 342, 358, 288, 149, -32,-213,-338,-376,-317,-181, -18, 147, 282, 361, 364, 279, 136, -42,-210,-329,-378,-352,-248, -95,  73, 229, 333, 370, 336, 241,  98, -54,-192,-298,-350,-340,-271,-155, -15, 124, 242, 322, 346, 303, 208,  76, -65,-203,-301,-350,-338,-268,-158, -25, 115, 232, 301, 320,},
{-344,-314,-190,  -7, 181, 334, 393, 341, 193, -11,-219,-370,-415,-339,-159,  74, 272, 391, 404, 301, 108,-109,-291,-393,-389,-272, -68, 140, 281, 350, 327, 217,  57,-107,-238,-318,-331,-252,-121,  32, 175, 282, 335, 314, 220,  83, -62,-195,-290,-322,-285,-192, -67,  83, 213, 295, 314, 275, 192,  71, -63,-185,-263,-294,},
{-346,-279,-111,  86, 257, 340, 306, 170, -26,-214,-332,-332,-210, -18, 187, 332, 362, 266,  87,-135,-316,-379,-324,-172,  30, 221, 356, 374, 261,  77,-125,-295,-353,-322,-213, -24, 166, 308, 353, 304, 184,  18,-166,-308,-365,-329,-212, -42, 129, 266, 340, 339, 251,  85, -85,-230,-325,-357,-312,-179,   2, 176, 298, 358,},
{-473,-341, -85, 187, 388, 429, 293,  56,-210,-387,-392,-229,  25, 257, 385, 363, 215, -11,-226,-357,-345,-198,  27, 228, 332, 321, 170, -66,-256,-335,-301,-161,  51, 246, 340, 307, 194,   2,-179,-293,-326,-261,-114,  67, 223, 312, 301, 210,  66, -92,-228,-297,-287,-193, -51, 107, 232, 308, 307, 218,  58,-122,-256,-319,},
{-320,-206, -12, 177, 292, 271, 125, -67,-241,-308,-232, -37, 190, 328, 303, 132, -95,-271,-318,-217, -15, 184, 304, 294, 144, -89,-316,-386,-268, -46, 188, 359, 376, 256,  41,-212,-371,-388,-261, -40, 179, 333, 378, 291,  92,-139,-319,-370,-297,-137,  61, 234, 329, 336, 230,  53,-141,-333,-416,-350,-151, 107, 310, 416,},
{-404,-253,  14, 264, 381, 292,  42,-216,-351,-295, -81, 182, 352, 323, 118,-149,-333,-338,-175,  80, 297, 356, 229, -11,-220,-335,-310, -96, 164, 328, 350, 208, -41,-243,-340,-321,-120, 140, 315, 349, 246,  54,-171,-334,-346,-223,   5, 219, 327, 306, 182,  -6,-195,-317,-306,-187,  21, 258, 376, 338, 176, -58,-253,-360,},
{-370,-193,  87, 301, 340, 175, -92,-280,-295,-146,  80, 271, 306, 152,-101,-298,-329,-168,  84, 284, 324, 189, -53,-268,-327,-205,  59, 298, 340, 200, -49,-284,-361,-249,   3, 264, 366, 278,  68,-165,-327,-340,-200,  43, 274, 387, 315,  67,-194,-351,-352,-202,  36, 271, 370, 315, 113,-168,-364,-398,-268,   3, 256, 402,},
{-373,-156, 143, 332, 323,  97,-192,-355,-280, -21, 256, 372, 229, -75,-317,-349,-166, 122, 327, 320, 120,-145,-322,-294, -96, 190, 390, 289, -21,-269,-350,-232,  31, 275, 343, 218, -24,-271,-358,-256, -13, 234, 369, 316,  73,-209,-379,-330,-130, 114, 308, 365, 239, -23,-267,-361,-271, -24, 227, 342, 271,  53,-161,-298,},
{-432,-140, 221, 407, 314, -22,-352,-420,-173, 230, 454, 310, -60,-364,-382,-119, 204, 368, 284,   7,-269,-369,-214, 100, 326, 313,  68,-231,-331,-204,  35, 261, 330, 187, -93,-300,-322,-118, 141, 303, 272,  82,-168,-317,-269, -70, 177, 319, 273,  88,-161,-317,-298, -96, 169, 330, 280,  60,-174,-298,-250, -52, 138, 258,},
{-297, -95, 166, 311, 204, -96,-300,-239,  15, 276, 313,  71,-218,-317,-145, 140, 304, 240, -17,-264,-321,-147, 181, 372, 258, -58,-373,-367, -57, 261, 399, 253,-102,-418,-375, -16, 283, 371, 201, -86,-316,-334,-119, 164, 350, 330,  63,-260,-394,-277,  16, 277, 390, 241, -73,-340,-349,-143, 127, 328, 305,  89,-127,-286,},
{-310, -73, 236, 327, 125,-194,-349,-179, 169, 371, 226,-134,-367,-252,  94, 344, 274, -28,-287,-324,-110, 230, 373, 185,-164,-390,-249, 105, 350, 321,  14,-324,-378,-143, 230, 425, 225,-117,-345,-335, -63, 265, 379, 211,-116,-359,-322, -40, 251, 364, 217, -68,-305,-327,-129, 147, 321, 271,  13,-230,-277,-141,  77, 245,},
{-242, -55, 220, 254,  90,-166,-332,-120, 201, 338, 172,-259,-390,-131, 223, 437, 177,-285,-414,-218, 208, 509, 191,-275,-423,-196, 309, 441,  69,-249,-369,-153, 297, 376,  99,-217,-346,-124, 179, 230, 167, -33,-214,-141, -81,  64, 219, 149,  11,-142,-283,-145, 110, 297, 309,  47,-341,-384,-118, 210, 445, 239,-136,-350,},
{-501,  83, 427, 370,-120,-554,-220, 265, 458, 166,-388,-350,  20, 304, 295,-153,-346, -59, 147, 262,  96,-293,-175,  69, 167, 170,-144,-254,  88, 162,  98, -33,-300, -68, 278, 208,  46,-291,-393, 108, 357, 292,   3,-468,-325, 157, 341, 308, -81,-444,-197, 110, 296, 261,-123,-338, -89,  89, 196, 156,-176,-174,  44,  95,},
{-291,  -2, 272, 252, -17,-294,-255,  67, 320, 229,-111,-349,-192, 177, 329, 136,-205,-313,-106, 211, 347, 103,-291,-344, -49, 278, 328,  27,-290,-300, -25, 297, 318,   6,-304,-319, -41, 310, 356,  70,-286,-386,-124, 269, 392, 147,-219,-384,-196, 158, 356, 281, -54,-377,-342, -35, 305, 392, 131,-219,-355,-227,  70, 333,},
{-168,  16, 166, 113, -30,-173,-134, 112, 208,  69,-147,-225,  -1, 229, 179, -44,-243,-161,  89, 253, 182,-123,-339,-126, 206, 290,  62,-239,-269,   6, 267, 271,  -6,-351,-310, 116, 372, 245,-109,-371,-250, 102, 358, 290,-113,-398,-289,  99, 436, 340,-118,-445,-353,  96, 475, 373,-105,-483,-373, 110, 480, 370, -48,-454,},
{ 308, -59,-331,-185, 197, 317,  35,-278,-235, 115, 301,  93,-248,-278,  83, 369, 152,-236,-361, -37, 399, 280,-227,-405, -92, 372, 338,-164,-349,-149, 190, 370,  60,-390,-273, 223, 362,  63,-285,-292,  14, 309, 274,-116,-347,-170, 184, 341,  93,-260,-295, -32, 297, 253, -70,-312,-180, 166, 329,  94,-266,-374, -29, 379,},
{ 367,-130,-405,-122, 307, 321,-125,-371,-106, 303, 305,-145,-394, -57, 354, 271,-185,-374, -86, 341, 302,-159,-390, -84, 335, 251,-213,-280,  50, 246, 157,-147,-310, -61, 289, 291,-102,-356,-152, 226, 328,  34,-321,-259, 138, 328, 137,-260,-318,  29, 307, 227,-118,-350, -87, 264, 254, -73,-326,-146, 233, 363,  50,-373,},
{ 388,-192,-399, -36, 364, 226,-264,-313,  68, 368, 120,-359,-242, 230, 332, -33,-375,-166, 281, 328, -85,-394,-127, 335, 275,-207,-278,  36, 238, 157,-162,-296,  17, 315, 151,-254,-269, 113, 313,  67,-238,-252,  82, 343,  84,-262,-266, 111, 355,  99,-287,-286,  50, 361, 206,-255,-355,  10, 344, 225,-179,-401, -87, 370,},
{-314, 177, 351, -44,-323,-121, 269, 234,-181,-306,  72, 334,  48,-316,-167, 268, 274,-131,-324, -24, 348, 175,-317,-332, 232, 388,-134,-382, -32, 353, 245,-212,-422,  23, 454, 131,-332,-281, 174, 362,  39,-328,-253, 202, 342,   4,-330,-178, 203, 282, -32,-255,-163, 172, 300, -20,-339,-190, 265, 283, -84,-339,-115, 309,},
{ 284,-213,-284, 108, 302,  -2,-302, -75, 258, 160,-221,-219, 150, 265, -64,-291, -53, 272, 173,-213,-277, 132, 363, -53,-374,   4, 329,  87,-277,-186, 164, 299, -39,-382, -50, 382, 151,-339,-279, 205, 400,   7,-369,-205, 237, 365,-116,-429, -61, 359, 273,-194,-393,  14, 366, 149,-289,-301, 175, 390, -26,-426,-179, 386,},
{ 347,-249,-355, 147, 379, -34,-421, -60, 417, 116,-377,-147, 323, 214,-282,-305, 216, 362, -83,-404, -25, 429,  80,-437,-128, 413, 174,-340,-235, 233, 326,-109,-430,  63, 419, -43,-313, -53, 249, 164,-190,-226,  75, 260,  42,-281,-109, 250, 175,-173,-234,  46, 249,  73,-218,-162, 161, 255, -84,-340,  -4, 359, 136,-291,},
{-183, 147, 205,-117,-246,  97, 283, -95,-275,  67, 244, -38,-242,  -5, 262,  67,-258,-116, 241, 171,-263,-187, 277, 213,-317,-209, 326, 203,-289,-240, 203, 314, -93,-408,  15, 497, -50,-462, -23, 406, 182,-333,-280, 217, 322,-113,-391,  73, 381,  81,-333,-252, 261, 373,-211,-398, 141, 392, -45,-373, -75, 367, 196,-328,},
{-357, 378, 294,-363,-310, 346, 333,-355,-319, 360, 291,-317,-293, 273, 308,-234,-333, 177, 371,-152,-410, 205, 355,-228,-288, 229, 262,-207,-296, 169, 312,-105,-309,  65, 285, -70,-267, 109, 235, -61,-234, -27, 229, 110,-234,-153, 227, 197,-189,-265, 119, 342, -88,-313,  20, 324,  10,-322, -30, 249, 106,-225,-160, 212,},
{ 259,-314,-138, 297, 130,-290,-159, 340, 107,-334, -59, 280,  47,-246, -29, 253,  12,-259, -17, 270,   0,-263,  -5, 297, -15,-310,  94, 305,-199,-259, 174, 255,-129,-320, 148, 392,-275,-333, 297, 276,-202,-310, 153, 369,-173,-407, 170, 392, -68,-378, -61, 408, 128,-398,-198, 396, 232,-405,-201, 336, 217,-289,-245, 278,},
{-223, 268, 115,-266, -73, 299, -12,-280,  82, 248, -83,-284,  97, 341,-168,-307, 150, 281, -89,-308, 131, 348,-341,-207, 471,  10,-441,  41, 405,  11,-425, -46, 576,-229,-451, 450, 205,-407,-137, 291, 208,-268,-235, 279, 202,-312,-126, 259, 122,-158,-218, 141, 272,-162,-274, 173, 215,-157,-157, 138, 185,-163,-217, 202,},
{-141, 192,  81,-236, -37, 271, -46,-267, 137, 196,-168,-136, 235,  39,-254,  19, 259, -44,-273,  83, 280,-218,-180, 375, -75,-342, 291, 136,-337, -22, 339, -19,-405, 277, 199,-414, 123, 230,-155,-122,  91, 144, -79,-200,  68, 335,-251,-262, 334, 170,-338,-156, 382, 136,-501, -33, 591,-145,-558, 304, 399,-311,-314, 301,},
{-226, 285, 114,-297, -64, 340, -27,-339, 139, 288,-194,-255, 251, 187,-283,-126, 295, 118,-328, -97, 448,-122,-431, 351, 271,-460, -38, 448, -94,-392, 111, 401,-216,-380, 371, 222,-389, -83, 330,  16,-282,   8, 256,  -4,-274,  -2, 318, -34,-315,  49, 298, -13,-344,  71, 352,-139,-322, 188, 232,-174,-176, 141, 184,-168,},
{-223, 384, -43,-333, 135, 280,-182,-263, 289, 155,-381,  96, 293,-238,-171, 219, 158,-195,-221, 296, 131,-465, 251, 288,-440,  56, 294,-257,  -5, 148, -18,-126,   7, 250,-205,-214, 357,  -6,-304, 175, 190,-185,-154,  90, 309,-187,-361, 416, 156,-490,  75, 432,-212,-328, 250, 270,-283,-199, 342,  49,-371, 129, 317,-244,},
{ 182,-318,  51, 262,-146,-182, 187, 129,-268,  -6, 296,-148,-204, 211, 103,-236, -21, 235,  -8,-271, 118, 231,-292, -39, 416,-319,-187, 505,-222,-329, 329, 188,-416, 149, 182,-342, 195, 197,-215,-199, 237, 229,-347,-132, 413, -26,-384, 163, 333,-243,-352, 398, 193,-505, 146, 379,-372,-111, 372,-100,-228, 136, 111, -97,},
{  61,  12,-220, 180,  20,-209, 344,-171,-200, 216, -24,  85,-113,-114, 253,-181, -30, 321,-261,-114, 227,-152, 200, -96,-232, 400,-349, -43, 666,-516,-368, 684,-270,-171, 389,-336,  33, 166,-181, 256,-105,-275, 274, -29, -34, 160,-291,  59, 197,-207, 199, -50,-216, 259,-147, -76, 429,-367,-182, 432,-211, -81, 284,-185,},
{ 201,-274, -94, 368, -83,-411, 412, 141,-504, 171, 315,-290,-131, 321,   5,-367, 170, 358,-380,-207, 529,-189,-286, 446,-196,-279, 526,-250,-293, 413,  76,-419, 152, 264,-296, -16, 256,-147,-198, 297,  73,-340,  85, 241,-150,-142, 132, 143,-214,-105, 330, -48,-262, 169,  75,-160,  86,  49,-135,  93,   0, -68, 102, -60,},
{ 198,-381, 113, 340,-381, -20, 353,-274,-100, 297, -59,-254, 113, 294,-316,-112, 444,-245,-264, 425, -92,-245, 266, -75,-117, 160, -12,-110,  17, 128, -68,-124, 192, -50,-174, 294,-140,-234, 372, -37,-328, 191, 231,-268, -84, 253,  -1,-255,  88, 286,-274,-198, 480,-136,-407, 503, -55,-471, 454, 110,-456, 182, 227,-185,},
{ -51, 126, -57,-146, 284,-201, -69, 320,-284,   9, 174,-190, 109,   8, -55,  87,-188, 178,  72,-315, 305, -87,-202, 409,-331, -33, 394,-520, 247, 303,-499, 116, 367,-544, 271, 268,-526, 196, 276,-284, -92, 285, -45,-222,  59, 316,-255,-243, 495,-106,-431, 395, 109,-402, 229,  22, -97, 137,-138,  15, 100,-113,  95, -45,},
{-200, 395,-155,-281, 313,  90,-366, 169, 255,-380,  13, 411,-334,-199, 488,-148,-352, 369, 111,-422, 168, 234,-318, 138, 131,-307, 195,  71,-224, 120, 123,-198,  51, 131,-175,  88,  45,-174,  85, 176,-196, -84, 267,-116,-161, 214,  13,-182,   8, 208, -42,-260, 259,  83,-404, 313, 140,-487, 277, 326,-545,  45, 501,-330,},
{-241, 434,-162,-290, 391, -19,-435, 492, -51,-468, 553, -64,-509, 527,  45,-566, 449, 137,-513, 320, 171,-454, 332,  27,-318, 328,-183,  20, 156,-198,  63,  96,-133,  53,   9,   1,   7, -60, 100, -58, -66, 130, -50, -86, 178,-147,  -8, 150,-187, 121,  45,-229, 240, -74, -79, 163,-177,  93,  82,-222, 160, 105,-312, 183,},
{ 132,-265, 104, 228,-373, 211, 115,-388, 399,-127,-249, 440,-275,-133, 404,-264, -91, 281,-163,-104, 222,-102, -71, 128, -75, -25,  97, -59, -42,  71,  26,-151, 159, -26,-156, 334,-358,  20, 409,-352,-180, 489,-136,-462, 547,   2,-517, 509, -55,-383, 414, -83,-273, 326,-125, -53, 102, -68,  12, -90, 125, 143,-379, 205,},
{ 158,-379, 289, 115,-420, 329,  21,-321, 340,-100,-189, 303,-127,-192, 314, -80,-248, 273,  17,-270, 300,-141,-142, 373,-355, 123, 143,-327, 296, -15,-242, 279,-145, -33, 182,-292, 287, -67,-249, 317, -20,-252, 137, 157,-175,-107, 307,-185,-158, 388,-237,-188, 474,-381,  21, 365,-470, 234, 147,-407, 295,  96,-349, 195,},
{ -17,  55, -58,   5,  28, -23,  29, -33,  15,  -4, -30,  83, -56, -49, 110, -81, -10,  87, -82,  20,   4, -27, 122,-177,  81,  77,-187, 193, -64, -94, 181,-138, -69, 341,-379,  57, 426,-656, 223, 533,-669, -16, 614,-414,-241, 587,-307,-237, 480,-218,-244, 446,-225,-209, 411,-205,-161, 361,-221,-129, 253, -20,-192, 118,},
{-237, 596,-604, 140, 412,-615, 412,  12,-382, 480,-255,-112, 337,-253, -55, 281,-194, -77, 220,-132, -45, 148,-145,  67,  56,-128,  88, -29,  18,  -5, -43,  88, -79,  22,  45, -90, 121,-118,  17, 183,-266,  98, 151,-303, 276, -62,-165, 273,-262,  69, 244,-396, 239,  51,-298, 392,-242, -40, 276,-387, 200, 259,-514, 259,},
{-134, 330,-360, 162, 136,-348, 404,-273, -14, 322,-442, 270,  75,-359, 394,-151,-208, 442,-362,  12, 305,-415, 343,-155, -69, 243,-337, 349,-193,-115, 343,-323, 120, 112,-241, 305,-304, 153, 124,-343, 274,  37,-286, 277, -45,-225, 350,-265,  -7, 327,-392,  99, 214,-312, 255,-113, -52, 162,-174,  97,  30,-131, 125, -44,},
{  65,-159, 162, -53, -63, 105,-117, 133,-137,  76,  84,-236, 219, -38,-169, 273,-200, -22, 241,-276, 120,  41,-132, 221,-290, 242, -96, -92, 269,-289, 116, 130,-333, 405,-284,  48, 227,-448, 401, -48,-342, 436,-166,-248, 502,-400,  35, 334,-466, 291,  30,-265, 302,-247, 217,-148,  -6, 239,-473, 478,-111,-367, 502,-219,},
{-121, 313,-391, 270, -25,-187, 327,-375, 310,-119,-167, 415,-446, 206, 142,-407, 465,-267,-105, 413,-474, 326,-104,-108, 292,-404, 383,-200, -73, 286,-333, 197,  47,-244, 295,-229, 118, -21, -47, 113,-145, 100,  10,-155, 244,-177,  17, 100,-138, 134, -78, -50, 193,-296, 279, -74,-176, 349,-423, 284,  48,-343, 379,-157,},
{ 146,-375, 477,-372, 104, 140,-268, 317,-327, 264,-101,-107, 288,-346, 240, -36,-178, 327,-327, 162,  74,-293, 428,-432, 309,-117,-109, 339,-465, 404,-174,-139, 382,-448, 377,-221,  24, 141,-251, 293,-184, -49, 245,-306, 222, -13,-193, 267,-180, -28, 246,-347, 288,-145, -12, 124,-161, 192,-195,  63, 119,-221, 205, -85,},
{ -37, 121,-211, 243,-179,  54,  65,-152, 203,-205, 143, -50, -71, 216,-270, 177,  11,-175, 215,-141,  21,  86,-142, 177,-227, 273,-270, 182,   9,-236, 360,-312, 147,  12,-136, 306,-454, 457,-298,  10, 312,-483, 398,-108,-226, 449,-456, 259,  26,-255, 375,-400, 327,-175, -83, 346,-447, 399,-254,  14, 235,-397, 359,-136,},
{ 120,-333, 488,-509, 370,-141, -84, 274,-426, 501,-441, 238,  29,-273, 438,-471, 331, -56,-229, 397,-446, 426,-324, 160,  13,-137, 224,-330, 410,-365, 162,  91,-276, 358,-360, 324,-252, 121,  40,-165, 214,-167,  46,  87,-168, 176,-108,  19,  39, -69,  82, -84,  61, -21, -49, 144,-184, 135, -42, -39,  74, -60,  21,   0,},
{ -22,  39, -34,  17,  16, -52,  65, -62,  60, -49,  44, -60,  76, -71,  39,  28,-104, 135, -92,  21,  56,-120, 142,-143, 112, -57,  39, -45,   7,  71,-132, 146, -92,   5,  88,-221, 385,-520, 529,-356,  36, 317,-521, 506,-350, 137,  61,-223, 378,-463, 378,-147,-117, 354,-516, 532,-417, 240,   1,-255, 446,-565, 475,-175,},
{  79,-214, 335,-409, 400,-324, 212, -84, -67, 226,-349, 415,-389, 247, -44,-171, 359,-460, 422,-277, 110,  73,-270, 417,-476, 472,-432, 349,-204, -15, 237,-364, 388,-351, 300,-249, 167, -36,-107, 214,-260, 220, -92, -71, 182,-213, 184,-133,  82,  -6, -92, 173,-215, 226,-178,  75,  16, -69, 107,-141, 177,-213, 189, -75,},
{ 102,-299, 487,-617, 634,-573, 504,-434, 328,-174, -17, 203,-344, 408,-374, 266,-106, -77, 234,-318, 340,-365, 382,-362, 321,-264, 188,-115,  45,  42,-128, 186,-198, 173,-129,  76, -11, -69, 134,-172, 178,-142,  51,  65,-137, 151,-131, 101, -32, -83, 194,-261, 277,-261, 209,-140,  89, -39, -15,  55, -99, 142,-128,  48,},
{ -37,  93,-130, 160,-189, 215,-250, 290,-331, 362,-354, 302,-203,  77,  32,-113, 185,-230, 220,-157,  67,  31,-113, 160,-196, 233,-263, 296,-350, 378,-298, 146,  -2, -96, 150,-204, 261,-328, 386,-376, 263, -58,-192, 402,-487, 437,-332, 225, -71,-102, 212,-250, 264,-267, 218,-122,  -8, 143,-264, 378,-471, 488,-361, 129,},
{  28, -86, 148,-210, 263,-305, 344,-384, 414,-437, 466,-481, 457,-378, 246, -90, -66, 199,-292, 340,-347, 335,-310, 272,-219, 152, -72, -18, 128,-267, 381,-417, 402,-373, 345,-317, 284,-246, 189,-113,  27,  50,-109, 149,-154, 123, -84,  45,  29,-124, 186,-207, 212,-218, 200,-145,  78,  -9, -77, 195,-296, 308,-221,  80,},
{ -29,  80,-138, 198,-228, 230,-223, 220,-217, 203,-174, 142, -86,   7,  67,-139, 222,-302, 337,-327, 311,-312, 330,-345, 350,-370, 407,-472, 590,-713, 697,-554, 411,-323, 266,-206, 134, -67,   4,  60,-104, 118,-111,  88, -36, -16,  58,-103, 133,-153, 174,-185, 172,-151, 126, -96,  67, -25, -20,  48, -61,  68, -55,  21,},
{  12, -29,  42, -39,  14,   9, -25,  48, -83, 129,-200, 289,-374, 442,-488, 506,-496, 471,-404, 302,-213, 144, -91,  55, -22,  -8,  25, -43,  83,-127, 128, -92,  47,  -5, -28,  77,-154, 256,-361, 444,-481, 453,-371, 240, -63, -94, 181,-229, 248,-220, 152, -63, -29, 108,-180, 254,-319, 350,-381, 428,-434, 378,-258,  91,},
{  -5,   7,   6, -20,  23, -33,  66,-103, 138,-185, 254,-323, 374,-411, 435,-458, 483,-484, 418,-316, 220,-142,  88, -49,  11,  32, -72, 118,-169, 214,-235, 228,-220, 232,-253, 271,-275, 291,-329, 348,-327, 277,-199,  79,  92,-248, 345,-423, 463,-435, 350,-245, 139, -51, -28, 107,-166, 201,-238, 276,-277, 248,-180,  66,},
{  16, -45,  63, -72,  74, -66,  48, -39,  45, -62,  80, -81,  70, -66,  55, -46,  66,-100, 122,-137, 143,-133, 123,-116,  88, -61,  61, -72,  71, -74,  76, -67,  77,-100, 109,-112, 119,-160, 249,-358, 434,-454, 416,-343, 264,-185, 100,  -9,-107, 245,-363, 429,-460, 497,-522, 535,-553, 554,-543, 504,-424, 331,-222,  82,},
{ -16,  24, -26,  50, -78,  91, -91,  96,-108, 114,-118, 129,-145, 155,-159, 158,-151, 149,-138, 124,-126, 130,-129, 118, -86,  54, -45,  60, -61,  42, -25,  14, -18,   7,  35, -77, 126,-208, 297,-363, 422,-474, 503,-524, 542,-553, 578,-636, 648,-576, 461,-348, 252,-202, 184,-158, 131,-103,  76, -51,  19,  19, -38,  20,},
},

{
{  43,  57,  69,  79,  87,  96, 102, 106, 114, 122, 130, 138, 147, 155, 161, 166, 172, 179, 184, 189, 196, 206, 217, 228, 235, 240, 246, 251, 258, 264, 270, 274, 278, 282, 285, 287, 289, 294, 298, 301, 305, 309, 313, 316, 320, 324, 325, 327, 328, 328, 331, 332, 332, 331, 329, 328, 328, 326, 323, 317, 313, 311, 307, 301,},
{-150,-182,-208,-227,-244,-260,-273,-284,-297,-310,-322,-334,-343,-347,-348,-346,-346,-345,-346,-344,-335,-326,-323,-314,-299,-287,-276,-260,-243,-226,-208,-189,-167,-147,-127,-100, -71, -45, -20,   7,  34,  59,  87, 114, 139, 159, 178, 198, 216, 230, 244, 257, 268, 277, 287, 294, 298, 301, 303, 305, 305, 301, 293, 280,},
{ 175, 213, 247, 274, 296, 314, 326, 333, 339, 340, 341, 340, 329, 304, 273, 240, 206, 172, 138, 102,  59,  12, -32, -76,-119,-155,-191,-228,-260,-286,-306,-323,-341,-354,-358,-357,-352,-343,-332,-321,-302,-277,-248,-213,-171,-127, -84, -38,   3,  44,  82, 116, 149, 181, 211, 238, 264, 286, 302, 313, 318, 319, 315, 309,},
{ 211, 254, 286, 306, 318, 322, 317, 303, 286, 260, 229, 193, 146,  95,  40, -15, -71,-123,-172,-217,-259,-295,-319,-335,-343,-337,-327,-313,-290,-256,-212,-159,-103, -51,   2,  63, 125, 181, 231, 275, 314, 345, 368, 380, 379, 367, 346, 313, 273, 229, 175, 116,  58,   0, -63,-126,-188,-239,-281,-316,-339,-349,-348,-331,},
{ 295, 355, 393, 411, 413, 393, 345, 281, 203, 119,  33, -49,-133,-217,-290,-345,-386,-403,-402,-389,-360,-313,-253,-178,-101, -21,  59, 129, 190, 240, 277, 304, 323, 328, 314, 282, 239, 189, 128,  62,  -7, -73,-135,-190,-238,-274,-292,-295,-290,-275,-247,-210,-168,-118, -60,   4,  64, 116, 167, 214, 253, 276, 284, 280,},
{ 290, 328, 340, 325, 289, 233, 154,  66, -30,-123,-202,-259,-303,-331,-337,-324,-292,-236,-165, -80,  19, 118, 211, 293, 355, 385, 387, 360, 305, 230, 137,  35, -68,-153,-234,-305,-354,-376,-367,-336,-291,-228,-147, -59,  35, 122, 194, 258, 310, 337, 344, 330, 301, 256, 195, 121,  36, -47,-132,-210,-264,-296,-313,-307,},
{ 268, 298, 295, 266, 220, 149,  56, -38,-129,-214,-278,-315,-317,-288,-227,-148, -47,  53, 145, 225, 288, 330, 340, 314, 254, 167,  56, -58,-159,-239,-293,-326,-334,-315,-269,-201,-115, -22,  89, 197, 281, 344, 383, 379, 329, 249, 147,  32, -84,-196,-301,-371,-406,-405,-368,-295,-193, -83,  35, 152, 267, 349, 388, 383,},
{ 325, 354, 339, 285, 204, 102, -27,-157,-265,-338,-373,-364,-304,-209, -83,  52, 178, 278, 346, 372, 346, 279, 171,  36,-105,-221,-309,-359,-360,-308,-229,-129,  -2, 123, 232, 314, 353, 351, 300, 204,  86, -35,-153,-252,-315,-337,-323,-266,-174, -63,  66, 181, 272, 334, 354, 328, 266, 177,  67, -48,-161,-251,-307,-319,},
{ 286, 306, 283, 220, 111, -23,-159,-269,-339,-349,-304,-217, -89,  64, 207, 304, 340, 334, 276, 166,  30,-108,-231,-325,-378,-359,-265,-126,  27, 161, 272, 346, 365, 337, 253, 112, -39,-182,-297,-357,-363,-314,-218, -89,  67, 208, 302, 343, 332, 289, 196,  68, -71,-203,-302,-354,-361,-309,-195, -49, 111, 242, 322, 347,},
{ 342, 372, 319, 199,  37,-135,-300,-408,-415,-327,-170,  15, 193, 321, 373, 357, 269, 132, -18,-157,-272,-342,-342,-274,-145,  12, 160, 271, 323, 307, 233, 112, -32,-159,-264,-326,-314,-233, -97,  53, 194, 297, 345, 326, 233,  85, -71,-211,-316,-358,-328,-237,-102,  46, 190, 302, 362, 348, 257, 120, -32,-181,-288,-335,},
{-316,-325,-248, -97,  87, 243, 334, 348, 284, 160,  -9,-181,-310,-357,-302,-171,   4, 163, 275, 316, 288, 197,  39,-128,-261,-323,-300,-199, -56,  82, 201, 284, 314, 289, 191,  17,-181,-324,-373,-330,-216, -57, 123, 270, 357, 367, 275, 105, -89,-247,-342,-363,-299,-151,  42, 225, 365, 419, 353, 202,   7,-190,-336,-405,},
{-275,-282,-208, -65, 108, 257, 341, 331, 223,  43,-158,-320,-390,-326,-143,  69, 251, 352, 354, 259,  88,-108,-286,-381,-349,-210, -11, 194, 344, 397, 331, 176, -48,-274,-427,-418,-280, -87, 127, 304, 384, 353, 214,  31,-161,-318,-370,-291,-128,  39, 184, 279, 298, 234, 106, -46,-203,-310,-296,-193, -44, 100, 215, 269,},
{-318,-288,-155,  36, 244, 364, 326, 165, -48,-235,-323,-304,-182, -20, 144, 284, 337, 258,  92, -89,-234,-312,-301,-198,  -4, 215, 380, 378, 223,  12,-184,-328,-379,-315,-128, 118, 333, 430, 348, 138, -94,-278,-368,-341,-178,  39, 235, 361, 361, 226,  24,-176,-321,-353,-265, -97, 107, 275, 339, 298, 144, -69,-245,-323,},
{-487,-382,-117, 160, 346, 379, 276,  99,-120,-291,-330,-248, -78, 117, 253, 311, 266, 107, -98,-267,-339,-275, -70, 174, 307, 303, 209,  35,-173,-334,-335,-194,  31, 245, 339, 284, 112, -98,-276,-343,-244, -58, 150, 308, 337, 220,   9,-208,-365,-364,-204,  16, 227, 362, 347, 202, -13,-233,-379,-352,-196,  35, 252, 356,},
{-477,-386,-111, 229, 453, 456, 230,-103,-352,-414,-293, -52, 225, 393, 357, 152,-137,-334,-361,-239, -19, 224, 372, 335, 142,-118,-321,-358,-243, -30, 181, 320, 322, 187, -48,-280,-353,-254, -47, 160, 311, 320, 164, -64,-280,-341,-222, -20, 179, 291, 263, 127, -42,-208,-277,-204, -37, 133, 233, 200, 100, -14,-139,-196,},
{-345,-217,   4, 220, 338, 263,   0,-273,-355,-220,  14, 239, 340, 240,   8,-214,-352,-292, -92, 139, 336, 364, 175, -79,-299,-372,-226,  31, 258, 360, 268,  33,-216,-338,-292,-107, 134, 302, 339, 183, -85,-295,-346,-220,  50, 298, 388, 249, -34,-258,-347,-271, -85, 129, 291, 320, 194,  -7,-253,-416,-342, -82, 222, 446,},
{-371,-203,  80, 298, 328, 142,-126,-304,-276, -81, 150, 295, 262,  71,-158,-295,-286,-113, 133, 295, 277, 118, -93,-240,-272,-133,  96, 264, 284, 132, -98,-285,-289,-122,  99, 281, 283, 121,-141,-320,-280, -79, 180, 357, 307,  69,-211,-378,-330, -88, 237, 420, 364, 113,-197,-398,-403,-199, 158, 473, 463, 163,-211,-476,},
{-351,-186, 114, 334, 340, 102,-254,-452,-303,  62, 365, 435, 184,-191,-431,-383, -98, 238, 431, 355,  51,-277,-407,-261,  45, 283, 338, 182, -92,-298,-297,-115, 142, 288, 245,  35,-179,-276,-183,  24, 205, 276, 172, -49,-273,-324,-156, 116, 316, 306, 112,-136,-310,-280, -55, 200, 325, 216, -56,-279,-311,-139, 116, 315,},
{-372,-150, 198, 368, 244, -52,-306,-338, -85, 245, 366, 201,-119,-354,-310, -30, 254, 349, 208, -93,-355,-348, -55, 265, 378, 243, -85,-355,-365,-136, 223, 430, 312, -17,-337,-391,-162, 151, 331, 288,  60,-215,-342,-232,  43, 289, 305, 121,-158,-300,-210,   2, 196, 272, 150, -86,-284,-276, -24, 261, 344, 178,-115,-350,},
{-244, -84, 161, 282, 151,-109,-296,-252,  71, 333, 276,   4,-266,-350,-131, 225, 398, 219,-113,-354,-337, -42, 294, 390, 158,-207,-410,-268, 112, 384, 337,  32,-296,-390,-169, 191, 368, 259, -37,-312,-334, -85, 214, 340, 189,-101,-320,-293, -16, 268, 336, 146,-149,-338,-277, -10, 295, 348, 117,-176,-346,-257,  61, 350,},
{-404, -95, 293, 419, 137,-270,-407,-162, 234, 411, 189,-200,-374,-205, 143, 379, 239,-109,-336,-287,  15, 317, 328,  44,-298,-374, -88, 303, 411, 154,-258,-434,-212, 173, 380, 224,-122,-326,-247,  44, 317, 327,  40,-279,-344,-132, 160, 308, 215, -28,-254,-278, -63, 198, 286, 135,-151,-268,-127,  99, 231, 163, -30,-225,},
{-124,  12, 114,  92, -26,-106, -67,   3,  75,  94,  25, -53, -83, -76,   0,  88,  82,   8, -77, -96,  34, 111,  53, -69,-157, -75, 110, 203, 115, -92,-237,-183,  56, 278, 236,-109,-336,-231, 103, 343, 263, -52,-326,-332,  47, 414, 351, -87,-509,-414, 112, 542, 492, -43,-561,-554,  -6, 535, 511, -17,-497,-447,  25, 484,},
{-368, -41, 296, 315,  19,-288,-305,  10, 313, 283, -28,-336,-292,  71, 376, 284,-134,-384,-252, 109, 363, 278,-100,-361,-219, 105, 353, 254,-166,-412,-230, 192, 457, 241,-260,-466,-150, 307, 407, 111,-285,-419,-116, 313, 379,  67,-278,-338, -18, 268, 259,  18,-250,-256,  -1, 230, 202, -29,-158, -92,  32,  90,  31, -61,},
{ 609, -33,-525,-412, 160, 532, 238,-341,-474, -40, 407, 373, -46,-399,-285, 164, 416, 179,-233,-388,-151, 270, 364,  48,-279,-280,  45, 314, 183,-100,-280,-203, 143, 315, 125,-206,-292, -28, 247, 216, -22,-231,-190,  73, 244, 154,-113,-262,-102, 139, 211,  78,-161,-191,  24, 211, 140,-109,-245, -88, 183, 214,  15,-192,},
{ 239, -68,-191, -74, 108, 176,   9,-184,-157,  52, 220, 132,-134,-240, -79, 220, 266, -77,-306,-185, 185, 354,  52,-277,-254,  36, 355, 192,-237,-324, -54, 282, 324, -66,-368,-168, 255, 356,   3,-404,-324, 143, 421, 213,-236,-424, -52, 424, 332,-188,-467,-203, 323, 434,  37,-349,-351,  65, 403, 202,-217,-375, -84, 366,},
{ 186, -62,-186, -70, 155, 182, -45,-235, -88, 187, 215, -29,-262,-175, 170, 328,  44,-281,-267,  50, 315, 192,-183,-336, -49, 331, 278,-150,-334,-158, 246, 398, -64,-450,-179, 337, 397, -82,-434,-222, 246, 423,  35,-426,-255, 229, 396,  33,-409,-247, 214, 381,  61,-360,-258, 175, 359,  88,-315,-295, 140, 352, 104,-305,},
{-509, 202, 523,  80,-425,-326, 225, 464,   1,-466,-240, 294, 420, -25,-411,-264, 234, 394,  34,-367,-273, 181, 436, 105,-394,-313, 177, 378,  31,-343,-192, 235, 319, -62,-315,-116, 222, 225,-109,-280, -38, 250, 180,-140,-204,  14, 151,  88,-143,-130, 103, 183,  -9,-178,-131,  92, 222,  74,-207,-201,  92, 239,  72,-215,},
{-236, 106, 215, -22,-164, -56, 149, 158,-124,-241,   9, 237, 115,-199,-160, 105, 194,  43,-204,-195, 108, 266,  68,-280,-196, 218, 295, -81,-333, -79, 245, 236,-133,-308,  21, 372, 103,-327,-259, 159, 351,  63,-365,-254, 327, 393,-182,-468, -44, 420, 254,-267,-416,  73, 446, 143,-399,-335, 242, 461,  -7,-515,-233, 462,},
{ 382,-168,-365,  16, 330, 102,-286,-217, 198, 296, -60,-341, -96, 328, 251,-237,-392,  63, 421, 147,-380,-323, 303, 414,-174,-432,  11, 412, 162,-314,-287, 176, 353, -45,-357, -92, 288, 222,-232,-314, 162, 337, -26,-318, -96, 283, 174,-185,-261,  88, 297, -20,-282, -67, 218, 152,-149,-211,  87, 303,   5,-345,-154, 303,},
{ 380,-337,-358, 272, 414,-130,-457,   1, 417,  90,-323,-209, 257, 320,-158,-392,  68, 379,  55,-326,-159, 214, 263,-111,-329, -15, 338, 108,-291,-188, 201, 264, -99,-312,  41, 325,  44,-333,-104, 300, 177,-223,-230,  85, 280,  32,-303,-128, 285, 179,-243,-229, 195, 303,-129,-336,  61, 342,  35,-376, -93, 335, 226,-327,},
{-353, 313, 301,-263,-346, 193, 430,-151,-458,  85, 453,  24,-425,-128, 354, 225,-306,-266, 206, 317,-139,-345,  94, 349, -54,-330,  -9, 305,  68,-264,-120, 241, 143,-234,-178, 217, 224,-164,-253, 119, 262, -61,-296,   4, 305,  67,-307,-129, 270, 193,-204,-266, 124, 328, -25,-357, -78, 352, 179,-332,-216, 299, 267,-307,},
{-368, 407, 197,-378,-167, 360, 153,-412, -94, 434,  81,-392,-118, 301, 174,-222,-254, 198, 307,-142,-376,  86, 399, -38,-403,  -5, 398,  80,-407,-128, 358, 186,-343,-209, 329, 218,-285,-240, 245, 222,-205,-231, 133, 303, -82,-345,  39, 354, -21,-318, -26, 317,  30,-260, -41, 223,  62,-172,-157, 155, 215,-182,-241, 236,},
{ 297,-350,-137, 332, 119,-361,-107, 419,  40,-426,   8, 388, -22,-369,  32, 360, -19,-351, -21, 364,  -1,-346,  18, 352,   1,-339, -54, 355,  24,-338,   9, 310,   0,-321, -14, 366, -12,-367,   0, 353,  57,-302,-130, 300, 121,-307, -97, 291, 136,-256,-188, 230, 243,-234,-299, 245, 322,-257,-289, 283, 253,-280,-271, 286,},
{-280, 378,  98,-397,  18, 382,-109,-371, 235, 268,-247,-239, 266, 195,-252,-153, 246, 116,-209,-156, 243, 143,-278, -87, 334,  32,-349, -15, 334,   6,-328, -50, 383,  25,-372,  45, 350, -80,-338,  42, 339,  12,-360, -44, 357,  10,-328,  32, 346, -66,-349,  71, 328, -36,-324,  -9, 386, -49,-420, 134, 415,-207,-402, 323,},
{ 265,-428,  22, 428,-208,-316, 343, 198,-458,  -7, 412, -92,-393, 212, 359,-313,-284, 349, 266,-383,-247, 433, 132,-439,  28, 398,-167,-309, 265, 208,-277,-156, 283,  86,-329,  29, 322, -70,-320, 117, 307,-191,-245, 193, 212,-232,-150, 274,  87,-288, -38, 275,   0,-259,   2, 255, -57,-204, 150, 114,-173, -17, 104, -43,},
{-109, 183,   2,-204,  75, 209,-162,-142, 204,  58,-188, -20, 143,  42,-153, -79, 228,  60,-212, -89, 245,  73,-317,  19, 352,-139,-330, 208, 312,-252,-269, 243, 282,-317,-216, 385, 111,-417, -13, 412, -42,-396,  65, 429,-159,-447, 334, 306,-440,-154, 420, 150,-424,-124, 456,  53,-477,  25, 407, -56,-344, 121, 312,-241,},
{-267, 509,-200,-390, 419, 185,-506,  51, 478,-290,-321, 426, 128,-472,  70, 448,-278,-262, 311, 220,-481,  -3, 488,-238,-286, 302, 134,-297, -38, 262,   3,-280,  81, 249,-180,-140, 200,  74,-190, -66, 221,  24,-280,  90, 251,-194,-156, 243,  83,-243,  -2, 195, -63,-118,  50, 102, -80, -88, 162,  75,-288,  44, 294,-197,},
{ 113,-200,  95, 107,-166, -12, 184,-143, -90, 283, -96,-289, 241, 249,-411, -51, 417,-121,-357, 201, 348,-381,-129, 445,-148,-315, 325,  66,-310,  52, 253, -78,-256, 222, 119,-307,  78, 295,-281,-160, 328,  52,-275, -57, 336, -26,-388, 232, 296,-408, -67, 392, -32,-378,  52, 430,-243,-300, 392,  86,-399,  69, 357,-237,},
{-234, 456,-239,-213, 354, -28,-275, 225,  40,-262, 164, 215,-337,   0, 349,-221,-252, 393,  77,-481, 244, 261,-382,   3, 367,-193,-256, 297,  91,-336,  70, 306,-218,-187, 283,  36,-301, 153, 179,-184,-110, 178, 107,-260,  14, 298,-212,-199, 361, -43,-294, 152, 228,-204,-213, 340,  74,-428, 201, 329,-428, -31, 401,-241,},
{-139, 292,-209, -79, 289,-123,-244, 352, -71,-305, 308, 150,-407,  91, 315,-322, -42, 340,-183,-229, 340, -27,-280, 226, 112,-312, 120, 283,-435,  18, 463,-304,-222, 394, -85,-263, 220,  85,-145,-156, 279, 102,-426, 200, 282,-444,  87, 332,-314, -42, 219,  17,-229,  45, 300,-335, -74, 447,-298,-166, 346, -53,-241, 153,},
{ 169,-307, 168,  90,-189,  95,  20,-122, 136,  21,-144,  60,  78, -65, -51,  96,  -5,-148, 121, 128,-299, 187, 154,-444, 342, 148,-512, 302, 221,-410,  75, 306,-238,-155, 304,  -2,-297, 103, 391,-469, -75, 593,-426,-213, 539,-213,-211, 204,  29, -53,-181, 299, -46,-349, 438, -42,-444, 423,  26,-332, 196, 126,-238, 109,},
{  43, -56,  32, -23,  13, -17,  35, -33,  -7,  52, -65,  -1, 128,-131, -57, 222,-186,  -1, 189,-252, 103,  97,-103,  46, -90, 150, -79,-184, 425,-287,-226, 627,-451,-180, 617,-456,-130, 591,-503,  -7, 426,-366,  26, 154, -73,  39,-207, 225,  79,-434, 471, -24,-525, 533, -16,-387, 315,  -7,-153,  58,  70, -56, -39,  45,},
{ -96, 240,-239, -25, 289,-201,-105, 261,-172, -27, 133, -84,  25, -40,  28,  73,-136, 111, -10,-193, 353,-239,-128, 424,-403,  98, 300,-495, 237, 280,-458,  84, 304,-266,   0, 167, -81,-174, 192, 168,-402, 128, 296,-388, 113, 322,-492, 135, 354,-418,   4, 370,-235,-199, 343, -13,-373, 317, 122,-464, 336, 116,-363, 186,},
{ 108,-243, 222, -51,-142, 172, -39,-138, 194, -16,-189, 126, 157,-283,  14, 390,-434,  -1, 507,-479,  16, 334,-409, 193, 160,-288, 121,  90, -95, -76, 146,  31,-314, 323,  26,-431, 491, -57,-391, 371, -14,-277, 252,   4,-156,  82,  99,-167, -25, 235,-226, -35, 329,-322, -24, 391,-404,  21, 389,-471, 181, 270,-451, 206,},
{-246, 539,-453,  -7, 408,-354, -27, 350,-351,  42, 270,-265, -36, 318,-261,-134, 371,-175,-182, 330,-159,-130, 270,-135,-143, 266,-117,-168, 351,-185,-205, 401,-264, -61, 322,-351, 135, 156,-230,  79,  85, -73, -89, 129,  44,-233, 237,  -1,-349, 444,-120,-333, 481,-204,-203, 402,-254, -70, 297,-274,  64, 169,-275, 140,},
{  95,-257, 303,-126,-137, 236,-124, -86, 197,-115, -58, 152, -85, -61, 112, -24,-112,  79, 140,-280, 201,  40,-292, 370,-191,-145, 425,-389,  14, 378,-403,  47, 344,-463, 216, 231,-492, 268, 197,-405, 186, 236,-404,  78, 376,-435,  88, 327,-502, 284, 186,-478, 328,  88,-385, 301,  20,-217, 212,-100, -40, 122,-100,  30,},
{-111, 357,-490, 298, 119,-432, 361,  16,-356, 420,-128,-313, 464,-154,-290, 455,-260,-109, 343,-222, -83, 274,-209, -74, 290,-205, -45, 224,-176, -30, 165,-147,  15, 122,-168,  82, 101,-252, 199,  79,-356, 346, -29,-330, 424,-133,-276, 458,-292,-121, 453,-348, -71, 351,-300,  18, 225,-232,  77,  69,-144, 131, -31, -20,},
{  48, -96,  27, 136,-251, 143, 149,-342, 263,  -3,-236, 291,-115,-185, 363,-237,-103, 370,-316, -46, 363,-394, 174, 185,-422, 347, -68,-239, 411,-281, -53, 333,-462, 371, -35,-330, 503,-412,  91, 329,-515, 295,  84,-349, 386,-193, -94, 269,-290, 183,  48,-219, 164, -17, -47,  -1,  68,  -7,-169, 255, -85,-181, 271,-127,},
{ -60, 219,-332, 225,  75,-348, 370, -99,-239, 415,-294, -87, 380,-293, -74, 355,-351, 120, 185,-342, 258, -42,-155, 246,-200,  32, 143,-187, 118,   3,-106,  76,  46, -91,  39,  38,-112,  96,  62,-241, 234,  14,-296, 353,-109,-267, 467,-346, -11, 411,-536, 253, 159,-404, 416,-206, -95, 334,-429, 307,  25,-363, 425,-183,},
{ 142,-403, 618,-598, 274, 182,-526, 574,-311,-136, 501,-481,  96, 326,-448, 237, 104,-377, 400,-165,-107, 221,-195, 133, -59, -28, 116,-166, 125, -21, -84, 155,-176, 135, -15,-138, 244,-247,  93, 155,-298, 221,  -7,-160, 205,-157,  61,  26, -93, 129, -92,  34,  10, -55, 127,-178, 110,  91,-289, 305, -50,-302, 414,-190,},
{ -82, 232,-359, 374,-221, -19, 219,-318, 330,-218, -36, 280,-358, 251, -25,-204, 352,-353, 131, 233,-492, 485,-269, -49, 373,-529, 434,-149,-196, 412,-396, 186,  85,-285, 361,-298, 121,  92,-284, 354,-228,   8, 183,-259, 174,  15,-141, 143,-104,  71, -56,  31,  65,-190, 227,-116, -69, 241,-318, 186, 103,-312, 302,-123,},
{  84,-242, 362,-390, 305,-127, -56, 204,-310, 311,-167, -77, 298,-377, 268,  -8,-270, 410,-343, 137,  80,-237, 311,-306, 210,  -1,-233, 357,-347, 204,  35,-273, 438,-481, 396,-196,-108, 400,-522, 417,-150,-148, 319,-297, 172,  14,-192, 261,-221, 113,  39,-196, 267,-201,  62,  84,-184, 212,-178,  68, 103,-231, 224, -93,},
{ -30,  85,-126, 139,-124,  64,  33,-125, 196,-219, 159, -24,-135, 249,-262, 157,  25,-195, 239,-164,  58,  32, -87, 146,-240, 298,-218,  19, 209,-379, 396,-235,   0, 207,-342, 400,-374, 229, -31,-161, 342,-425, 352,-153, -87, 300,-429, 375,-191, -42, 303,-451, 400,-200, -76, 314,-442, 421,-236, -63, 357,-515, 429,-159,},
{  46,-156, 309,-438, 475,-373, 145, 124,-380, 554,-545, 305,  49,-348, 502,-465, 316,-124, -81, 239,-325, 308,-184,  -3, 188,-297, 296,-222, 115,  29,-199, 325,-384, 375,-283, 138,  38,-222, 330,-322, 250,-128,  -3,  97,-170, 207,-176,  72,  57,-146, 189,-163,  73,  35,-128, 173,-170, 118,   5,-157, 255,-252, 169, -58,},
{  32,-106, 222,-366, 476,-479, 371,-202,   9, 197,-380, 458,-416, 252, -26,-189, 362,-426, 345,-188,  41,  52,-125, 220,-267, 201, -77, -44, 171,-299, 359,-292, 152,  -5,-115, 200,-261, 286,-272, 201, -53,-136, 290,-346, 294,-174,  41,  67,-160, 222,-224, 168, -54,-123, 279,-316, 247,-123, -40, 223,-396, 476,-377, 142,},
{ -23,  81,-169, 283,-406, 483,-466, 372,-256, 115,  62,-218, 300,-276, 168, -36, -91, 206,-275, 272,-218, 151, -83,   5, 103,-211, 248,-212, 145, -50, -67, 173,-244, 278,-296, 299,-264, 173, -49, -97, 247,-334, 325,-206,  30, 154,-302, 353,-343, 306,-206,  36, 144,-283, 339,-277, 120,  61,-245, 411,-528, 530,-377, 134,},
{  42,-123, 231,-354, 459,-504, 479,-439, 421,-390, 277, -69,-161, 337,-430, 433,-412, 354,-193, -20, 200,-307, 364,-384, 364,-293, 165, -14,-157, 323,-403, 368,-290, 229,-175, 102,  14,-142, 220,-237, 232,-212, 138, -25, -59, 106,-140, 146,-110,  44,  25, -84, 122,-138, 116, -51, -33, 106,-163, 203,-235, 236,-168,  59,},
{  -9,  25, -39,  57, -79,  91, -95,  95,-105, 125,-133, 112, -55, -14,  71,-148, 266,-363, 367,-305, 237,-165,  76,  25,-141, 226,-264, 273,-256, 194, -68, -96, 267,-410, 495,-556, 605,-603, 559,-492, 405,-272,  76, 124,-270, 352,-353, 263,-138,  11, 103,-176, 202,-197, 155, -79,   0,  89,-208, 292,-305, 255,-147,  43,},
{   2,  10, -42,  92,-159, 223,-264, 290,-337, 425,-505, 521,-488, 428,-336, 206, -72, -30, 113,-156, 146,-108,  60,  18,-121, 223,-300, 366,-423, 459,-446, 365,-283, 249,-245, 230,-214, 227,-265, 303,-312, 307,-304, 283,-237, 174, -91,  27,   4, -25,  33, -15, -43, 131,-211, 251,-287, 311,-281, 220,-173, 144, -96,  33,},
{  25, -62, 100,-138, 173,-196, 197,-191, 194,-204, 214,-217, 194,-159, 119, -61, -25, 125,-191, 224,-278, 326,-310, 247,-166,  51,  72,-158, 221,-261, 247,-185, 146,-136, 115, -81,  25,  54,-153, 246,-344, 459,-523, 487,-390, 256, -86, -92, 251,-352, 393,-370, 274,-141, -19, 196,-339, 418,-458, 481,-482, 416,-266,  90,},
{  17, -45,  76,-121, 159,-172, 159,-142, 125,-100,  71, -42,  -3,  55, -94, 133,-186, 255,-313, 334,-353, 399,-425, 396,-360, 336,-288, 223,-152,  88, -35,  -3,  19, -31,  46, -58,  55, -34,  21,   2, -62, 143,-230, 318,-403, 480,-538, 561,-564, 554,-505, 407,-269, 123,  15,-135, 230,-282, 277,-219, 176,-166, 123, -46,},
{  11, -30,  29,  17, -87, 137,-181, 230,-275, 339,-412, 475,-516, 510,-465, 406,-369, 341,-260, 132, -22, -62, 149,-230, 318,-405, 464,-511, 535,-521, 463,-370, 288,-243, 212,-195, 182,-163, 114, -39, -28,  86,-139, 176,-194, 192,-161, 111, -70,  30,  12, -31,  26, -23,  39, -65,  85, -83,  66, -64,  81, -73,  34,  -8,},
{  -4,   8, -12,  19, -26,  23,  -7,  -8,  19, -26,  28, -42,  68, -88, 107,-132, 171,-222, 255,-256, 242,-235, 229,-196, 139, -88,  42,   2, -29,  49, -70,  93,-123, 138,-140, 152,-170, 189,-209, 241,-283, 333,-382, 389,-356, 304,-230, 142, -14,-137, 277,-414, 548,-645, 667,-618, 557,-491, 393,-287, 197,-136,  83, -28,},
{   0,   5, -14,  10,   7, -27,  45, -62,  87,-125, 162,-185, 209,-242, 268,-281, 323,-387, 407,-401, 409,-421, 421,-402, 369,-332, 291,-257, 218,-178, 143, -91,  25,  23, -43,  58, -85, 119,-153, 175,-182, 195,-224, 265,-294, 306,-316, 333,-370, 412,-431, 424,-413, 409,-385, 330,-266, 210,-152,  94, -46,  14,   0,  -2,},
},

{
{  28,  37,  45,  52,  61,  72,  82,  92, 102, 111, 121, 134, 147, 157, 166, 176, 185, 193, 199, 206, 215, 224, 231, 238, 244, 258, 267, 271, 277, 281, 286, 292, 298, 303, 307, 310, 313, 316, 318, 321, 324, 328, 331, 330, 331, 332, 330, 328, 327, 324, 322, 319, 315, 311, 308, 304, 298, 293, 286, 279, 274, 267, 257, 248,},
{ -45, -64, -84,-100,-122,-144,-166,-186,-204,-224,-244,-262,-280,-298,-310,-318,-327,-333,-342,-348,-345,-343,-338,-333,-327,-309,-298,-288,-272,-257,-238,-215,-196,-174,-152,-126,-100, -75, -48, -21,   5,  34,  66,  93, 128, 162, 185, 210, 232, 253, 274, 292, 308, 321, 335, 345, 355, 360, 363, 363, 359, 349, 335, 321,},
{  72, 110, 150, 186, 218, 250, 281, 305, 325, 342, 356, 363, 364, 358, 348, 331, 310, 286, 258, 225, 190, 149, 106,  66,  24, -24, -68,-107,-146,-186,-223,-260,-288,-313,-334,-349,-358,-357,-353,-345,-331,-313,-289,-258,-218,-180,-143,-103, -61, -17,  28,  71, 111, 149, 189, 226, 265, 297, 314, 329, 337, 334, 326, 312,},
{  82, 130, 180, 223, 261, 293, 318, 336, 345, 346, 335, 310, 277, 237, 191, 137,  81,  23, -33, -90,-149,-202,-250,-290,-319,-346,-360,-359,-349,-327,-296,-255,-206,-151, -95, -36,  24,  88, 149, 205, 254, 294, 327, 349, 356, 356, 349, 329, 301, 259, 209, 151,  93,  29, -36,-103,-167,-226,-273,-312,-343,-358,-357,-341,},
{ 119, 185, 247, 293, 328, 352, 363, 354, 331, 294, 240, 171,  95,  18, -59,-135,-203,-261,-304,-335,-359,-365,-349,-313,-266,-198,-124, -51,  28, 104, 177, 244, 292, 330, 351, 354, 341, 310, 263, 206, 140,  68,  -9, -86,-168,-240,-291,-331,-355,-356,-341,-310,-265,-206,-138, -57,  26, 103, 176, 238, 285, 314, 328, 323,},
{ 130, 200, 264, 311, 338, 343, 324, 289, 234, 157,  69, -23,-115,-201,-271,-318,-345,-347,-333,-296,-227,-139, -42,  54, 145, 237, 305, 344, 359, 352, 322, 268, 197, 110,  17, -80,-171,-249,-310,-348,-362,-351,-315,-255,-167, -61,  41, 141, 229, 297, 345, 364, 358, 326, 268, 184,  89, -14,-110,-194,-263,-311,-330,-323,},
{ 148, 225, 289, 330, 342, 324, 272, 194,  97, -11,-119,-215,-290,-338,-353,-334,-282,-204,-105,   3, 108, 207, 290, 342, 359, 342, 286, 202,  97, -17,-126,-231,-306,-351,-363,-338,-278,-187, -81,  33, 144, 244, 325, 372, 369, 327, 262, 170,  59, -63,-180,-273,-338,-372,-368,-318,-229,-122, -14, 104, 209, 280, 315, 324,},
{ 174, 260, 321, 348, 345, 289, 193,  74, -58,-177,-279,-343,-358,-325,-260,-167, -50,  74, 197, 291, 340, 348, 314, 240, 134,  -5,-130,-233,-309,-341,-334,-285,-192, -75,  46, 161, 260, 331, 355, 329, 255, 143,   8,-117,-239,-326,-360,-346,-288,-188, -60,  73, 198, 304, 369, 380, 335, 243, 121, -18,-152,-265,-333,-353,},
{ 181, 266, 319, 324, 284, 193,  75, -58,-177,-269,-323,-328,-279,-183, -66,  65, 189, 289, 340, 328, 276, 169,  33,-109,-235,-326,-357,-330,-247,-121,  28, 172, 284, 353, 353, 293, 189,  49,-101,-234,-329,-371,-351,-266,-114,  60, 205, 313, 371, 357, 282, 161,   9,-150,-280,-358,-380,-335,-227, -83,  81, 241, 344, 381,},
{ 213, 311, 364, 348, 271, 135, -37,-199,-323,-383,-356,-262,-125,  40, 190, 297, 353, 343, 265, 128, -45,-218,-338,-384,-353,-242, -81,  87, 240, 337, 364, 323, 211,  56,-108,-245,-337,-360,-301,-181, -21, 137, 272, 344, 326, 240, 119, -27,-172,-272,-316,-295,-206, -64,  94, 232, 312, 332, 280, 152, -12,-173,-297,-364,},
{ 235, 340, 373, 320, 195,  16,-176,-325,-399,-374,-254, -63, 145, 311, 389, 363, 253,  85,-109,-267,-354,-353,-268,-120,  54, 229, 333, 341, 264, 127, -36,-198,-306,-345,-303,-175,   1, 175, 297, 340, 300, 184,  17,-163,-287,-322,-287,-180, -31, 128, 256, 325, 304, 195,  39,-134,-274,-342,-320,-217, -54, 131, 282, 372,},
{-244,-337,-341,-252, -85, 112, 280, 368, 346, 219,  15,-192,-332,-363,-292,-146,  53, 233, 366, 379, 255,  55,-148,-291,-353,-320,-174,  19, 205, 321, 348, 276, 113, -90,-262,-351,-332,-205, -31, 139, 274, 333, 289, 168, -33,-223,-324,-326,-232, -70, 113, 260, 332, 309, 195,  14,-190,-341,-367,-278,-110, 100, 281, 387,},
{-256,-340,-322,-191,  11, 213, 345, 342, 221,  32,-174,-322,-342,-231, -54, 139, 286, 340, 276, 116, -86,-269,-353,-309,-165,  61, 266, 360, 327, 177, -37,-240,-345,-332,-201,  -5, 203, 344, 358, 253,  59,-154,-325,-382,-285, -78, 133, 298, 354, 287, 126, -78,-249,-340,-314,-175,  35, 252, 364, 345, 195, -48,-268,-378,},
{-316,-391,-326,-121, 128, 332, 411, 307,  85,-168,-347,-385,-262, -27, 216, 373, 375, 225, -31,-273,-376,-313,-121, 101, 259, 331, 265,  87,-118,-278,-335,-258, -72, 154, 320, 345, 226,   8,-204,-321,-313,-191,  11, 208, 334, 319, 167, -60,-252,-335,-286,-124,  81, 257, 325, 256,  84,-139,-298,-329,-223,  -2, 210, 333,},
{-290,-356,-263, -42, 200, 342, 316, 152, -79,-271,-335,-233, -20, 206, 332, 286, 112,-118,-291,-314,-203,  22, 225, 319, 277,  82,-144,-297,-321,-196,  16, 246, 347, 300, 113,-126,-311,-354,-232,  -2, 241, 365, 318, 122,-157,-349,-369,-214,  32, 261, 381, 321, 119,-143,-344,-381,-227,  47, 289, 386, 306,  72,-206,-399,},
{-252,-290,-195,   6, 190, 287, 243,  64,-149,-281,-259, -98, 127, 290, 301, 148, -91,-287,-349,-217,  48, 288, 364, 242,   1,-269,-393,-286, -30, 233, 376, 336, 111,-167,-361,-360,-173, 118, 336, 379, 221, -70,-338,-418,-222,  93, 316, 378, 242, -15,-262,-376,-301, -65, 203, 356, 307,  99,-156,-342,-322,-100, 161, 350,},
{-361,-403,-207, 116, 370, 406, 199,-124,-366,-394,-172, 168, 402, 392, 145,-205,-427,-393,-101, 234, 399, 327,  73,-210,-351,-296, -78, 189, 350, 292,  70,-209,-339,-281, -44, 220, 344, 264,  41,-212,-342,-274, -23, 255, 335, 198, -10,-220,-299,-192,  24, 219, 293, 183, -39,-220,-256,-143,  51, 213, 231,  94, -87,-225,},
{-299,-310,-115, 130, 301, 274,  46,-199,-304,-208,  26, 256, 303, 159,-101,-306,-299,-100, 194, 357, 249, -34,-276,-327,-161, 138, 340, 297,  61,-209,-361,-264,  22, 284, 348, 185,-101,-334,-318,-103, 170, 360, 302,  35,-312,-436,-215, 130, 383, 374, 126,-215,-398,-303, -22, 257, 367, 236, -47,-300,-368,-172, 140, 366,},
{-437,-402, -72, 303, 448, 253,-147,-408,-343, -27, 309, 409, 193,-155,-404,-307,   4, 304, 372, 155,-161,-358,-270,  23, 258, 316, 169,-109,-314,-286, -38, 232, 322, 185, -96,-292,-274, -55, 201, 308, 202, -58,-282,-310, -66, 231, 320, 180, -87,-289,-286, -76, 188, 316, 209, -49,-268,-292, -91, 184, 323, 210, -77,-307,},
{-376,-331, -12, 329, 389, 130,-254,-395,-197, 153, 387, 285, -79,-382,-338,   3, 316, 365, 115,-235,-373,-180, 152, 351, 246, -78,-329,-286,   3, 269, 323, 114,-183,-325,-195,  77, 271, 255,  42,-194,-273,-129, 122, 267, 196, -21,-232,-280,-114, 176, 339, 237, -53,-327,-351, -78, 271, 391, 180,-163,-382,-289,  64, 370,},
{-248,-188,  53, 266, 227, -13,-272,-263,   7, 261, 277,  32,-237,-308, -85, 249, 348, 128,-262,-401,-132, 272, 411, 191,-208,-458,-283, 163, 448, 341, -72,-415,-352, -17, 340, 378,  67,-291,-363,-118, 220, 367, 195,-176,-353,-191,  81, 284, 247, -10,-262,-297, -60, 254, 316, 107,-182,-313,-181, 102, 311, 253, -31,-308,},
{-166,-124,  49, 192, 131, -78,-200,-124,  96, 213, 109,-113,-201, -87, 114, 188,  66, -95,-222,-122, 153, 279, 129,-143,-308,-178, 135, 328, 218,-118,-343,-251,  89, 373, 301, -76,-378,-317,  30, 347, 345,   7,-358,-374,  41, 439, 352, -72,-440,-385,  42, 435, 410, -42,-439,-403,  38, 427, 356, -22,-375,-385,  -3, 415,},
{-446,-281, 174, 446, 254,-194,-459,-190, 268, 422, 108,-303,-365, -56, 299, 311,  59,-256,-311, -30, 263, 273,  -5,-266,-248,  38, 311, 231, -93,-325,-196, 143, 329, 151,-196,-331,-109, 250, 328,  75,-250,-338, -76, 306, 317, -17,-290,-268,  38, 301, 260, -64,-342,-238, 124, 352, 178,-185,-324,-110, 231, 307,  31,-294,},
{-353,-185, 186, 338, 119,-218,-333, -45, 270, 296, -17,-313,-247,  88, 325, 172,-148,-318, -88, 254, 258, -36,-261,-191, 118, 307, 135,-200,-308, -69, 245, 292, -11,-303,-235, 114, 317, 156,-157,-297,-132, 198, 301,  70,-293,-328,  64, 402, 280,-189,-441,-201, 286, 438,  63,-361,-345,  58, 404, 276,-211,-460,-112, 417,},
{-311,-131, 200, 289,  68,-268,-305,  46, 370, 239,-234,-397, -60, 386, 311,-140,-415,-200, 303, 420,  -1,-431,-320, 191, 457, 136,-326,-354,  50, 348, 256,-141,-369,-154, 251, 327,  25,-308,-252,  91, 317, 159,-199,-280,  -5, 250, 196,-123,-273, -58, 223, 237, -49,-293,-158, 200, 272,  26,-255,-241, 106, 353, 107,-313,},
{ 261,  80,-202,-229,  27, 295, 154,-201,-281,  21, 305, 183,-176,-349,  -8, 331, 215,-168,-368, -68, 355, 329,-145,-439,-154, 371, 345,-139,-393,-153, 284, 358, -63,-381,-212, 235, 360,  18,-332,-264, 138, 385, 117,-333,-262, 126, 333,  89,-289,-251, 104, 310, 127,-269,-276, 116, 325, 117,-254,-321,  82, 387, 126,-332,},
{ 577, 144,-491,-428, 269, 553,  12,-506,-332, 333, 511, -68,-536,-182, 394, 338,-113,-411,-104, 345, 224,-194,-338,  12, 354, 145,-246,-231,  94, 266,  48,-225,-158, 109, 240,  35,-219,-183, 101, 264,  68,-240,-243, 165, 258, -41,-169,-108, 107, 173,   8,-150,-126,  82, 185,  24,-153,-141,  81, 197,   5,-201, -80, 171,},
{-317, -45, 289, 185,-228,-243, 159, 265, -33,-295, -57, 261, 157,-221,-205, 103, 231,  47,-262,-143, 222, 253,-107,-320, -40, 326, 170,-230,-294, 109, 305,  81,-276,-233, 161, 328,  29,-350,-214, 220, 352,  -2,-367,-221, 322, 370,-148,-435,-114, 374, 299,-170,-406, -37, 379, 214,-273,-402, 140, 469,  47,-441,-217, 369,},
{ 460,   1,-451,-147, 394, 237,-315,-295, 209, 344,-110,-368,   1, 389, 103,-298,-245, 193, 321,-112,-303, -32, 302, 169,-257,-259, 169, 295, -48,-307, -84, 278, 185,-187,-274,  35, 323, 131,-278,-277, 145, 331,  24,-346,-120, 326, 185,-217,-277,  93, 321,  35,-316,-159, 226, 247, -94,-352, -15, 397, 138,-379,-257, 351,},
{ 335, -26,-351, -59, 331, 129,-315,-180, 291, 241,-274,-254, 221, 285,-150,-295,  23, 322,  80,-305,-157, 266, 244,-222,-326, 152, 370, -64,-390, -69, 382, 213,-330,-328, 249, 372,-119,-425, -12, 418, 155,-322,-275, 200, 369,-108,-405,   3, 374,  82,-287,-168, 179, 244, -94,-269,  -4, 291,  97,-289,-169, 252, 220,-244,},
{-323,  81, 351, -74,-323,  35, 337,  -8,-354, -36, 375,  49,-346, -63, 274, 118,-218,-209, 213, 237,-167,-311, 153, 363, -93,-429,  39, 431,  76,-415,-179, 388, 248,-318,-345, 251, 411,-180,-407,  55, 409,  59,-395,-132, 359, 204,-307,-261, 224, 293,-134,-289,  64, 292,  -2,-298, -33, 283,  60,-224,-109, 173, 156,-173,},
{ 304, -86,-298,  68, 277, -36,-298,  46, 314, -61,-284,  53, 279, -73,-252,  51, 232, -16,-267, -12, 313,  45,-313,-113, 284, 148,-263,-156, 272, 165,-267,-196, 260, 234,-263,-250, 216, 279,-147,-309,  89, 318, -24,-376,  57, 405, -33,-419, -61, 418, 151,-443,-199, 436, 245,-401,-307, 383, 320,-337,-317, 290, 300,-288,},
{ 381,-141,-364, 150, 369,-201,-383, 298, 343,-325,-318, 321, 269,-297,-237, 235, 275,-208,-271, 193, 264,-189,-281, 181, 318,-266,-253, 273, 234,-263,-218, 242, 191,-216,-180, 212, 174,-175,-192, 140, 223,-124,-227, 118, 204,-121,-210, 107, 261, -98,-280,  61, 351, -90,-392, 103, 432,-110,-461, 174, 392,-222,-300, 256,},
{-296, 144, 276,-196,-245, 254, 213,-307,-154, 337,  85,-381,  -5, 409, -43,-412,  48, 420, -88,-408, 164, 369,-216,-335, 236, 291,-245,-228, 195, 211,-170,-194, 160, 193,-145,-197, 143, 217,-154,-239, 168, 226,-196,-206, 298, 166,-375,-117, 381,  73,-366, -47, 345,  23,-314, -60, 361,  76,-447,  27, 442,-147,-394, 287,},
{ 281,-174,-283, 314, 160,-366, -10, 314, -65,-315, 179, 262,-256,-223, 319, 197,-360,-129, 318,  97,-275, -88, 313,  40,-381, 108, 309,-155,-295, 145, 345,-215,-285, 216, 272,-229,-252, 268, 177,-258,-156, 265, 148,-316, -61, 391,-111,-360, 162, 328,-166,-323, 178, 319,-248,-276, 342, 215,-435, -73, 399, -51,-322, 195,},
{-373, 271, 316,-413,-159, 456, -42,-374, 186, 272,-304,-146, 352,   2,-342, 128, 303,-192,-252, 217, 206,-264,-104, 289, -32,-220,  71, 181, -91,-212, 158, 197,-192,-181, 228, 181,-284,-141, 296, 122,-319, -82, 331,   0,-354, 153, 285,-204,-261, 256, 232,-318,-166, 385,  40,-436, 172, 374,-385,-152, 423, -69,-352, 238,},
{ -40,   6,  78, -45,-122, 151,  89,-233,  24, 238,-174,-155, 293,  14,-331, 140, 293,-214,-288, 293, 243,-433, -51, 469,-164,-372, 319, 180,-365,   4, 343,-129,-295, 198, 248,-281,-159, 326,  80,-341, -44, 368, -45,-389, 259, 299,-424,-112, 440, -57,-399, 217, 289,-329,-147, 397, -58,-371, 285, 198,-355,  19, 267,-164,},
{ 332,-286,-194, 422, -82,-346, 283, 173,-362,  -5, 383,-215,-266, 358,  92,-389,  50, 369,-218,-298, 387, 144,-468,  36, 450,-250,-294, 389, 140,-457,  23, 467,-218,-376, 348, 230,-404, -67, 374, -13,-353,  65, 336,-170,-216, 248,  64,-243,  21, 204, -90,-115, 118,  -8, -60,  72, -24, -72,  96,  37,-141,  45, 101, -76,},
{-317, 283, 169,-388,  72, 325,-264,-121, 324, -99,-236, 264,  57,-342, 159, 286,-334,-110, 371, -67,-317, 256, 146,-338,  63, 298,-269,-112, 359, -84,-326, 254, 129,-275,  40, 244,-188,-174, 287,  83,-337,  49, 339,-258,-166, 333,   4,-357, 135, 348,-333,-182, 434, -82,-388, 365, 105,-430, 219, 233,-325,  14, 241,-150,},
{ 191,-190, -54, 228,-138,-115, 211, -32,-203, 175, 121,-278,  49, 237,-159,-157, 258, -16,-204, 130,  85,-189,  68, 187,-242,  -2, 285,-233,-166, 373, -71,-338, 286, 181,-394,  40, 362,-275,-204, 396,  -6,-358, 184, 232,-345, -16, 390,-242,-264, 432,  24,-513, 357, 304,-604, 148, 441,-448, -12, 346,-244, -88, 272,-146,},
{-389, 409,  97,-479, 281, 212,-444, 187, 270,-394,  51, 345,-340, -56, 407,-275,-215, 431, -71,-385, 323, 141,-407, 181, 251,-415, 120, 327,-362, -86, 393,-119,-289, 246, 123,-302,  64, 246,-185,-138, 261, -51,-182, 199, -14,-197, 194,  38,-200, 100, 152,-240,  16, 260,-234, -66, 280,-196, -59, 262,-208, -81, 295,-165,},
{ 114,-147,  19, 148,-126, -80, 220,-119,-144, 259, -92,-189, 246,  27,-314, 215, 185,-411, 224, 230,-517, 305, 167,-422, 247, 103,-300, 196, 147,-316,  80, 277,-312, -68, 398,-205,-314, 498, -72,-470, 431, 143,-505, 273, 171,-404, 244, 144,-331, 146, 231,-348,  33, 317,-277, -56, 287,-199, -69, 242,-165, -65, 200,-104,},
{-326, 421, -60,-387, 491,-127,-372, 515,-149,-356, 499,-160,-329, 531,-196,-385, 548,-103,-384, 398, -38,-318, 346,  10,-339, 282,  66,-329, 220, 138,-340, 181, 141,-297, 115, 214,-304,  40, 268,-243, -90, 315,-189, -91, 180, -58, -76, 107,  -7,-115, 121,  -6,-100, 127, -58, -76, 148, -86, -52, 133, -92, -27, 108, -60,},
{-170, 222, -48,-205, 290,-109,-154, 248,-123, -76, 172,-103, -56, 204,-176, -59, 243,-183,   4, 113,-149,  80,  42, -96,  88, -69, -12, 129,-111, -51, 138, -30,-115,  91,  67,-158,  64, 144,-199, -37, 254,-167,-151, 378,-281,-125, 505,-433, -95, 568,-474, -90, 551,-517,  50, 480,-602, 195, 363,-562, 268, 229,-476, 248,},
{ 318,-464, 233, 180,-454, 368, -34,-289, 357,-119,-206, 328,-161,-152, 358,-255, -75, 284,-178, -56, 196,-171, -52, 282,-217, -72, 298,-263, -19, 292,-302,   5, 316,-346,  17, 385,-451,  98, 355,-459, 109, 348,-462, 146, 243,-382, 195, 139,-321, 229,  48,-271, 249, -37,-165, 249,-183, -18, 212,-252, 102, 127,-239, 122,},
{-113, 174, -82, -92, 207,-155, -29, 196,-206,  59, 132,-228, 143,  63,-247, 259, -33,-249, 313, -83,-263, 420,-256,-142, 458,-453, 103, 363,-535, 255, 225,-507, 378,  72,-466, 471, -55,-438, 527,-103,-393, 468,-130,-252, 361,-154,-130, 278,-205, -31, 226,-216,  57, 103,-168, 127, -47, -36, 119,-135,  47,  86,-148,  78,},
{-275, 441,-293, -53, 373,-444, 237, 118,-384, 347, -66,-242, 385,-225,-147, 458,-426,  67, 307,-468, 328,  71,-428, 455,-165,-203, 457,-437,  91, 344,-494, 228, 202,-425, 291,  37,-299, 339,-152,-134, 284,-185, -37, 169,-119,  28, -17,  13,  24, -53,  51,  -2, -68,  99, -71,  14,  33, -71,  80, -44, -31,  95, -97,  39,},
{-125, 210,-163,  32,  89,-125,  88,  -4, -81,  95, -32, -37,  97,-121,  83,  29,-165, 222,-145, -34, 225,-280, 133, 104,-251, 285,-203, -53, 342,-371,  94, 258,-432, 276,  90,-358, 360,-155,-119, 298,-294, 126, 109,-323, 398,-150,-294, 523,-358, -70, 450,-480, 151, 254,-461, 397,-116,-237, 432,-355,  38, 350,-499, 236,},
{ 235,-410, 350, -86,-220, 359,-290,  64, 215,-367, 262,  19,-268, 351,-250,  20, 238,-354, 229,  21,-242, 328,-210, -36, 223,-284, 230, -59,-139, 244,-210,  97,  29,-170, 234,-136, -78, 287,-340, 155, 164,-377, 309, -49,-228, 400,-344,  72, 242,-412, 312, -27,-237, 366,-327, 142,  87,-284, 377,-255, -69, 397,-471, 214,},
{ 275,-481, 417,-174,-106, 307,-385, 283,  -7,-272, 386,-305,  59, 230,-410, 376, -93,-262, 466,-427, 155, 165,-348, 365,-222, -11, 241,-368, 295, -80,-143, 306,-346, 180, 106,-312, 329,-227,  45, 179,-311, 268, -81,-146, 315,-330, 165,  55,-203, 233,-131, -41, 176,-228, 184, -51, -98, 210,-256, 148,  81,-270, 281,-119,},
{-181, 337,-331, 189,  45,-264, 408,-421, 243,  62,-352, 491,-407, 146, 166,-405, 461,-308,  31, 230,-393, 402,-257,  31, 209,-385, 396,-211, -75, 318,-399, 303,-103,-122, 290,-326, 200,  13,-212, 329,-290,  67, 206,-347, 299,-133, -61, 211,-251, 183, -42,-103, 180,-177, 117,   2,-133, 213,-195,  69,  89,-184, 172, -69,},
{ 117,-245, 270,-194,  41, 137,-275, 327,-250,  60, 156,-306, 335,-274, 158,  24,-216, 299,-288, 218, -38,-174, 284,-281, 192, -59, -92, 237,-315, 261, -90,-134, 325,-394, 307, -61,-233, 423,-423, 232,  58,-312, 420,-355, 171,  81,-315, 424,-350, 107, 190,-386, 403,-287,  89, 139,-297, 335,-255,  57, 169,-326, 317,-128,},
{-167, 321,-343, 276,-170,  18, 177,-351, 423,-347, 153, 113,-360, 473,-423, 246,  16,-231, 295,-271, 188, -23,-135, 237,-299, 297,-184,   1, 175,-311, 346,-272, 114,  94,-280, 381,-341, 196,  -6,-214, 380,-373, 205,  -4,-125, 236,-348, 321,-149, -77, 273,-359, 317,-180, -38, 247,-316, 257,-160,  29, 147,-303, 293,-118,},
{ 164,-343, 431,-435, 354,-211,   6, 225,-400, 439,-354, 191,  23,-222, 345,-362, 272,-126,   4,  87,-181, 222,-190, 120, -24, -95, 224,-296, 272,-176,  33, 151,-308, 339,-266, 132,  48,-222, 327,-326, 235,-107, -38, 184,-319, 359,-222, -12, 235,-367, 378,-279, 103,  81,-251, 375,-358, 221, -58,-117, 291,-393, 328,-122,},
{-187, 386,-475, 488,-440, 316,-122, -81, 247,-366, 421,-403, 300,-141, -14, 147,-247, 267,-240, 214,-123, -21, 142,-226, 261,-246, 182, -61, -83, 217,-296, 295,-215,  65,  92,-194, 223,-206, 160, -66, -59, 170,-231, 230,-179,  57, 125,-278, 351,-331, 214, -42,-117, 248,-344, 374,-304, 162,  23,-242, 420,-483, 377,-138,},
{-140, 287,-361, 401,-431, 437,-410, 329,-168, -28, 205,-343, 406,-399, 373,-303, 152,  13,-172, 345,-451, 453,-383, 251, -55,-149, 304,-397, 416,-355, 224, -53,-127, 253,-294, 276,-190,  99, -21, -85, 181,-206, 167,-108,  26,  73,-162, 202,-206, 194,-143,  44,  76,-182, 230,-206, 132, -43, -57, 174,-265, 289,-226,  84,},
{  40, -72,  88,-113, 141,-154, 172,-202, 198,-157, 115, -38, -62, 145,-220, 283,-302, 263,-204, 102,  58,-208, 313,-359, 345,-301, 224, -76,-124, 312,-415, 476,-500, 408,-233,  31, 164,-318, 416,-418, 338,-211,  24, 156,-246, 287,-320, 309,-238, 114,  53,-229, 347,-391, 369,-284, 150,  11,-163, 279,-357, 370,-268,  94,},
{ -80, 181,-257, 319,-354, 360,-359, 354,-328, 287,-241, 168, -60, -36, 103,-166, 237,-285, 293,-246, 144, -32, -55, 138,-224, 298,-347, 359,-341, 285,-186,  59,  80,-198, 270,-286, 248,-167,  57,  77,-221, 353,-450, 477,-456, 397,-269, 116,  33,-175, 270,-307, 287,-222, 119,   9,-129, 223,-298, 346,-347, 287,-167,  49,},
{  20, -30,  11,  27, -73, 119,-171, 229,-272, 305,-339, 379,-415, 439,-458, 489,-518, 522,-497, 436,-352, 293,-248, 202,-153, 108, -93,  93, -97, 120,-155, 196,-242, 285,-311, 294,-267, 278,-291, 264,-222, 172,-129,  86, -48,  30, -22,  36, -88, 151,-201, 234,-252, 265,-274, 285,-287, 270,-239, 194,-133,  70, -28,   7,},
{  70,-155, 229,-295, 344,-378, 417,-460, 460,-417, 363,-315, 253,-169,  76,  28,-143, 235,-288, 322,-345, 345,-323, 266,-176,  91, -16, -61, 131,-193, 215,-212, 217,-185, 117, -59,  -2,  99,-212, 296,-342, 360,-342, 278,-183,  68,  67,-196, 292,-340, 339,-309, 257,-187,  86,  47,-151, 214,-278, 337,-350, 302,-195,  64,},
{ -39,  72, -84, 109,-139, 137,-131, 156,-189, 208,-226, 233,-203, 161,-126,  84, -26, -38, 128,-232, 309,-396, 483,-523, 514,-508, 522,-499, 427,-343, 261,-162,  50,  50,-130, 199,-245, 277,-307, 306,-259, 177, -65, -43, 123,-198, 285,-347, 361,-359, 344,-293, 213,-136,  54,  43,-120, 164,-209, 251,-242, 199,-136,  50,},
{   0,  -7,  19, -45,  77, -91, 105,-133, 152,-173, 214,-261, 296,-316, 327,-332, 330,-336, 338,-298, 228,-160,  80,  13, -91, 159,-238, 314,-363, 396,-411, 451,-519, 544,-539, 521,-469, 393,-313, 216, -99, -10,  93,-155, 192,-204, 191,-160, 118, -65,  -2,  70,-113, 140,-166, 198,-223, 230,-225, 210,-184, 153,-102,  33,},
{  17, -28,  38, -51,  54, -60,  80, -95, 102,-109, 111,-102,  93, -85,  83, -94, 110,-111, 109,-128, 151,-167, 182,-191, 177,-163, 169,-174, 170,-180, 190,-205, 226,-226, 221,-222, 223,-219, 207,-184, 146, -92,  27,  36, -79, 117,-185, 269,-338, 390,-431, 469,-495, 517,-537, 557,-558, 539,-513, 470,-388, 277,-168,  58,},
{  24, -42,  54, -70,  78, -79,  87,-100, 105,-101,  90, -84,  86, -79,  77, -81,  86, -95, 115,-133, 134,-134, 158,-198, 203,-178, 174,-175, 160,-124,  71,  -3, -73, 131,-166, 201,-248, 320,-398, 448,-490, 521,-533, 532,-533, 539,-527, 499,-448, 382,-301, 203,-106,  21,  54,-127, 194,-243, 271,-292, 295,-271, 204, -78,},
},

{
{  97, 108, 115, 122, 131, 140, 149, 157, 165, 173, 179, 185, 191, 198, 206, 210, 213, 220, 229, 234, 238, 241, 246, 251, 258, 264, 265, 269, 277, 280, 283, 288, 290, 289, 290, 294, 298, 299, 301, 302, 304, 305, 303, 300, 298, 299, 301, 301, 300, 301, 303, 302, 301, 296, 292, 288, 285, 282, 280, 278, 274, 271, 266, 259,},
{-207,-231,-253,-270,-284,-297,-311,-321,-330,-344,-353,-356,-360,-360,-359,-358,-352,-346,-342,-335,-321,-307,-296,-280,-257,-232,-204,-181,-161,-138,-110, -81, -53, -23,   8,  36,  60,  79,  97, 117, 135, 149, 163, 177, 193, 207, 218, 225, 234, 245, 255, 263, 272, 280, 289, 293, 297, 292, 287, 282, 278, 271, 264, 254,},
{ 220, 241, 257, 269, 279, 288, 294, 295, 294, 288, 280, 266, 246, 223, 198, 168, 131,  94,  60,  23, -17, -63,-107,-152,-200,-242,-287,-320,-343,-364,-380,-388,-394,-393,-384,-367,-348,-322,-292,-266,-233,-194,-154,-114, -79, -46,  -4,  35,  69, 103, 135, 166, 198, 226, 250, 270, 287, 304, 321, 329, 333, 339, 333, 318,},
{ 285, 315, 332, 346, 351, 347, 336, 313, 282, 242, 190, 133,  74,  17, -41, -96,-161,-208,-247,-281,-311,-344,-364,-366,-361,-339,-310,-275,-228,-178,-125, -58,   4,  65, 123, 177, 232, 272, 304, 330, 347, 355, 349, 332, 311, 283, 255, 221, 175, 122,  78,  32, -18, -74,-125,-172,-217,-250,-277,-299,-311,-317,-312,-299,},
{ 293, 333, 348, 345, 327, 299, 252, 193, 132,  65,  -7, -79,-145,-201,-251,-290,-330,-344,-338,-322,-296,-256,-203,-137, -61,  24, 114, 189, 253, 298, 322, 347, 351, 336, 296, 237, 185, 120,  46, -30,-101,-163,-223,-274,-315,-340,-347,-349,-344,-321,-282,-233,-180,-123, -52,  27, 102, 162, 228, 296, 346, 368, 364, 343,},
{ 289, 330, 342, 322, 284, 237, 169,  89,  -5,-103,-203,-278,-330,-357,-364,-358,-329,-263,-188,-105, -14,  83, 177, 274, 355, 388, 385, 356, 305, 227, 132,  31, -76,-174,-257,-318,-353,-364,-355,-318,-261,-190,-104, -15,  73, 151, 221, 279, 320, 343, 349, 329, 286, 232, 161,  85,   7, -71,-136,-194,-246,-272,-284,-278,},
{ 300, 320, 312, 279, 211, 130,  33, -75,-186,-286,-360,-397,-390,-348,-265,-161, -43,  84, 194, 281, 340, 381, 399, 355, 263, 145,   0,-132,-236,-311,-349,-362,-339,-273,-185, -92,   5, 112, 213, 287, 335, 347, 337, 299, 230, 137,  29, -77,-173,-250,-295,-317,-322,-300,-261,-200,-123, -46,  49, 140, 216, 271, 297, 292,},
{ 294, 317, 292, 237, 150,  51, -60,-170,-251,-304,-332,-313,-251,-160, -41,  83, 186, 279, 327, 332, 303, 222, 110, -28,-162,-267,-340,-356,-316,-232,-123,  -2, 138, 259, 349, 385, 352, 280, 177,  59, -79,-203,-300,-357,-371,-346,-290,-194, -77,  48, 175, 266, 336, 378, 382, 331, 248, 152,  29,-100,-209,-306,-361,-368,},
{ 258, 280, 249, 169,  60, -57,-171,-260,-311,-308,-274,-200, -90,  44, 169, 250, 296, 309, 272, 192,  87, -47,-179,-298,-377,-370,-275,-123,  47, 182, 277, 338, 346, 302, 176,   8,-141,-266,-339,-354,-314,-240,-126,  11, 157, 284, 364, 388, 349, 273, 157,  13,-139,-275,-373,-417,-405,-319,-169, -10, 154, 283, 351, 377,},
{ 348, 359, 290, 169,  12,-132,-264,-352,-367,-314,-198, -59,  79, 213, 305, 351, 331, 234,  96, -56,-221,-346,-379,-335,-207, -32, 160, 311, 366, 339, 240, 100, -74,-229,-325,-343,-295,-181, -40, 103, 225, 294, 318, 288, 204,  77, -68,-192,-286,-326,-296,-218,-103,  30, 174, 300, 363, 356, 293, 157, -23,-222,-360,-407,},
{-355,-355,-271,-120,  80, 241, 363, 406, 352, 219,  32,-162,-304,-382,-361,-246, -73, 115, 270, 347, 345, 268, 110, -93,-274,-364,-358,-243, -56, 132, 260, 330, 322, 245, 114, -79,-241,-332,-342,-268,-134,  23, 185, 310, 350, 314, 178,   6,-154,-266,-310,-298,-213, -76,  82, 223, 293, 327, 289, 167,  10,-165,-282,-333,},
{-316,-314,-246, -88, 145, 336, 421, 361, 198, -16,-210,-342,-377,-305,-150,  65, 268, 377, 375, 260,  77,-125,-314,-421,-391,-174, 130, 334, 395, 340, 189, -14,-242,-425,-431,-262, -29, 207, 363, 390, 303, 139, -76,-268,-365,-336,-229, -44, 154, 270, 283, 223, 106, -30,-129,-190,-196,-138, -48,  33,  80,  65,  68,  83,},
{-172,-158, -91,  35, 187, 260, 195,  52, -85,-175,-175,-124, -46,  35,  98, 180, 204, 120, -23,-130,-186,-201,-130, -31, 111, 241, 315, 234,  -3,-205,-315,-343,-246, -42, 190, 326, 367, 303, 128, -94,-291,-408,-407,-284, -23, 254, 448, 496, 383, 158,-105,-343,-483,-468,-308, -30, 259, 430, 468, 355, 123,-124,-319,-407,},
{-445,-335,-107, 126, 326, 366, 315, 183, -21,-224,-340,-337,-212, -22, 168, 342, 406, 308,  95,-153,-375,-442,-282,  22, 242, 311, 281, 115,-102,-261,-281,-172,  24, 228, 308, 206,  33,-131,-241,-256,-164, -22, 144, 269, 301, 193, -22,-223,-345,-320,-168,  37, 219, 338, 340, 206,   9,-232,-426,-400,-210,  63, 288, 378,},
{-446,-358, -80, 237, 438, 438, 248, -70,-323,-398,-300,-100, 133, 326, 370, 241,  12,-205,-330,-321,-174,  73, 283, 345, 227,  29,-219,-338,-281,-127,  91, 296, 361, 254, -27,-287,-344,-243, -71, 120, 267, 288, 214,  45,-172,-309,-297,-157,  69, 252, 317, 237,  63,-168,-302,-282,-131,  82, 274, 342, 238,  22,-211,-342,},
{-409,-256,  26, 267, 401, 341,  82,-193,-339,-318,-141,  74, 232, 314, 243,  19,-226,-356,-274, -47, 212, 353, 262,  85,-172,-348,-310, -92, 181, 336, 322, 125,-134,-320,-323,-146,  83, 263, 292, 170, -22,-197,-280,-207, -25, 166, 314, 255,  75,-129,-322,-341,-178,  74, 303, 391, 279,  40,-271,-471,-386, -80, 247, 435,},
{-254,-168,  19, 181, 265, 187,  -6,-178,-238,-191,  -7, 165, 258, 215,  54,-139,-311,-293, -79, 170, 364, 333,  63,-269,-434,-273, 105, 398, 394, 169,-149,-410,-385,-161, 148, 383, 365, 143,-158,-353,-333,-129, 147, 350, 326, 136,-115,-314,-345,-171,  92, 306, 359, 208, -51,-284,-386,-237,  71, 339, 410, 150,-168,-357,},
{-323,-194,  97, 320, 396, 196,-203,-466,-407, -38, 375, 478, 288, -69,-397,-426,-245,  32, 339, 441, 274, -51,-394,-382,-117, 146, 425, 341,-100,-363,-374,-103, 298, 376, 149,-125,-269,-215, -36,  43, 137, 191, 138,  55,-153,-295,-175,  16, 259, 313,  91,-136,-274,-247,  21, 232, 242,  84,-131,-198,-118, -18,  68, 144,},
{ 294, -24,-134,-171, -18, 101, -74, -33,  75, 164, 192,-190,-376,-203, 149, 523, 332,-249,-481,-324, 138, 533, 210,-200,-339,-190, 327, 422,-118,-337,-275,  31, 454, 223,-212,-289,-192, 182, 366,   0,-199,-199, -70, 277, 161,-134,-144, -67, 195, 262,-112,-277,-234,   6, 395, 378,-104,-353,-321,   0, 417, 256,-111,-255,},
{ 201, 150, -74,-239,-225, -59, 168, 284, 200, -74,-296,-259, -74, 182, 333, 199, -62,-241,-308,-125, 188, 315, 210, -50,-295,-322, -58, 334, 439, 159,-299,-531,-288, 268, 547, 306,-111,-443,-412, -64, 243, 373, 261,  -7,-188,-264,-215, -25, 174, 246, 192, -10,-221,-310,-166, 124, 343, 305,  25,-272,-370,-203, 120, 356,},
{ 187, 141, -98,-284,-265, -23, 332, 400,  72,-331,-485,-165, 303, 510, 263,-240,-480,-241, 122, 342, 282, -25,-241,-233, -51, 152, 245, 131, -89,-243,-217, -13, 185, 339, 161,-247,-339,-233,  59, 371, 346,  41,-266,-411,-196, 170, 397, 326, -52,-365,-335,-104, 162, 352, 239, -24,-227,-288, -88, 170, 231, 171, -63,-243,},
{  -8, -11,  48,  44, -77,-155, -14, 220, 229, -23,-290,-298, -60, 246, 373, 130,-290,-377, -70, 305, 428,  61,-426,-415, 106, 459, 282,-225,-436,-184, 274, 446,  73,-438,-378, 204, 492, 264,-234,-507,-307, 189, 482, 326,-159,-446,-210, 162, 294, 126,-103,-165, -57, -10, -12,   3,  57,  85,  43, -50,-113, -48,  45,  75,},
{ 683, 145,-471,-613,-203, 377, 557, 199,-339,-505,-164, 237, 366, 182,-147,-252, -95,  42, 119,  78, -45, -22,  -6, -48, -10,  50,  75,   6,-145,-105,  39, 142, 179,  -3,-229,-173,  45, 237, 268, -28,-300,-345,-112, 303, 472, 179,-256,-482,-215, 219, 349, 218, -79,-306,-218,  26, 227, 227,  14,-190,-221, -98,  69, 220,},
{ 324,  15,-261,-280, -40, 277, 293,   3,-273,-281,  12, 242, 214,  21,-232,-207,  50, 196, 197,  -5,-279,-216,  36, 260, 254, -65,-332,-217, 120, 336, 204,-152,-296,-200, 120, 381, 152,-153,-262,-202,  46, 228, 243, 103,-180,-360,-198, 167, 444, 317,-162,-504,-355, 144, 497, 382,-131,-466,-331,  64, 408, 383, -73,-392,},
{ 512, -89,-432,-266, 141, 391, 204,-230,-383,-116, 272, 341,  -1,-336,-261, 129, 338, 110,-231,-268,   0, 259, 239,-134,-308,  -9, 270, 159,-159,-251,   7, 225, 104,-161,-108, 169, 175, -80,-300,-223, 115, 427, 246,-255,-450,-139, 362, 457,  -6,-487,-400, 126, 477, 260,-183,-311, -81, 181, 168, -34,-165,-157,  49, 167,},
{ -67,  24,  94,   9, -94, -31,  43,  82,  11, -72, -68, -31,  92, 134, -71,-186, -14, 137, 130, -28,-185,-115, 125, 188,   8,-220,-108, 156, 177,  46,-215,-230, 133, 337, -11,-336,-104, 251, 284, -80,-313,-203, 163, 398,  87,-389,-315, 185, 473, 119,-429,-441, 116, 591, 232,-406,-526,  55, 590, 327,-339,-603, -85, 543,},
{-294,  56, 322, 214,-246,-350,  16, 319, 236,-202,-350,  19, 322, 213,-187,-391,  39, 345, 191,-147,-409,-111, 492, 364,-357,-451,  83, 420,  85,-305,-211, 158, 264,  56,-292,-259, 228, 353,  -4,-307,-250, 141, 346,  54,-241,-193,  64, 260,  16,-204, -19, 152,  86,-142,-239,  69, 362, 142,-313,-313, 120, 401, 118,-372,},
{-448, 152, 443, 119,-292,-251, 157, 322, -15,-313,-172, 241, 314, -93,-276, -98, 218, 245,-112,-343,-105, 288, 352,-216,-438, 102, 430,  84,-370,-259, 203, 357, -62,-337, -43, 359, 147,-292,-301, 121, 320,  90,-252,-227, 205, 285,-136,-284, -28, 258, 232,-215,-374, 108, 342,  85,-258,-230, 149, 301, -18,-371,-113, 318,},
{ 373,-118,-427, -84, 428, 217,-321,-368, 139, 489,  23,-510,-183, 429, 328,-202,-439, -12, 404, 193,-314,-393, 287, 464,-245,-404, 146, 374, -25,-358,-115, 325, 206,-330,-161, 216, 174, -19,-257, -98, 208, 147, -72,-204, -19, 255,  24,-207, -88, 129, 189, -54,-265,  -6, 223,  71,-145,-126, 100, 183, -34,-252, -43, 191,},
{-263, 220, 265,-187,-302,  67, 274,  98,-256,-168, 223, 175,-133,-244,  70, 264, -19,-246, -10, 188,  78,-108,-229, 135, 292, -86,-311,  21, 317,  48,-349,-150, 347, 221,-345,-293, 273, 411,-107,-506, -49, 440, 207,-277,-355, 125, 466, -44,-470, -19, 367, 127,-270,-288, 262, 321,-181,-325,  47, 348,  95,-388,-207, 343,},
{-383, 284, 389,-215,-375,  63, 328, 123,-353,-168, 388, 133,-269,-224, 136, 373,-129,-409, 119, 288,  62,-270,-261, 465, 233,-537, -93, 419,  32,-275,-160, 264, 318,-518,-103, 583,-149,-310,  20, 174, 168,-167,-239, 157, 164, -43,-138, -82, 253,  87,-329, -47, 280, 116,-182,-249, 109, 247, -63,-147,  20, 123,  44,-120,},
{ 216,-263,-119, 272,  10, -78, -86,  15, 193,-123,-112, 178,-103,  29,  42,-227, 231,  94,-226, -18, 102, 153,-118,-300, 358, 121,-491, 136, 421,-100,-382, -89, 453, 180,-627,  29, 534,-118,-353,  -1, 272, 168,-214,-331, 240, 287,-113,-272, -37, 356,  81,-490,   8, 452, -77,-249, -99, 192, 267,-260,-319, 412, 239,-344,},
{-209, 279,  41,-115,-210,  79, 414,-267,-318, 248, 124, 119,-176,-356, 328, 225,-201,-102, -95, 241, 134,-437, 270, 244,-562, 183, 315,-217,  -9, -73, 133, 137,-471, 339, 257,-631, 246, 311,-287,  22, -18,  -1, 186,-230,  32, 279,-268, -85, 114,  23, 236,-256,-317, 462,  78,-318,  47, 143,  58,-156,-223, 296, 307,-366,},
{ 370,-403,-258, 432,  75,-253, -91, 107, 333,-346,-301, 588,  -9,-450,  99, 184, 170,-280,-327, 525, 139,-524, 153, 194, -47, -30,-268, 270, 208,-442,  94, 303,-263,  51,  28,   5,  91,-272,  55, 401,-265,-202, 173, -17, 125,-125,-160, 276, -35,-214, 218, -30, -33,  30,-229, 263, 219,-457, -50, 395, -62,-195, -88, 173,},
{  33, 149,-298, -48, 462,-300,-145, 228, -46,  74,-241, -73, 528,-254,-268, 230, -39, 207,-208,-310, 559,-107,-360, 346, -62, -35,  76,-369, 419, 164,-594, 210, 321,-285,  82,-132,  33, 285,-322,  16, 231,-183,  14,  -3, -59, 301,-272,-192, 482,-175,-182, 199,-130,  28, 136,-300, 309,  51,-450, 280, 211,-290,  42,  45,},
{ 204,-194, -72,  36, 285,-128,-437, 350, 274,-305,-166,  84, 350,-152,-450, 427, 222,-491,  -2, 306,  47,-221,-165, 377,  16,-420, 329, 140,-512, 198, 374,-245,-185, 161, 128,-125,-159, 210, 111,-381, 101, 317,-115,-112,-165, 152, 347,-403,-217, 530, -69,-324, 109, 184,  41,-256,-114, 338,   2,-250, 132,  57, -67,   3,},
{-208, 246, 160,-266,-220, 438,  78,-505, 160, 373,-210,-265, 160, 358,-288,-379, 489, 211,-550, -72, 541, -54,-451, 180, 398,-354,-194, 447,-180,-357, 394, 156,-413, 217,  53,-235, 153,  86, -86, -97,   9, 206, -69,-218, 213, 100,-233,  17, 103, -10, -71, -46, 151,  61,-244,  65, 207,-159,-136, 106, 167, -73,-227, 178,},
{ -19, -38,  94, -32, -83, 110, -16, -64,  11,  52,  22, -37, -25, -32,  20, 139, -52,-259, 182, 263,-281,-170, 330,  -5,-296, 189, 191,-238,-207, 381, 106,-533, 188, 497,-590,-114, 614,-157,-392,  75, 345,  19,-395, -25, 385, -60,-302,  99, 287, -53,-337,   8, 422,-120,-350, 125, 305, -25,-397,  76, 454,-251,-378, 337,},
{  70, -63, -91, 143,  54,-178,   2, 198, -78,-260, 260, 224,-421, -63, 471,-138,-379, 199, 345,-247,-298, 392,  29,-336, 213,  74,-244, 188,  43,-238, 115, 208,-277, -73, 404,-249,-216, 343,  68,-299, -29, 228,  75,-266, -61, 336, -29,-373, 163, 352,-258,-313, 296, 284,-403,-162, 567,-120,-492, 290, 249,-202,-231, 216,},
{-297, 387, 126,-420, -35, 486,-146,-462, 408, 198,-431,  14, 285, -75,-140,  59, 159,-185, -69, 277,-206, -92, 352,-230,-222, 437,-144,-254, 298,  44,-297,  43, 303,-259, -39, 337,-267,-213, 354, 116,-397,  14, 319, -45,-254,  70, 230,-104,-253, 264, 191,-457,  46, 424,-229,-265, 289, 123,-323,  32, 228, -82,-171, 130,},
{-116, 138,  56,-174,  27, 205,-171,-148, 285,  -9,-287, 105, 252,-173,-198, 313,  13,-350, 183, 233,-307,  21, 257,-256,  14, 208,-199, -27, 233,-139,-139, 198, -15,-196, 291,-106,-242, 294, 124,-404,  26, 426,-155,-435, 343, 311,-485,-110, 530,-110,-470, 303, 315,-470,  11, 454,-329,-211, 425, -69,-319, 208, 148,-153,},
{-220, 388, -19,-479, 284, 348,-421,-132, 454,-120,-393, 387, 188,-528, 174, 370,-504,  96, 441,-335,-205, 377,-163,-137, 324,-171,-180, 292, -39,-234, 151, 117,-209, 154,  -5,-277, 343,  17,-342, 132, 309,-246,-230, 344, 123,-440, 132, 259,-193,-118, 171,  67,-221,  82, 133,-148, -43, 115,   5, -67,   5,  43,   6, -28,},
{-252, 437,-131,-265, 249,  59,-173, -11, 182,-111,-143, 313,-166,-181, 359,-191,-175, 365,-100,-248, 266, -72,-167, 313,-157,-198, 368,-105,-295, 287,  62,-288, 163, 141,-273, 142,  80,-186,  70, 172,-206, -84, 262, -34,-231, 137, 216,-348,  36, 346,-320,-150, 461,-173,-359, 461,  88,-565, 267, 376,-505,  20, 428,-282,},
{ 132,-252, 119, 152,-202, -97, 302, -73,-355, 479, -64,-482, 475,  68,-506, 373, 117,-391, 282,  59,-339, 296,  -8,-167, 151, -92, -14,  24,  96, -55,-143, 228,-143, -31, 168,-258, 232,  62,-341, 199, 242,-388,  26, 344,-249,-146, 356,-181,-175, 314, -84,-219, 234,  90,-379, 217, 297,-482,  85, 385,-407, -19, 320,-179,},
{-151, 266, -61,-232, 222,  83,-286, 181,  77,-283, 270,  -3,-255, 271, -64,-244, 430,-236,-204, 384,-231,  31, 157,-200, 127,-123, 105,  30,-267, 274,  19,-233, 243,-168,  94,  54,-289, 359, -67,-336, 343,  72,-315,  96, 224,-177,-197, 347, -31,-377, 360,  72,-376, 367,-126,-244, 484,-316,-183, 553,-440, -84, 481,-287,},
{ 234,-402,  84, 331,-357, -25, 412,-398,   5, 408,-452,  53, 362,-407,  86, 338,-423,  90, 246,-211, -71, 228,-174, -33, 261,-311, 103, 213,-423, 259, 218,-477, 304,  47,-273, 343,-260, -58, 313,-130,-256, 251, 144,-341,  94, 242,-272,  68, 180,-329, 205, 127,-346, 243,  23,-228, 250,-105, -91, 209,-150, -31, 148, -81,},
{ 165,-288,  93, 190,-278,  65, 192,-265, 139,  87,-239, 204, -10,-177, 218, -77,-186, 286, -75,-193, 277,-119,-156, 318,-255,  -1, 279,-319, 104, 210,-333, 121, 187,-375, 319,   0,-361, 433, -70,-401, 391, 180,-537, 147, 437,-485,  43, 383,-467, 185, 220,-427, 288,  99,-384, 295,  -2,-181, 215,-178,  58, 119,-233, 129,},
{-219, 480,-364,-117, 496,-357,-121, 477,-462,  93, 300,-414, 237,  76,-326, 375,-220, -36, 246,-259,  67,  96,-121,  89, -44,  45, -70,  20,  90,-120,  54,  12, -57, 126,-162,  99,  41,-231, 256,  48,-379, 332,  58,-402, 415, -78,-342, 464,-230,-157, 461,-460, 129, 263,-413, 248,  32,-197, 236,-186,  26, 145,-175,  74,},
{ 183,-419, 432,-158,-262, 503,-390,  20, 345,-465, 272,  43,-242, 263,-155,  26,  97,-244, 325,-190, -92, 319,-366, 225,  41,-292, 428,-317, -24, 310,-354, 188,  17,-143, 197,-235, 228, -86,-171, 347,-212,-185, 426,-244,-130, 349,-275,  26, 200,-321, 253, -22,-174, 261,-243,  99,  89,-222, 276,-199,  -2, 226,-337, 176,},
{-241, 568,-608, 278, 191,-417, 365,-169, -44, 193,-249, 199, -39,-102, 138,-169, 227,-115,-167, 319,-267, 101,  94,-248, 320,-260,  75, 183,-348, 267, -46,-165, 276,-257, 166, -65, -91, 287,-345, 154, 178,-378, 239,  92,-281, 218, -49, -62, 105,-106,  34,  52, -95, 131,-162, 158, -73,-136, 370,-416, 111, 400,-695, 378,},
{ -68, 204,-262, 138,  94,-242, 249,-155, -27, 190,-256, 234,-155,  -8, 178,-278, 324,-258,  53, 210,-390, 389,-236,  25, 258,-474, 476,-293, -15, 328,-496, 429,-196, -51, 178,-262, 382,-387, 171, 182,-463, 436,-107,-289, 453,-281,  12, 160,-227, 225,-147,  27,  89,-219, 292,-176, -84, 262,-287, 174,  62,-206, 211,-106,},
{  80,-196, 244,-169,  10, 107,-133, 134,-103, -10, 115,-165, 196,-189, 134, -28, -96, 179,-183,  81,  69,-185, 241,-279, 251, -59,-197, 375,-416, 252,  27,-249, 399,-465, 425,-298,  48, 241,-445, 449,-177,-193, 377,-303,  97, 136,-306, 332,-242,  56, 234,-464, 485,-339,  94, 188,-363, 351,-227,  11, 224,-383, 407,-184,},
{-115, 356,-514, 468,-244, -52, 298,-447, 456,-285,  27, 177,-293, 363,-333, 133, 142,-354, 390,-218, -76, 326,-401, 340,-257, 147,   0, -98, 194,-288, 275,-162,   3, 126,-155, 189,-263, 265,-171, -15, 226,-302, 185, -11,-131, 215,-230, 176, -39,-119, 278,-365, 319,-227,  84, 118,-273, 301,-248, 122, 107,-340, 372,-157,},
{ 103,-256, 394,-433, 323,-134, -58, 218,-318, 330,-261,  96, 130,-318, 420,-370, 201, -13,-185, 337,-384, 329,-208,  31, 124,-208, 238,-230, 224,-159, -14, 200,-331, 353,-320, 300,-225,  58, 145,-266, 283,-173, -41, 233,-342, 325,-191, -24, 253,-386, 414,-348, 187,  37,-243, 320,-272, 143,  47,-214, 315,-323, 238, -89,},
{ -85, 219,-329, 345,-258,  97,  94,-235, 317,-349, 296,-178,  39,  94,-208, 322,-370, 304,-164,  -6, 200,-362, 439,-454, 429,-310, 153,  -9,-166, 327,-398, 364,-237,  94,  -2, -87, 203,-302, 323,-251,  97,  98,-243, 268,-205,  56, 131,-252, 298,-290, 222, -93, -66, 247,-375, 361,-231, 108,   1,-172, 322,-397, 355,-145,},
{ -28, 105,-192, 218,-170, 123,-100,  83, -53,  21,  32,-146, 238,-280, 314,-323, 284,-197,  95,  21,-143, 278,-408, 421,-397, 394,-322, 198, -55,-107, 258,-349, 401,-441, 438,-402, 361,-264, 100,  72,-242, 374,-374, 236, -60,-116, 275,-358, 356,-272,  99, 102,-241, 313,-324, 248,-115, -27, 140,-217, 284,-324, 284,-119,},
{ -71, 226,-413, 525,-552, 527,-393, 175,  64,-277, 468,-605, 619,-509, 327, -93,-159, 345,-433, 446,-384, 282,-176,  68,  45,-173, 248,-255, 239,-187,  92,  13, -96, 135,-147, 162,-147, 129,-135, 105, -59,  32, -17,  15, -38,  79,-115, 132,-138, 130, -86, -14, 113,-167, 179,-144,  63,  -2, -39, 118,-185, 199,-166,  68,},
{ -27,  86,-158, 192,-206, 231,-219, 191,-183, 161,-105,  14, 106,-204, 236,-237, 263,-308, 332,-326, 254,-142,  39,  54,-132, 187,-245, 311,-349, 342,-290, 210,-100, -27,  99,-172, 314,-455, 521,-470, 329,-129,-122, 341,-448, 453,-413, 320,-146, -48, 174,-232, 251,-238, 183, -70, -55, 156,-253, 352,-408, 377,-278, 110,},
{  46,-138, 245,-308, 358,-413, 404,-362, 303,-240, 166, -53, -82, 197,-282, 350,-433, 528,-558, 491,-397, 322,-260, 221,-167,  90, -43,  25,  12, -70, 123,-152, 193,-232, 245,-305, 375,-357, 266,-155,  33, 101,-208, 235,-209, 154, -67, -27, 117,-194, 255,-295, 282,-243, 208,-140,  37,  33,-102, 219,-312, 316,-244,  96,},
{  -1,  -4,  27, -43,  64, -98, 118,-108,  96, -89,  99,-121, 145,-174, 196,-211, 239,-271, 299,-320, 313,-327, 362,-376, 415,-484, 518,-540, 582,-609, 568,-465, 353,-266, 235,-248, 262,-274, 249,-189, 126, -62,  13,  13,  -8, -20,  40, -50,  68,-114, 180,-218, 212,-208, 221,-221, 212,-178, 126, -71,  15,  29, -43,  17,},
{   2,  -5,   3,   1, -11,  15,  -6,   6, -19,  21,   0, -17,  12, -27,  54, -85, 114,-147, 191,-188, 149,-139, 131, -84,  40, -24, -11,  60, -85, 100,-106, 118,-136, 139,-138, 173,-271, 377,-440, 472,-492, 478,-391, 198,  51,-273, 456,-550, 581,-588, 547,-467, 364,-275, 194, -68, -75, 160,-222, 293,-360, 379,-295, 115,},
{  -7,  18, -48, 101,-160, 219,-273, 313,-326, 323,-333, 334,-303, 262,-234, 205,-176, 156,-115,  51,  -2, -27,  57, -98, 158,-231, 271,-293, 312,-318, 300,-262, 214,-163, 126,-102,  55,  13, -73, 115,-147, 164,-162, 165,-159, 139,-106,  74, -74,  88,-125, 198,-276, 365,-471, 548,-595, 600,-555, 475,-357, 259,-187,  76,},
{ -52, 126,-221, 319,-405, 467,-492, 491,-474, 471,-472, 465,-424, 362,-310, 265,-238, 249,-250, 204,-131,  71, -28,  18,  -9, -10,  18, -11,   9, -19,  34, -49,  73,-103, 109,-101, 106,-128, 157,-178, 191,-203, 191,-171, 172,-189, 220,-243, 250,-266, 279,-290, 296,-310, 343,-352, 327,-294, 256,-233, 202,-148,  97, -38,},
{  -1,   7, -17,  22, -30,  38, -35,  45, -53,  76,-150, 200,-186, 162,-138, 116, -73,  11,  31, -40,  38, -52,  70, -81,  82, -48,  27, -40,  58, -53,  30, -24,  10,  19, -26,  22, -61, 155,-287, 414,-523, 627,-697, 695,-649, 584,-496, 396,-291, 209,-110,   4,  65,-124, 190,-267, 346,-378, 354,-302, 247,-222, 183, -75,},
},

{
{  78,  84,  90,  98, 108, 118, 129, 139, 148, 157, 165, 173, 181, 189, 198, 205, 213, 220, 227, 235, 241, 247, 253, 260, 267, 273, 278, 283, 286, 289, 293, 296, 298, 300, 302, 304, 306, 307, 309, 310, 311, 311, 311, 311, 311, 311, 310, 308, 306, 304, 302, 299, 295, 291, 287, 282, 278, 272, 267, 262, 257, 252, 247, 241,},
{-185,-204,-222,-238,-255,-270,-285,-298,-312,-324,-335,-346,-355,-361,-363,-363,-359,-352,-344,-334,-322,-309,-294,-277,-259,-238,-218,-198,-178,-156,-132,-106, -80, -54, -28,  -2,  23,  47,  69,  92, 113, 131, 150, 169, 187, 203, 218, 231, 245, 257, 268, 278, 286, 292, 298, 302, 305, 306, 304, 301, 298, 292, 286, 278,},
{-234,-265,-293,-315,-333,-347,-354,-353,-350,-341,-329,-312,-289,-260,-221,-180,-140, -98, -54, -11,  33,  76, 118, 158, 195, 228, 259, 284, 306, 323, 337, 346, 349, 347, 342, 331, 317, 296, 273, 247, 219, 189, 155, 120,  86,  52,  18, -14, -46, -79,-111,-145,-181,-213,-241,-265,-287,-304,-320,-328,-333,-331,-323,-310,},
{-261,-293,-317,-330,-334,-330,-320,-300,-270,-231,-183,-129, -71, -10,  51, 108, 159, 208, 250, 285, 315, 334, 344, 347, 340, 322, 295, 257, 215, 167, 111,  52,  -7, -66,-126,-183,-234,-277,-310,-338,-358,-364,-363,-349,-324,-293,-260,-224,-183,-134, -83, -31,  17,  63, 110, 159, 210, 255, 291, 320, 340, 351, 353, 346,},
{-288,-320,-336,-333,-316,-290,-250,-198,-134, -66,   2,  74, 145, 210, 274, 322, 350, 361, 359, 343, 307, 253, 186, 113,  32, -50,-129,-205,-272,-324,-361,-379,-379,-362,-335,-287,-221,-146, -68,  13,  91, 159, 219, 268, 305, 330, 345, 344, 329, 300, 264, 219, 165, 106,  44, -23, -87,-148,-209,-258,-293,-313,-314,-301,},
{-292,-324,-331,-315,-280,-230,-163, -85,   5,  94, 183, 259, 312, 342, 350, 336, 299, 242, 166,  78, -16,-110,-204,-285,-343,-373,-373,-343,-292,-219,-134, -40,  62, 161, 251, 327, 372, 386, 374, 334, 278, 206, 125,  39, -49,-135,-215,-286,-334,-356,-359,-338,-299,-241,-167, -86,  -9,  67, 141, 204, 254, 291, 306, 300,},
{-341,-366,-358,-312,-235,-140, -32,  80, 186, 276, 340, 369, 362, 321, 250, 159,  53, -58,-166,-260,-329,-364,-355,-311,-239,-141, -30,  81, 185, 270, 325, 348, 332, 288, 223, 131,  23, -87,-186,-262,-312,-333,-329,-302,-245,-165, -72,  33, 138, 231, 299, 341, 361, 350, 304, 234, 144,  41, -67,-164,-241,-301,-327,-327,},
{-385,-404,-366,-285,-161, -24, 118, 245, 343, 399, 404, 349, 248, 118, -27,-164,-271,-340,-371,-355,-291,-186, -50,  94, 217, 306, 351, 346, 290, 197,  83, -34,-144,-236,-299,-327,-310,-247,-150, -40,  68, 165, 240, 292, 310, 296, 252, 178,  86, -19,-127,-223,-294,-334,-340,-310,-251,-158, -47,  73, 180, 266, 319, 335,},
{-336,-334,-275,-166, -25, 114, 236, 316, 346, 324, 241, 116, -28,-166,-284,-350,-352,-295,-191, -50, 101, 239, 339, 384, 359, 277, 145, -15,-166,-284,-355,-362,-305,-200, -67,  82, 218, 320, 366, 349, 277, 164,  27,-115,-243,-338,-380,-358,-283,-171, -48,  81, 197, 292, 342, 339, 296, 217, 110, -14,-135,-238,-297,-317,},
{-331,-324,-241,-100,  67, 210, 305, 339, 306, 215,  80, -76,-213,-308,-330,-283,-182, -39, 117, 255, 345, 361, 293, 156, -16,-183,-308,-377,-362,-266,-118,  54, 214, 328, 377, 346, 239,  86, -78,-223,-316,-355,-334,-253,-123,  34, 185, 304, 371, 363, 286, 157,  -3,-156,-275,-349,-365,-312,-207, -63,  88, 217, 301, 333,},
{-339,-325,-227, -70, 110, 260, 343, 334, 253, 123, -44,-203,-319,-354,-291,-147,  29, 194, 312, 353, 299, 165, -31,-220,-339,-362,-280,-118,  67, 234, 341, 360, 277, 137, -29,-194,-314,-358,-309,-187, -29, 136, 274, 357, 357, 269, 119, -65,-232,-342,-371,-320,-203, -40, 136, 280, 352, 352, 283, 154,  -8,-175,-299,-361,},
{ 379, 327, 169, -40,-241,-363,-382,-288,-112,  96, 283, 390, 375, 247,  25,-205,-353,-394,-322,-149,  65, 260, 373, 366, 248,  56,-153,-311,-363,-304,-150,  45, 222, 337, 345, 243,  66,-122,-263,-334,-312,-208, -41, 140, 283, 349, 315, 188,  14,-159,-286,-329,-276,-151,  10, 165, 275, 309, 269, 157,   7,-136,-242,-293,},
{ 374, 287, 106,-111,-285,-349,-290,-135,  65, 241, 324, 284, 147, -37,-212,-310,-302,-188,   2, 193, 318, 324, 202,  20,-170,-310,-325,-230, -53, 147, 299, 349, 257,  76,-135,-303,-355,-287,-123,  87, 269, 365, 341, 200, -10,-228,-377,-397,-276, -74, 142, 318, 394, 345, 180, -40,-246,-373,-387,-275, -78, 138, 290, 359,},
{ 392, 286,  71,-173,-353,-377,-246, -25, 195, 326, 331, 212,   5,-208,-332,-324,-180,  51, 257, 351, 287, 105,-122,-295,-337,-236, -34, 188, 316, 310, 182, -16,-216,-339,-330,-167,  73, 281, 370, 306, 123,-107,-294,-374,-314,-129, 108, 299, 370, 314, 138, -81,-263,-357,-309,-138,  63, 237, 350, 333, 189, -38,-250,-369,},
{ 343, 236,   9,-224,-339,-291, -97, 130, 291, 317, 198,  -8,-213,-321,-282,-123,  90, 264, 317, 225,  20,-208,-327,-271, -84, 135, 289, 314, 192, -20,-227,-350,-300,-114, 142, 335, 365, 220, -31,-260,-365,-316,-132, 110, 312, 388, 276,  15,-252,-388,-345,-147, 106, 309, 381, 283,  70,-168,-362,-403,-270, -15, 259, 423,},
{ 351, 223, -19,-247,-356,-282, -56, 200, 353, 324, 123,-153,-363,-382,-176, 129, 360, 406, 239, -63,-348,-432,-249,  64, 315, 395, 250, -37,-286,-381,-282, -37, 223, 367, 317,  96,-176,-338,-315,-133, 107, 283, 307, 184, -25,-228,-312,-220, -20, 178, 283, 248,  82,-133,-283,-285,-142,  57, 253, 340, 257,  45,-193,-344,},
{ 295, 171, -61,-267,-316,-176,  82, 300, 339, 171,-103,-328,-358,-164, 142, 360, 359, 119,-194,-384,-322, -50, 238, 353, 256,  17,-237,-346,-254, -25, 225, 359, 276,  26,-246,-372,-265, -12, 256, 370, 262,  14,-244,-366,-276, -31, 234, 360, 270,  26,-235,-364,-286, -47, 213, 350, 287,  86,-179,-352,-319,-100, 171, 366,},
{ 423, 181,-185,-420,-357, -68, 264, 421, 289, -41,-336,-401,-178, 151, 372, 339,  87,-215,-365,-279, -21, 246, 334, 183, -63,-254,-278,-121, 129, 294, 260,  39,-225,-338,-210,  68, 300, 325, 113,-167,-323,-272, -47, 202, 321, 245,   9,-234,-323,-203,  53, 283, 338, 173,-104,-312,-324,-163, 138, 351, 334, 119,-163,-353,},
{ 415, 133,-229,-385,-253,  58, 313, 320,  95,-200,-331,-198,  60, 268, 269,  64,-170,-279,-183,  40, 242, 267,  77,-175,-287,-196,  50, 273, 299,  98,-187,-351,-238,  57, 318, 334,  97,-225,-382,-254,  53, 319, 365, 158,-148,-366,-331, -63, 257, 397, 246, -70,-339,-386,-158, 186, 382, 339,  33,-309,-406,-216, 127, 390,},
{ 409,  59,-284,-385,-137, 215, 324, 194, -81,-271,-218, -62, 137, 249, 150, -21,-197,-254, -65, 183, 294, 185,-162,-383,-252,  82, 410, 403, -41,-423,-434, -71, 383, 448, 131,-246,-423,-201, 180, 307, 228,  -6,-219,-218,-122,  40, 208, 214,  93,-119,-308,-235,  43, 318, 357,  88,-282,-408,-187, 181, 413, 281,-101,-364,},
{  -9, 118,  25,-116,-221,-110, 257, 351, 110,-280,-486,-102, 374, 466, 134,-446,-525,  -4, 427, 481,  32,-529,-393, 111, 407, 352,-130,-451,-182, 124, 293, 177,-200,-188,  51, 130,  98,-143,-261,  72, 251, 203,   2,-339,-257, 111, 284, 256, -93,-383,-150, 149, 277, 155,-183,-282, -18, 146, 171,  63,-188,-170,  47, 131,},
{-286, -11, 228, 255,  25,-223,-224, -51, 161, 218,  58, -97,-167,-121,  48, 149, 149,  45,-154,-221, -86, 115, 288, 195,-118,-333,-268,  77, 408, 337, -41,-404,-438, -23, 439, 464,  54,-432,-501, -49, 396, 476, 137,-373,-472,-120, 282, 421, 171,-234,-367,-204, 126, 350, 238, -51,-250,-254, -56, 183, 243, 155, -43,-262,},
{ 552,  15,-481,-477,  47, 513, 411, -95,-506,-419, 114, 506, 375,-131,-523,-352, 190, 504, 306,-195,-489,-247, 225, 427, 201,-219,-375,-132, 191, 304, 100,-191,-210, -43, 117, 132, -24, -85, -13,  26,  45, -13, -56,  16,  44,   7, -41, -70,  33, 135,  44,-102,-154, -24, 159, 186,  -9,-178,-166,   1, 158, 163,  -2,-168,},
{  29,   7, -27, -28,  -2,  35,  53, -16, -74, -37,  60, 114,   8,-110, -91,  12, 133, 115, -78,-203, -96, 138, 260,  69,-221,-286, -42, 273, 307, -15,-332,-295,  60, 371, 302,-104,-400,-297, 109, 452, 322,-135,-455,-335, 117, 488, 350,-148,-513,-375, 166, 533, 374,-140,-497,-362, 101, 432, 323, -66,-376,-317,  28, 348,},
{-423,  75, 407, 244,-210,-375,-105, 273, 332,   7,-307,-250,  80, 301, 158,-190,-256,  19, 214, 157,-125,-229,  18, 198, 135,-105,-260, -24, 238, 184, -78,-278,-126, 208, 281,  -6,-311,-202, 166, 329, 108,-289,-351,  54, 358, 269,-143,-427,-162, 289, 404,  54,-411,-347, 124, 441, 275,-235,-460,-122, 265, 404,  89,-404,},
{-359, 110, 359, 136,-264,-297,  48, 329, 186,-200,-329, -15, 328, 224,-196,-361, -34, 333, 271,-148,-366, -70, 314, 261,-126,-372,-105, 301, 257, -78,-326,-149, 274, 293, -78,-325,-140, 268, 310, -94,-367,-185, 237, 377,  26,-343,-274, 166, 373,  69,-303,-290, 109, 384, 123,-261,-325,  17, 354, 206,-182,-343, -89, 327,},
{-377, 174, 399,  37,-395,-218, 257, 369, -21,-396,-220, 282, 377, -58,-399,-164, 297, 310,-104,-364,-102, 312, 264,-180,-338,   1, 319, 201,-176,-333, -37, 339, 195,-196,-306,  -1, 331, 174,-230,-289,  11, 313, 188,-224,-262,  47, 278, 115,-244,-220, 121, 283,  77,-295,-235, 184, 334,  52,-325,-278, 153, 402,  98,-346,},
{ 349,-208,-360,  38, 358, 119,-278,-251, 160, 305,  -2,-288,-149, 212, 248, -99,-281, -42, 251, 192,-190,-304,  76, 337,  83,-293,-230, 190, 297, -27,-313,-143, 265, 277,-158,-342,  10, 356, 154,-280,-307, 109, 386,  80,-342,-241, 211, 350, -31,-397,-134, 330, 281,-208,-391,  54, 409, 169,-293,-357,  80, 437, 159,-376,},
{ 283,-192,-295, 110, 325,  -6,-297,-131, 228, 242,-130,-312,   0, 320, 145,-292,-255, 200, 316, -32,-358,-122, 377, 209,-320,-290, 219, 364, -75,-391, -99, 365, 222,-253,-305, 107, 352,  27,-346,-159, 246, 280, -95,-358, -47, 363, 184,-299,-314, 190, 383, -60,-387, -70, 334, 202,-261,-290, 146, 335,  -2,-336,-165, 301,},
{-287, 210, 289,-132,-316,  38, 327,  89,-315,-187, 264, 256,-183,-313,  96, 360,   8,-368,-132, 332, 234,-289,-309, 233, 366,-140,-386,  25, 348,  99,-316,-200, 271, 274,-185,-318,  83, 363,   2,-370,-117, 317, 230,-210,-323, 125, 357, -51,-355, -39, 328, 149,-303,-224, 229, 294,-143,-347,  46, 361,  77,-350,-189, 298,},
{-271, 227, 273,-210,-296, 161, 323, -73,-339, -24, 349,  88,-322,-160, 281, 239,-231,-304, 176, 356,-116,-416,  77, 456, -17,-465, -33, 432,  84,-364,-170, 324, 237,-284,-300, 233, 323,-158,-311,  80, 328, -37,-336, -14, 313,  86,-287,-148, 254, 219,-228,-263, 180, 286,-124,-321,  59, 350,   6,-327, -85, 291, 179,-261,},
{ 185,-157,-200, 178, 181,-152,-168,  71, 234, -38,-267,  19, 256,  20,-273, -45, 281,  65,-284, -85, 317,  62,-296, -84, 257, 151,-278,-165, 286, 161,-252,-193, 216, 264,-192,-317, 184, 330,-185,-303, 133, 327, -74,-396,  44, 437,   5,-438, -71, 458, 112,-471,-136, 472, 156,-479,-168, 437, 200,-368,-248, 357, 293,-355,},
{-348, 368, 279,-376,-282, 381, 278,-344,-308, 316, 336,-297,-330, 252, 325,-210,-323, 175, 327,-148,-339, 180, 334,-233,-306, 238, 272,-224,-249, 212, 234,-207,-212, 178, 233,-176,-264, 197, 262,-209,-242, 183, 261,-158,-272, 145, 265,-136,-263, 160, 244,-170,-223, 188, 221,-225,-224, 237, 220,-223,-219, 224, 220,-234,},
{-243, 278, 170,-343, -77, 343,  26,-295, -73, 316, 113,-382, -62, 373,  16,-308, -47, 288,  97,-350, -45, 430,-142,-363, 240, 265,-246,-221, 227, 238,-273,-222, 331, 149,-318,-105, 279, 159,-321,-194, 373, 170,-370,-107, 303,  83,-257, -90, 276,  95,-365,  -8, 378, -88,-337, 115, 330,-133,-369, 201, 358,-261,-327, 307,},
{ 223,-280,-146, 377,  14,-366,  93, 293, -82,-305,  73, 363,-152,-340, 238, 241,-220,-230, 225, 268,-358,-201, 534, -56,-466, 210, 307,-234,-199, 191, 220,-232,-227, 318, 151,-360, -49, 330,  -8,-291,  18, 300, -39,-337, 113, 342,-216,-251, 247, 181,-220,-199, 220, 212,-331,-107, 431, -58,-400, 163, 282,-171,-222, 196,},
{-250, 343,  92,-418, 130, 342,-264,-200, 244, 176,-233,-221, 337, 135,-419,  64, 330,-148,-253, 176, 251,-280,-179, 393,   7,-397, 174, 289,-285,-145, 265,  97,-235,-120, 271, 108,-337, -16, 349,-107,-259, 153, 198,-121,-257, 202, 257,-344,-113, 408,-115,-296, 215, 190,-238,-208, 326, 168,-425, -15, 453,-181,-367, 296,},
{ 185,-371, 171, 124, -48, -61,-221, 374,  12,-365, 308, -98,-116, 221,-157, 123, -16,-335, 434,  -7,-390, 436,-264,-131, 457,-324,  -4, 276,-431, 171, 285,-347, 141,   3,-170, 290,-152,-112, 311,-366,  68, 354,-328,  52,  43,-163, 286, -99,-189, 309,-264, -29, 360,-295, -23, 309,-436, 162, 330,-439, 139, 154,-243, 113,},
{ 212,-306, -58, 362,-137,-270, 221, 181,-244,-131, 275,  79,-334,  30, 349,-197,-251, 294, 131,-320, -22, 359,-188,-267, 356,  97,-424, 129, 331,-265,-205, 326,  63,-313,  27, 290, -93,-281, 193, 260,-308,-177, 353,  76,-321,  -5, 313, -86,-336, 260, 263,-430, -30, 452,-215,-329, 347, 167,-408,  35, 314,-121,-229, 173,},
{-145, 260, -45,-289, 328,  16,-321, 183, 137,-124,-109,  72, 220,-248,-107, 401,-217,-239, 320,  33,-236,  48, 152, -28,-207, 138, 211,-359,   2, 401,-249,-233, 364, -12,-258,  83, 176, -48,-249, 131, 286,-334,-125, 469,-179,-328, 300, 132,-228,-119, 255,  90,-398, 145, 395,-488, -64, 574,-372,-264, 498, -84,-312, 199,},
{ -57,  33, 108,-116,-153, 392,-165,-348, 466, -43,-323, 223,  52, -44,-159,  81, 328,-494,   6, 607,-582, -22, 562,-551,  36, 439,-388,  -9, 213, -78, -42, -62, 129,  79,-322, 201, 219,-403, 112, 242,-249,  21, 115, -78,  58,-120,  89, 135,-311, 154, 200,-363, 152, 237,-392, 211, 105,-309, 224,  47,-196,  93,  24, -17,},
{-254, 477,-176,-366, 489, -42,-332, 187, 135,-114,-182, 198, 220,-463,  92, 463,-469,-100, 501,-270,-215, 355, -89,-177, 129, 110,-167, -80, 301, -97,-339, 368,  97,-440, 229, 229,-375,  60, 259,-153,-135, 120, 141,-152,-175, 349, -42,-368, 325, 103,-343, 147, 153,-188,  -2, 121,   6,-172,  90, 210,-295, -48, 362,-215,},
{  54, -80, -23, 107,  21,-213, 142, 161,-293,  74, 216,-263,  35, 177,-147, -22,  88,  18,-116,  20, 200,-258,  35, 296,-390,  36, 453,-480, -71, 559,-351,-294, 541,-101,-388, 297, 186,-376,  -8, 394,-188,-314, 370, 107,-447, 201, 318,-451,  34, 416,-349,-116, 386,-173,-197, 266,   9,-237, 135, 129,-200, -14, 205,-122,},
{-193, 359,-148,-233, 312,   3,-267, 132, 203,-271, -30, 306,-180,-185, 290, -29,-246, 204, 109,-308, 140, 179,-297, 141, 146,-325, 161, 248,-379,   7, 385,-280,-184, 410, -84,-344, 303, 135,-397, 138, 286,-313, -62, 306, -92,-273, 278,  99,-410, 267, 201,-459, 207, 260,-438, 163, 279,-406,  86, 303,-333, -10, 293,-178,},
{ 140,-219,   4, 237,-160,-185, 344, -54,-381, 416,  47,-505, 415, 137,-516, 322, 161,-371, 156, 163,-263, 120,  66,-144, 117, -44, -36,  89, -53, -16,  -5,  51,  46,-206, 190,  51,-312, 346, -41,-384, 465, -70,-375, 445,-107,-291, 303,  -8,-141,  37,  87, -46,-123, 227, -90,-230, 401,-202,-222, 523,-338,-235, 532,-259,},
{-235, 475,-267,-264, 581,-274,-357, 590,-156,-459, 561, -48,-491, 480,  24,-481, 419,  59,-407, 312,  25,-237, 250,-119, -57, 124, -87,  25, -10,  42, -31, -72, 194,-139,-118, 306,-212, -87, 326,-280, -40, 308,-230, -43, 214,-146, -83, 187, -37,-140, 111,  62,-168, 119,  53,-218, 203, -27,-149, 232,-136,-116, 251,-125,},
{  48,-114, 103,  -5,-113, 151, -77, -66, 170,-134, -25, 160,-152,  17, 134,-199, 118,  48,-161, 191,-167,  42, 179,-343, 269,  33,-350, 471,-238,-226, 474,-295, -65, 326,-293, -10, 225,-108,-130, 147,  77,-292, 199, 215,-506, 288, 238,-595, 478,  23,-475, 508,-149,-263, 459,-344,  10, 265,-352, 289, -34,-283, 364,-160,},
{ 127,-274, 229,   1,-226, 232, -22,-203, 220,  -6,-183, 139,  40,-126,  41, 103,-133, -11, 195,-224,  25, 241,-346, 192, 137,-390, 317,  69,-442, 420,  13,-435, 446, -38,-383, 394, -23,-336, 338,   6,-344, 307,  94,-402, 231, 217,-435, 254, 146,-431, 337,  63,-364, 304,  -1,-296, 343, -60,-280, 385,-184,-145, 291,-144,},
{-159, 365,-305, -18, 340,-397, 114, 307,-479, 185, 295,-471, 194, 257,-458, 235, 192,-438, 311,  57,-337, 338,-130,-127, 273,-190, -50, 201,-105, -99, 180, -36,-230, 290, -16,-327, 416,-149,-292, 549,-337,-161, 471,-392,  21, 344,-370,  94, 173,-265, 182,  23,-163, 126, -64,  34,  -2, -19,  55,-112,  79,  54,-102,  41,},
{-183, 439,-476, 257,  69,-315, 371,-204, -83, 289,-294, 143,  64,-209, 227,-135, -21, 171,-223, 136, -12, -64, 139,-236, 240, -96,-102, 294,-370, 195, 113,-318, 355,-177,-148, 363,-326, 137,  71,-250, 300,-173, -30, 223,-329, 248, -18,-203, 302,-257,  85, 131,-276, 310,-234,  47, 169,-328, 410,-308, -83, 529,-625, 275,},
{ 122,-313, 331,-122,-186, 398,-360,  44, 335,-474, 264, 134,-410, 362, -79,-235, 379,-270,  26, 188,-272, 249,-165,  38, 102,-214, 253,-170, -28, 262,-367, 238,  85,-421, 484,-185,-252, 520,-448,  94, 307,-474, 307,  37,-308, 346,-159, -98, 239,-217, 111,  33,-165, 192,-118, -21, 164,-217, 169, -56, -78, 162,-148,  57,},
{ 154,-403, 489,-332,  36, 235,-382, 331, -97,-181, 346,-327, 159,  79,-281, 353,-279,  74, 199,-394, 410,-275,  88, 122,-322, 397,-284,  35, 225,-357, 287, -70,-153, 263,-228,  63, 126,-196, 162,-110,  34,  59,-130, 182,-175,  70,  91,-229, 280,-218,  45, 182,-348, 352,-162,-146, 390,-444, 335, -76,-235, 402,-368, 154,},
{-129, 330,-424, 360,-186, -19, 192,-264, 214, -67,-109, 262,-327, 240, -37,-171, 307,-357, 290, -78,-176, 342,-394, 320,-121,-137, 317,-360, 270, -66,-148, 284,-315, 230, -64,-140, 295,-307, 187,  10,-213, 327,-299, 141,  87,-301, 375,-261,  45, 198,-392, 439,-306,  55, 209,-378, 379,-252,  70, 116,-264, 331,-285, 117,},
{-114, 304,-400, 357,-225,  57, 115,-241, 273,-202,  61, 121,-281, 326,-235,  50, 154,-302, 349,-289, 146,  40,-218, 351,-382, 275, -79,-123, 271,-318, 239, -82, -74, 205,-287, 268,-147, -36, 210,-313, 301,-162, -55, 262,-372, 340,-176, -51, 258,-377, 360,-225,  33, 180,-360, 443,-398, 253, -71,-141, 351,-445, 358,-136,},
{  92,-265, 395,-396, 293,-146, -15, 167,-273, 283,-189,  18, 165,-305, 372,-325, 135, 135,-325, 370,-339, 266,-141, -29, 196,-326, 382,-324, 187,  -7,-163, 267,-295, 231, -76,-116, 253,-291, 253,-139, -38, 211,-303, 284,-173, -11, 215,-357, 379,-306, 174,  25,-252, 404,-415, 298,-123, -47, 193,-310, 373,-373, 294,-116,},
{-101, 297,-452, 506,-491, 409,-225, -23, 249,-375, 380,-283, 128,  48,-220, 356,-412, 350,-181, -23, 182,-268, 308,-314, 258,-138,  -6, 159,-297, 352,-297, 156,  19,-187, 292,-310, 232, -76, -93, 235,-324, 333,-238,  71,  89,-197, 251,-251, 199, -88, -64, 204,-301, 346,-312, 211, -96,  -8,  99,-174, 241,-288, 248, -99,},
{  42,-132, 220,-272, 310,-336, 305,-199,  41, 117,-235, 280,-240, 126,  30,-172, 277,-365, 410,-373, 272,-154,  39,  77,-177, 246,-264, 227,-129, -29, 192,-322, 407,-400, 286,-106, -76, 202,-260, 257,-190,  67, 103,-287, 411,-439, 381,-252,  93,  58,-182, 263,-276, 228,-131,  -3, 134,-246, 340,-425, 474,-437, 312,-117,},
{ -36, 111,-188, 245,-293, 335,-339, 289,-187,  56,  79,-194, 261,-248, 157, -19,-134, 286,-406, 467,-486, 497,-511, 508,-435, 286,-113, -53, 210,-327, 360,-305, 188, -17,-163, 299,-366, 362,-305, 205, -84, -42, 158,-232, 222,-136,  21,  93,-189, 267,-323, 339,-315, 265,-181,  79,   0, -52,  91,-110, 109,-107,  85, -30,},
{ -33,  91,-140, 149,-113,  46,  43,-130, 199,-237, 246,-233, 192,-115,  24,  51, -98,  98, -58,  18,  15, -68, 140,-219, 291,-334, 338,-330, 313,-254, 161, -63, -37, 134,-171, 133, -39, -87, 189,-262, 317,-334, 312,-241, 124,  -3, -75, 104, -86,  28,  49,-133, 230,-348, 455,-539, 595,-599, 572,-553, 502,-418, 281, -96,},
{  41,-129, 216,-273, 332,-390, 405,-364, 279,-151, -23, 208,-374, 481,-523, 514,-458, 366,-241, 116, -23, -53, 132,-203, 251,-269, 256,-212, 151, -68, -42, 161,-261, 329,-359, 351,-295, 195, -84, -17, 107,-187, 222,-199, 129, -38, -53, 144,-229, 316,-385, 413,-391, 341,-280, 223,-175, 130, -86,  51, -22,   3,   1,   1,},
{ -19,  40, -40,  28, -15,  -5,  48,-110, 174,-228, 262,-267, 253,-224, 185,-153, 131,-113,  95, -89, 105,-129, 161,-220, 289,-349, 397,-435, 462,-456, 396,-306, 211, -98, -23, 120,-180, 184,-142,  88, -16, -88, 216,-345, 442,-493, 504,-486, 430,-352, 268,-179,  77,  28,-125, 210,-278, 318,-333, 345,-334, 302,-222,  81,},
{  12, -45, 102,-171, 253,-339, 393,-386, 333,-248, 123,  18,-146, 244,-309, 352,-380, 391,-386, 368,-339, 299,-257, 227,-188, 128, -62, -12,  95,-172, 222,-243, 246,-224, 178,-105,   3, 122,-247, 355,-435, 465,-423, 327,-196,  53,  75,-187, 277,-341, 390,-410, 387,-338, 272,-205, 159,-126,  88, -45,   0,  33, -39,  17,},
{   1, -23,  69,-128, 208,-297, 375,-424, 443,-443, 419,-378, 310,-214, 117, -38, -33, 101,-158, 188,-202, 222,-247, 267,-273, 266,-237, 199,-162, 114, -41, -50, 153,-268, 376,-457, 506,-531, 530,-512, 471,-404, 323,-236, 150, -75,  16,  25, -58,  85,-119, 149,-147, 137,-124, 108,-104, 106,-103, 100, -87,  72, -58,  23,},
{  -8,  28, -48,  64, -88, 118,-162, 208,-230, 240,-251, 268,-280, 274,-261, 246,-220, 187,-163, 154,-150, 148,-161, 185,-202, 217,-243, 286,-334, 367,-360, 335,-313, 275,-216, 150, -73, -22, 123,-211, 284,-346, 406,-472, 513,-516, 508,-488, 443,-393, 351,-304, 246,-183, 128, -88,  58, -40,  27, -10,  -4,  10, -17,  12,},
{ -16,  41, -65,  94,-145, 217,-290, 346,-386, 426,-457, 471,-455, 409,-351, 301,-253, 204,-146,  91, -41,  -6,  53,-102, 150,-182, 213,-266, 327,-380, 417,-449, 485,-503, 494,-482, 445,-390, 332,-276, 217,-154,  99, -45, -10,  43, -46,  36, -31,  33, -37,  38, -34,  38, -43,  41, -41,  40, -42,  42, -36,  31, -18,   4,},
},

{
{  69,  79,  88,  96, 105, 114, 122, 129, 137, 144, 151, 159, 167, 175, 183, 190, 196, 203, 209, 215, 221, 226, 233, 241, 248, 255, 260, 264, 270, 274, 278, 283, 286, 290, 293, 296, 299, 301, 303, 306, 308, 310, 311, 311, 313, 314, 315, 315, 315, 315, 315, 315, 314, 311, 309, 306, 304, 301, 297, 294, 290, 287, 283, 277,},
{-175,-199,-222,-240,-258,-273,-288,-301,-314,-326,-337,-344,-351,-354,-355,-354,-352,-349,-346,-340,-329,-318,-309,-296,-278,-261,-244,-225,-207,-187,-165,-142,-117, -92, -66, -39, -13,  10,  33,  56,  79,  99, 122, 144, 165, 183, 200, 216, 231, 244, 255, 266, 276, 285, 293, 299, 302, 303, 303, 302, 300, 294, 285, 275,},
{ 222, 251, 277, 297, 314, 327, 336, 338, 338, 333, 324, 313, 294, 266, 233, 198, 162, 126,  89,  51,   9, -35, -79,-122,-163,-199,-237,-269,-297,-320,-338,-350,-360,-365,-364,-355,-343,-325,-305,-284,-259,-231,-199,-164,-126, -87, -46,  -5,  33,  69, 103, 136, 169, 199, 229, 253, 275, 294, 308, 317, 320, 322, 317, 306,},
{ 260, 295, 320, 336, 344, 342, 331, 310, 283, 245, 199, 149,  93,  35, -26, -84,-141,-190,-234,-272,-305,-332,-349,-355,-351,-334,-311,-281,-244,-199,-148, -89, -29,  30,  87, 145, 200, 246, 285, 317, 342, 356, 362, 356, 340, 316, 290, 254, 212, 164, 114,  61,   9, -46,-101,-154,-204,-246,-278,-303,-321,-329,-327,-314,},
{ 294, 333, 354, 358, 348, 322, 275, 215, 146,  71,  -5, -80,-154,-221,-281,-327,-360,-372,-367,-348,-316,-267,-206,-132, -54,  28, 111, 183, 246, 293, 326, 349, 356, 347, 320, 273, 218, 154,  83,   9, -65,-133,-195,-248,-294,-325,-339,-339,-329,-306,-269,-223,-171,-112, -47,  24,  92, 150, 206, 255, 293, 313, 316, 304,},
{ 294, 326, 334, 319, 284, 231, 157,  73, -21,-115,-201,-267,-316,-345,-351,-337,-301,-240,-165, -78,  20, 118, 210, 289, 351, 379, 375, 345, 293, 221, 133,  37, -63,-153,-235,-303,-348,-367,-362,-332,-284,-218,-136, -47,  47, 136, 212, 278, 327, 352, 357, 339, 302, 247, 177,  99,  17, -65,-142,-209,-262,-296,-311,-303,},
{ 308, 332, 326, 292, 232, 147,  42, -70,-176,-270,-337,-370,-365,-325,-249,-152, -40,  74, 177, 264, 326, 360, 363, 325, 252, 150,  28, -93,-198,-279,-329,-352,-343,-298,-229,-140, -39,  64, 167, 253, 312, 343, 348, 322, 260, 175,  76, -30,-132,-222,-296,-341,-359,-347,-308,-238,-149, -53,  48, 148, 238, 303, 335, 332,},
{ 316, 334, 311, 253, 163,  53, -73,-191,-281,-335,-353,-325,-254,-153, -27, 103, 213, 298, 344, 346, 305, 221, 108, -27,-157,-260,-328,-351,-328,-257,-160, -46,  81, 199, 292, 349, 356, 319, 241, 130,   3,-121,-233,-315,-354,-350,-312,-233,-128,  -7, 122, 229, 312, 367, 380, 343, 267, 166,  44, -82,-197,-290,-345,-357,},
{ 319, 329, 289, 205,  82, -61,-198,-301,-355,-349,-291,-188, -54,  94, 228, 318, 353, 337, 267, 148,   7,-139,-264,-352,-385,-344,-234, -83,  75, 204, 298, 349, 344, 288, 177,  30,-114,-240,-326,-355,-327,-256,-145, -12, 134, 257, 334, 355, 322, 250, 140,   6,-131,-254,-333,-360,-341,-268,-150, -13, 130, 248, 314, 333,},
{ 342, 349, 282, 159,   1,-156,-290,-369,-369,-290,-147,  20, 177, 295, 348, 334, 254, 118, -34,-175,-293,-355,-335,-244, -98,  70, 223, 325, 351, 304, 197,  51,-109,-243,-331,-349,-294,-182, -35, 117, 245, 325, 349, 308, 202,  51,-102,-234,-327,-355,-312,-211, -72,  79, 219, 323, 369, 347, 258, 115, -53,-216,-325,-365,},
{-336,-330,-241, -88,  96, 256, 355, 370, 298, 158, -26,-209,-338,-382,-319,-171,  15, 190, 315, 357, 313, 194,  16,-167,-307,-359,-312,-184, -16, 145, 263, 326, 311, 229,  94, -83,-244,-340,-347,-271,-133,  30, 195, 317, 357, 316, 194,  24,-147,-277,-337,-329,-243, -90,  85, 240, 337, 365, 309, 177,   4,-177,-307,-367,},
{-325,-301,-193, -29, 158, 304, 367, 321, 175, -28,-225,-354,-376,-280, -91, 127, 299, 376, 345, 206,   3,-196,-339,-380,-308,-131,  97, 280, 368, 349, 232,  53,-154,-333,-407,-335,-167,  42, 226, 338, 355, 276, 113, -74,-242,-344,-351,-242, -69, 103, 242, 316, 302, 206,  60,-104,-244,-321,-299,-186, -29, 118, 234, 294,},
{-331,-280,-134,  59, 249, 349, 309, 156, -43,-219,-297,-273,-160,  -5, 148, 272, 312, 231,  60,-123,-259,-313,-255,-120,  69, 258, 369, 319, 132, -86,-260,-355,-331,-190,  26, 230, 362, 376, 247,  32,-185,-339,-383,-311,-114, 126, 315, 400, 346, 183, -31,-234,-363,-376,-264, -67, 157, 332, 390, 322, 140, -96,-282,-368,},
{-409,-305, -82, 150, 318, 349, 258,  88,-119,-283,-327,-245, -72, 128, 278, 335, 267,  88,-128,-297,-355,-264, -41, 206, 333, 309, 168, -42,-235,-339,-299,-133, 102, 306, 370, 267,  56,-175,-331,-352,-227, -28, 185, 335, 352, 222,  -5,-230,-368,-352,-196,  20, 222, 353, 344, 204,  -4,-230,-379,-360,-203,  42, 257, 357,},
{-434,-326, -69, 223, 412, 410, 213, -83,-320,-389,-284, -63, 188, 358, 356, 180, -83,-291,-353,-258, -52, 190, 340, 321, 157, -77,-299,-366,-254, -41, 188, 343, 338, 189, -70,-306,-366,-253, -42, 176, 321, 323, 179, -41,-257,-351,-270, -66, 163, 305, 305, 174, -21,-217,-311,-255, -85, 124, 279, 294, 188,   6,-186,-292,},
{-389,-244,  12, 254, 378, 303,  46,-228,-356,-286, -74, 169, 323, 307, 123,-128,-320,-340,-180,  68, 300, 370, 216, -28,-256,-363,-263, -10, 241, 357, 287,  62,-192,-337,-302,-115, 126, 300, 321, 170, -71,-267,-328,-222,  19, 254, 364, 257,  17,-204,-335,-301,-126, 110, 300, 345, 217, -10,-261,-411,-334, -67, 222, 410,},
{-341,-190,  66, 274, 328, 173, -84,-284,-303,-150,  97, 296, 320, 145,-116,-311,-339,-159, 122, 319, 330, 153,-111,-300,-320,-135, 150, 330, 302, 102,-154,-340,-311,-103, 160, 341, 308,  87,-190,-347,-290, -70, 200, 365, 298,  63,-203,-364,-321, -93, 205, 384, 355, 125,-177,-371,-361,-154, 152, 389, 380, 120,-176,-376,},
{-359,-175, 123, 330, 337, 113,-211,-407,-313,  12, 321, 418, 223,-118,-387,-395,-155, 183, 408, 367,  86,-253,-407,-263,  37, 273, 355, 188,-119,-322,-314, -95, 202, 351, 246, -28,-258,-318,-161,  80, 258, 292, 141, -95,-289,-309,-111, 153, 320, 275,  57,-184,-320,-252,  -5, 243, 327, 188, -83,-289,-302,-114, 126, 295,},
{-356,-148, 174, 355, 273, -10,-288,-363,-153, 197, 387, 276, -29,-328,-379,-132, 203, 377, 284, -28,-341,-386,-108, 241, 382, 259, -86,-377,-354, -91, 263, 428, 252,-109,-375,-343, -68, 244, 352, 217, -49,-284,-316,-136, 136, 322, 257,  21,-225,-296,-152,  77, 247, 262,  88,-150,-284,-231,   6, 251, 303, 145,-106,-296,},
{-253, -87, 151, 285, 177, -83,-282,-263,  18, 288, 297,  74,-217,-350,-172, 154, 352, 246, -54,-303,-324, -71, 262, 363, 142,-195,-402,-248, 162, 400, 309, -34,-363,-391, -71, 299, 379, 166,-158,-340,-264,  21, 274, 305, 118,-153,-328,-253,  18, 283, 344, 142,-153,-358,-309, -17, 318, 389, 135,-204,-388,-263,  91, 363,},
{-382,-120, 268, 409, 197,-192,-442,-262, 168, 444, 314,-135,-421,-315,  71, 418, 341, -64,-373,-355, -10, 362, 347,  29,-308,-365, -26, 337, 323,  55,-271,-346, -64, 216, 278,  94,-174,-243,-118,  67, 230, 211,   6,-178,-251,-116, 120, 249, 198, -13,-243,-269, -73, 210, 335, 162,-194,-344,-185, 127, 359, 233, -75,-311,},
{-168,  44, 125,  97, -53,-158,  -7,  88,  89,   2,-124, -23,  81,  64, -17,-140, -77, 129, 137,  10,-110,-174,  51, 183,  63, -81,-214, -80, 251, 231, -29,-261,-310,  89, 422, 213,-163,-432,-281, 255, 485, 265,-191,-571,-274, 317, 496, 215,-339,-540,-115, 349, 486, 161,-371,-507, -79, 358, 407,  63,-371,-343,  32, 322,},
{ 116,  33,-103,-131, -50,  80, 178,  92,-102,-215,-134, 125, 245, 121,-127,-271,-113, 174, 277, 119,-192,-342, -83, 259, 324,  75,-326,-385,  30, 364, 351,  -1,-415,-389,  80, 462, 345,-133,-459,-299, 111, 417, 342, -87,-409,-350,  22, 399, 379,  17,-317,-399,-112, 287, 381, 147,-182,-359,-189, 132, 298, 241, -32,-313,},
{ 613,   7,-515,-494,  40, 551, 403,-187,-537,-301, 275, 502, 203,-278,-473,-122, 360, 377,  27,-346,-387,  58, 404, 271,-132,-378,-178, 220, 291, 108,-188,-322, -33, 259, 217, -41,-260,-158, 129, 192,  95, -90,-192, -46, 131, 146,   6,-156,-114,  64, 108,  56, -54, -84,  19, 102,  25, -59,-106, -49, 109, 114, -20, -87,},
{ 340, -71,-298,-182, 127, 298,  97,-212,-274,  -6, 277, 207,-106,-302,-135, 232, 293, -34,-291,-204, 155, 330,  54,-277,-241,  94, 350, 120,-268,-283,  20, 298, 222,-175,-306,   1, 273, 209,-114,-360,-166, 256, 358,  46,-330,-327, 112, 428, 207,-280,-448, -86, 394, 389, -57,-410,-291, 173, 402, 135,-259,-396, -48, 395,},
{ 259, -62,-270,-129, 178, 244,  -9,-253,-149, 151, 248,  36,-239,-220, 118, 321,  77,-247,-264,  45, 315, 176,-217,-323,   7, 345, 185,-203,-262, -51, 256, 259,-147,-370, -34, 359, 226,-211,-364, -41, 319, 306,-109,-419,-143, 316, 359, -93,-435,-172, 291, 388,  -1,-438,-246, 259, 405,  33,-393,-295, 202, 435,  98,-388,},
{-396, 115, 426, 164,-330,-350, 126, 405, 115,-353,-310, 181, 406,  64,-340,-299, 174, 380,  60,-329,-293, 154, 452,  92,-420,-272, 243, 361, -51,-371,-140, 284, 290, -96,-343, -99, 287, 235,-149,-306, -57, 278, 236,-161,-270,   1, 215, 145,-164,-210,  76, 227,  69,-193,-223,  82, 304, 120,-259,-275, 100, 333, 106,-297,},
{-310, 124, 319,  38,-250,-163, 176, 260, -82,-304, -65, 269, 209,-179,-252,  32, 248, 126,-199,-259,  73, 313, 106,-313,-231, 235, 301, -85,-329, -76, 260, 234,-159,-318,  47, 379,  87,-351,-249, 201, 353,  19,-359,-223, 300, 362,-169,-421, -40, 385, 242,-252,-384,  82, 401, 107,-348,-289, 216, 395, -21,-450,-182, 399,},
{ 368,-167,-395, -11, 383, 159,-309,-285, 180, 377, -34,-412,-120, 371, 271,-242,-382,  67, 391, 113,-354,-291, 326, 379,-242,-389, 127, 387,  12,-345,-170, 268, 285,-183,-318,  57, 313,  86,-296,-211, 217, 280, -67,-313, -60, 319, 121,-244,-204, 150, 276, -63,-298, -32, 247, 124,-167,-201,  92, 276,   7,-324,-128, 277,},
{ 283,-216,-297, 140, 331, -15,-337,-108, 304, 187,-243,-239, 162, 295, -78,-315,  -7, 302,  99,-249,-191, 165, 294,-119,-345,  36, 362,  51,-349,-132, 303, 230,-238,-316, 200, 366,-116,-410,  -2, 413, 134,-344,-243, 212, 327, -86,-385,   1, 384,  78,-325,-181, 245, 301,-190,-344, 109, 372,  -6,-392, -94, 373, 219,-337,},
{-342, 256, 356,-185,-399,  95, 437, -11,-441, -73, 435, 144,-381,-236, 296, 326,-215,-373, 119, 381, -22,-406, -24, 440,  34,-424, -57, 377,  90,-306,-160, 262, 226,-286,-228, 291, 201,-210,-224, 138, 257, -73,-279,  -1, 266,  80,-264,-128, 254, 174,-206,-230, 147, 284, -90,-307,  20, 301,  54,-261,-119, 246, 182,-244,},
{ 289,-267,-247, 242, 244,-195,-253, 186, 252,-199,-226, 163, 213, -82,-235,  -4, 286,  23,-292, -60, 326,  85,-342,-115, 364,  92,-362, -95, 376, 128,-362,-184, 353, 241,-384,-228, 367, 208,-301,-225, 244, 260,-161,-341, 129, 372, -84,-396,  35, 397,  26,-413, -47, 376,  85,-342,-137, 306, 192,-267,-244, 295, 262,-305,},
{ 355,-365,-264, 332, 286,-334,-321, 390, 295,-425,-234, 363, 214,-304,-200, 272, 208,-268,-217, 260, 224,-280,-215, 319, 221,-338,-203, 324, 147,-284,-123, 245, 152,-285,-114, 344,  22,-325, -13, 296,  53,-243,-104, 213,  90,-213, -60, 186, 112,-183,-157, 176, 225,-206,-270, 227, 300,-273,-281, 309, 250,-305,-262, 295,},
{-249, 272, 167,-283,-134, 311,  93,-326, -40, 330,   1,-336,   7, 372, -29,-391,  51, 386, -19,-423,  70, 434,-231,-329, 371, 171,-402, -68, 388,  36,-405, -33, 464, -68,-430, 166, 349,-162,-313, 123, 310,-107,-306,  94, 301,-124,-273, 152, 272,-177,-281, 188, 262,-184,-227, 142, 236,-140,-247, 165, 243,-182,-249, 228,},
{-168, 234,  64,-253,  24, 204, -67,-189, 127, 162,-188,-113, 271, -18,-254, 135, 172,-154,-175, 177, 193,-297, -18, 317,-217,-149, 316, -45,-319, 139, 287,-169,-286, 300, 180,-396,  -6, 353, -78,-307,  77, 312, -77,-344, 117, 401,-279,-303, 361, 202,-343,-195, 330, 204,-381,-189, 510,  69,-565,  86, 508,-223,-393, 316,},
{ 257,-330,-143, 387,  37,-398,  94, 360,-213,-297, 270, 279,-340,-241, 420, 149,-434, -95, 438,  85,-551, 101, 546,-340,-367, 461, 114,-408,  38, 318, -57,-301,  66, 331,-202,-253, 301, 130,-283, -60, 248,  30,-221, -58, 251,  25,-276,  57, 265, -95,-213,  42, 215, -56,-240, 119, 231,-149,-144, 126,  59, -78, -64,  74,},
{-130, 205,  15,-227,  53, 217,-100,-235, 214, 190,-348, -16, 363,-140,-310, 232, 235,-248,-222, 326, 106,-416, 198, 262,-351,   0, 298,-200,-115, 205,  47,-225,  -1, 348,-256,-256, 454,   1,-416, 145, 311,-153,-282, 143, 336,-277,-279, 429, 105,-476, 103, 401,-209,-313, 252, 253,-337,-129, 376, -12,-341, 106, 300,-220,},
{ 255,-381, -34, 413,-161,-335, 322, 173,-411,  48, 359,-200,-258, 336,  65,-364, 101, 324,-227,-268, 415,   5,-395, 244, 230,-386,  37, 337,-251,-198, 337,  92,-411, 146, 257,-302, -15, 284,-103,-233, 140, 201,-161,-207, 267, 128,-347,  24, 332,-189,-249, 303, 140,-369,   7, 365,-167,-260, 277,  83,-263,  59, 177,-124,},
{-211, 369, -69,-325, 222, 202,-268,-109, 334, -92,-290, 289, 118,-342,  82, 263,-235, -81, 235, -29,-218, 145, 139,-266,  94, 205,-339,  56, 410,-372,-201, 499,-169,-344, 473, -76,-377, 291, 182,-279,-117, 296,  42,-372, 190, 314,-398,-105, 436,-133,-283, 211, 128,-204, -60, 256, -36,-307, 239, 187,-383,  72, 299,-209,},
{ -20, 150,-235,  74, 143,-263, 247,  15,-282, 171,  36,  -7,  -3,-129, 195,-106,-120, 381,-280,-235, 493,-267, -26, 269,-396, 220,  92,-367, 465,-118,-380, 425,-119,-136, 281,-282,  75, 153,-312, 359, -36,-412, 363,  22,-195, 201,-209,  85, 144,-339, 362,  10,-468, 404,  46,-386, 425, -84,-336, 341, -52,-175, 238,-117,},
{ 169,-326,  94, 318,-363, -53, 408,-256,-189, 363, -50,-317, 160, 275,-305,-119, 444,-235,-299, 452, -68,-318, 344, -43,-291, 281,  37,-283, 197, 153,-285, -25, 300,-172,-159, 335,-125,-268, 290, 141,-384,  46, 350,-245,-175, 332, -30,-262, 127, 199,-194,-166, 349, -53,-365, 367,  93,-426, 221, 205,-316,  41, 202,-126,},
{-225, 376, -91,-248, 167, 157,-211, -35, 234,-191, -57, 312,-242,-133, 348,-173,-155, 271, -22,-208, 107,  93,-161,  94,  84,-235, 133, 181,-321,  41, 301,-254,-101, 375,-287, -84, 347,-219,-150, 320, -78,-261, 249,  87,-265,  31, 267,-204,-156, 331, -44,-367, 346, 151,-543, 318, 317,-594, 163, 448,-495, -19, 425,-261,},
{-143, 278,-120,-227, 404,-169,-255, 414,-162,-164, 213, -95,  26, -16,  -1, 111,-241, 161, 143,-360, 284, -10,-311, 450,-223,-223, 559,-431,-178, 619,-334,-317, 605,-331,-141, 407,-301, -34, 214, -99, -72,  44,  84, -30,-185, 247,  -8,-283, 314,  -3,-360, 322, 103,-382, 233,  72,-212, 124,  29, -53,  20, -69, 140, -83,},
{-103, 179,-101, -32,  37, 114,-156, -40, 271,-308,  41, 381,-468,  22, 472,-481,  66, 348,-433, 123, 263,-370, 214,   7,-163, 211,-162,  73,   5, -93, 184,-183,  15, 253,-378, 238, 121,-484, 434, 125,-597, 392, 214,-530, 300, 190,-434, 241, 122,-284, 129, 102,-140, -28, 149, -43,-166, 207, -15,-217, 199,  86,-273, 145,},
{ 163,-289, 102, 186,-262,  35, 252,-294,  45, 279,-355,  55, 342,-411,  37, 425,-467,  50, 372,-373,  64, 167,-217, 106,  44, -51, -34,  40,  74,-146,  68,  81,-226, 280,-125,-221, 484,-341,-157, 539,-345,-256, 525,-165,-303, 363, -20,-315, 267,  65,-293, 185, 105,-305, 213, 127,-349, 209, 122,-338, 241,  97,-302, 159,},
{-197, 353,-140,-226, 344, -81,-265, 350,-114,-225, 343, -99,-232, 298, -57,-252, 278,  -2,-227, 167,  90,-269, 203,  68,-315, 325,-103,-211, 413,-254,-168, 422,-327,  19, 263,-374, 258,  57,-286, 196,  92,-224,  32, 207,-157,-119, 304,-207,-152, 447,-323,-148, 481,-372,  -4, 356,-414, 131, 245,-403, 220, 134,-341, 186,},
{ 164,-335, 200, 148,-370, 232, 119,-375, 318,   4,-313, 333, -46,-281, 325, -32,-310, 329,  13,-347, 373,-117,-232, 437,-337,   1, 325,-399, 181, 166,-332, 171,  97,-254, 212,  12,-228, 197,  49,-220, 107, 185,-288,  -1, 365,-364,  26, 320,-434, 218, 189,-446, 311,  59,-334, 334,-111,-139, 268,-241,  65, 177,-285, 139,},
{-199, 473,-419,  -8, 422,-438,  76, 340,-484, 265, 147,-425, 342,  24,-359, 398,-146,-170, 309,-186, -52, 195,-176,  46,  81, -94,  25,  16,  15, -34,  -1,  31, -60,  88, -78,  -6, 142,-238, 151, 142,-376, 283,  60,-374, 412, -97,-307, 466,-293,-106, 449,-426,  70, 286,-396, 239,  23,-213, 276,-206,   0, 228,-277, 118,},
{ 102,-279, 342,-169,-153, 393,-374,  86, 276,-459, 322,  30,-311, 336,-146, -99, 276,-306, 153,  75,-226, 259,-184,  21, 152,-242, 232,-102,-124, 259,-195,  33, 112,-208, 215,-141,   2, 192,-326, 245,  53,-358, 401,-127,-239, 438,-360,  73, 257,-456, 349,  -2,-274, 354,-299, 138,  71,-270, 387,-306,   3, 332,-434, 204,},
{-206, 504,-566, 291, 143,-429, 454,-252, -50, 304,-392, 254,  43,-278, 290,-149, -31, 222,-330, 243, -33,-143, 232,-271, 225, -69,-132, 300,-321, 146, 107,-288, 336,-241,  52, 157,-337, 402,-231,-116, 369,-338,  69, 211,-306, 200, -15,-117, 150,-107,  21,  36, -53,  77,-130, 166, -95,-111, 328,-353,  65, 349,-505, 243,},
{-100, 266,-337, 225,  11,-213, 305,-277, 143,  50,-238, 327,-253,  44, 185,-342, 380,-246, -38, 331,-471, 401,-189, -77, 358,-523, 453,-172,-184, 430,-462, 275,  14,-251, 369,-373, 267, -71,-168, 350,-343, 145, 108,-287, 296,-135, -47, 153,-197, 183,-100, -21, 146,-238, 232, -77,-131, 266,-291, 156,  77,-214, 200, -84,},
{ 119,-307, 380,-280,  55, 153,-250, 254,-188,  46, 120,-242, 290,-236,  96,  91,-261, 326,-255,  79, 128,-288, 353,-321, 179,  62,-292, 401,-371, 200,  56,-278, 394,-399, 324,-163, -89, 320,-423, 354,-113,-183, 350,-315, 157,  68,-268, 335,-260,  63, 198,-391, 398,-237,  12, 183,-278, 280,-208,  37, 178,-333, 334,-142,},
{-103, 283,-380, 329,-160, -50, 228,-331, 345,-251,  67, 130,-280, 350,-296, 113, 130,-317, 329,-183, -15, 179,-254, 266,-262, 218,-101, -45, 202,-332, 333,-196,   4, 156,-246, 315,-352, 275,-105,-114, 321,-386, 274, -66,-140, 289,-351, 273, -85,-119, 301,-391, 342,-202, -12, 244,-377, 373,-254,  30, 240,-434, 402,-157,},
{  99,-275, 425,-461, 348,-140, -90, 286,-421, 451,-344, 111, 169,-389, 484,-414, 223,   7,-210, 325,-350, 304,-195,  35, 122,-218, 246,-242, 212,-112, -67, 238,-333, 336,-286, 211, -76,-113, 271,-317, 259,-119, -47, 178,-262, 270,-184,  25, 151,-267, 306,-257, 120,  58,-221, 305,-282, 169,   9,-189, 297,-310, 227, -81,},
{ -78, 217,-332, 374,-318, 178,   0,-158, 281,-354, 351,-276, 151,   2,-141, 254,-325, 306,-194,  44,  98,-198, 249,-281, 260,-158,  54,  35,-159, 286,-335, 275,-129, -33, 154,-253, 337,-370, 324,-195,  -5, 224,-369, 379,-281, 101, 101,-244, 333,-353, 272,-114, -70, 257,-388, 390,-276, 120,  53,-246, 405,-476, 391,-149,},
{ -46, 133,-227, 301,-338, 321,-230,  87,  69,-216, 331,-363, 301,-162, -15, 186,-330, 411,-400, 307,-172,  30, 107,-218, 308,-373, 354,-264, 138,  29,-205, 328,-381, 378,-348, 302,-208,  72,  64,-183, 270,-291, 216, -67, -85, 214,-311, 337,-295, 205, -70, -94, 233,-314, 316,-228,  83,  62,-194, 312,-395, 410,-316, 119,},
{  69,-208, 352,-449, 477,-447, 362,-245, 107,  44,-211, 372,-460, 451,-380, 254, -80,-102, 264,-386, 450,-480, 484,-421, 312,-186,  49,  85,-221, 335,-371, 324,-235, 152, -84,  -3, 115,-231, 312,-320, 292,-246, 144,  -9, -80, 122,-144, 139, -92,   4,  94,-156, 169,-154, 118, -72,  26,  32, -89, 109,-116, 126,-100,  36,},
{ -34, 102,-169, 216,-239, 245,-233, 210,-188, 156, -88, -18, 140,-232, 271,-292, 325,-336, 290,-205, 100,  19,-125, 204,-268, 300,-302, 294,-262, 187, -69, -60, 186,-295, 358,-410, 459,-469, 430,-339, 198,  -9,-201, 363,-428, 413,-348, 237, -81, -78, 193,-251, 265,-245, 179, -72, -42, 149,-258, 352,-393, 355,-237,  81,},
{  59,-161, 256,-318, 346,-351, 323,-280, 228,-161,  78,  16,-120, 210,-263, 294,-343, 399,-401, 358,-328, 305,-269, 231,-186, 122, -61,  18,  25, -68,  90, -83,  71, -60,  44, -31,  -6,  88,-193, 292,-376, 439,-452, 387,-265, 120,  30,-156, 254,-314, 328,-296, 217,-120,   9, 120,-236, 308,-362, 423,-454, 415,-291, 106,},
{  20, -65, 124,-179, 238,-298, 335,-347, 352,-360, 367,-353, 313,-264, 216,-162, 104, -57,  26, -20,  34, -72, 128,-186, 251,-322, 379,-430, 491,-547, 547,-477, 387,-325, 304,-302, 300,-294, 266,-224, 182,-136, 100, -77,  63, -57,  57, -71, 101,-141, 191,-232, 250,-264, 274,-262, 248,-227, 179,-114,  49,  -5, -13,   7,},
{ -30,  81,-133, 186,-234, 254,-238, 218,-199, 170,-128,  72,   1, -80, 148,-213, 288,-366, 419,-420, 390,-389, 404,-387, 358,-340, 310,-264, 218,-171, 103, -20, -57, 110,-150, 204,-274, 326,-358, 369,-352, 310,-238, 124,  22,-165, 292,-373, 406,-412, 388,-325, 225,-119,  12,  96,-182, 233,-254, 243,-223, 209,-156,  58,},
{   9, -27,  66,-133, 205,-259, 307,-350, 377,-408, 454,-490, 486,-447, 402,-357, 311,-275, 218,-126,  35,  35, -99, 160,-225, 289,-337, 386,-430, 452,-436, 390,-338, 298,-271, 261,-239, 197,-146,  95, -54,  11,  37, -92, 144,-179, 188,-182, 190,-195, 184,-181, 189,-200, 199,-181, 162,-146, 125, -83,  28,   4,  -6,   2,},
{   5, -18,  38, -60,  82, -90,  88, -90, 100,-129, 175,-199, 185,-162, 133,-103,  82, -61,  40, -28,  26, -14,  -8,  23, -25,  10,   8, -14,   7, -11,  24, -33,  48, -61,  62, -65,  86,-134, 201,-275, 343,-403, 445,-444, 415,-373, 305,-223, 114,  15,-151, 284,-392, 477,-542, 585,-609, 600,-552, 472,-377, 296,-207,  78,},
{ -27,  65,-100, 143,-185, 211,-220, 221,-224, 230,-230, 231,-231, 225,-216, 203,-206, 228,-231, 217,-196, 173,-162, 165,-151, 122,-109, 119,-123, 107, -80,  42,   6, -46,  66, -79, 107,-157, 217,-265, 308,-355, 398,-437, 468,-492, 516,-531, 526,-510, 478,-430, 373,-329, 299,-254, 193,-137,  91, -57,  21,  18, -32,  16,},
}
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
