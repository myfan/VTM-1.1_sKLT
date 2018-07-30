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
{  26,  41,  54,  65,  74,  78,  79,  74,},
{  58,  82,  82,  57,  14, -34, -70, -80,},
{ -78, -75, -13,  61,  89,  47, -32, -79,},
{ -86, -30,  71,  70, -32, -88, -15,  74,},
{ -83,  33,  81, -45, -71,  60,  59, -65,},
{ -71,  78,   3, -81,  70,  15, -83,  54,},
{ -51,  87, -76,  28,  35, -79,  84, -40,},
{ -27,  55, -75,  85, -84,  74, -54,  23,},
};

extern TMatrixCoeff g_aiKLT8x8Col[8][8] =
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

extern TMatrixCoeff g_aiKLT16x4[64][64]=
{
{  11,  23,  33,  43,  54,  66,  78,  88,  99, 108, 113, 116, 118, 116, 113, 107,  21,  36,  49,  64,  77,  92, 109, 123, 136, 146, 152, 156, 158, 156, 150, 139,  29,  47,  62,  80,  97, 116, 133, 147, 162, 172, 177, 182, 183, 182, 176, 161,  34,  55,  74,  93, 112, 131, 148, 161, 176, 184, 191, 197, 197, 195, 188, 173,},
{  56,  77,  93, 110, 115, 113, 101,  78,  49,  19, -12, -43, -73, -97,-109,-112,  84, 117, 138, 158, 165, 164, 146, 112,  70,  23, -20, -63,-104,-137,-154,-155, 101, 141, 167, 191, 198, 189, 170, 130,  78,  24, -29, -78,-128,-165,-181,-179, 107, 146, 174, 197, 203, 193, 170, 128,  78,  24, -33, -84,-134,-169,-186,-184,},
{  89, 111, 104,  86,  45,  -7, -49, -89,-119,-134,-124, -86, -24,  38,  76,  98, 129, 166, 163, 139,  76,  -3, -70,-125,-161,-175,-155, -98, -12,  77, 137, 162, 149, 194, 194, 163,  96,   6, -69,-128,-169,-179,-153, -87,  13, 115, 184, 208, 155, 196, 194, 162, 101,  12, -63,-115,-152,-161,-133, -62,  37, 136, 198, 217,},
{  74,  69,  22, -45,-114,-166,-178,-152,-108, -63, -24, -19, -66,-133,-193,-206, 117, 122,  63, -23,-120,-186,-189,-147, -72,  11,  69,  80,  11, -85,-176,-212, 147, 168, 112,  30, -71,-142,-141, -87,  16, 116, 185, 200, 129,  16, -94,-154, 163, 188, 144,  71, -13, -80, -72,  -8,  89, 182, 249, 266, 203,  91, -17, -84,},
{ -78,-115,-119,-105, -79, -60, -70, -99,-136,-180,-226,-246,-233,-190,-132, -87, -98,-139,-128, -97, -40,   5,   5, -24, -79,-136,-190,-221,-187,-127, -55,  -5, -84,-117, -91, -38,  47, 120, 141, 118,  58,   6, -51, -82, -52,  16,  91, 123, -55, -71, -35,  32, 121, 204, 238, 228, 185, 140,  91,  68,  93, 148, 201, 218,},
{-110,-107, -22,  67, 110,  72, -16, -86,-119, -95,   1, 102, 105,  35, -59,-107,-164,-167, -36,  98, 169, 131,  -3,-127,-182,-145,   9, 151, 167,  67, -77,-160,-178,-181, -37, 109, 205, 173,  16,-125,-210,-172,   8, 174, 204,  96, -78,-169,-160,-153, -14, 121, 198, 165,  28,-108,-184,-157,  13, 176, 203, 103, -60,-153,},
{  71, 133, 185, 219, 220, 217, 199, 163, 115,  50, -25, -98,-159,-203,-227,-193,  40,  91, 134, 164, 152, 142, 130, 100,  72,  32, -32, -79,-109,-141,-149,-124, -37, -33, -28, -32, -46, -55, -48, -37, -24,  -7,   8,  18,  41,  51,  53,  49,-114,-147,-169,-193,-200,-197,-169,-125, -72, -12,  56, 112, 163, 187, 187, 169,},
{ 107,  71, -34,-100, -78,  40, 128,  93, -20,-119,-114,  13, 101,  84, -11,-103, 168, 120, -48,-160,-127,  52, 183, 140, -20,-172,-161,  26, 164, 147, -13,-145, 192, 126, -75,-209,-173,  46, 212, 158, -15,-179,-173,  29, 196, 168, -28,-161, 164, 102, -80,-204,-176,  39, 198, 141, -19,-166,-163,  21, 163, 140, -27,-160,},
{  85,  26, -72, -86,  11, 104,  54, -58, -90,  -5, 113,  84, -78,-135,  -4, 135, 148,  63,-106,-141,  28, 186, 103, -83,-155, -34, 163, 149, -91,-207, -26, 185, 181,  83,-129,-179,  16, 197, 106,-110,-210, -61, 182, 183, -88,-245, -41, 189, 162,  73,-124,-181, -13, 163,  94,-101,-199, -63, 166, 182, -51,-199, -37, 164,},
{  86, 175, 233, 209, 124,  42, -43,-146,-219,-245,-201, -96,   5,  97, 168, 161,  50, 130, 189, 162,  83,  24, -47,-122,-162,-162, -93, -14,  38, 103, 160, 157, -63, -35,  -4, -34, -57, -13,  26,  43,  59,  65,  84,  75,   7, -48, -44, -21,-163,-178,-161,-178,-149, -33,  76, 154, 191, 192, 163,  80, -59,-170,-206,-183,},
{ -99, -16,  84,  17,-110, -93,  62, 146,  26, -82,  38, 124,   8, -91, -13,  97,-165, -34, 156,  51,-171,-150,  85, 197, -26,-199, -11, 149, -12,-191, -60, 108,-185, -32, 220, 104,-169,-178,  96, 208, -53,-244, -26, 181,  20,-214, -70, 122,-154,  -4, 234, 130,-122,-151,  81, 184, -34,-200,  -6, 200,  79,-152, -32, 148,},
{  10, -30, -36,  41, 140, 165, 117, 175, 202,  91,  26,  97, 174, 174, 209, 245,   6, -72,-136, -66,  26,   3, -86, -45,  -3,-164,-265,-176,-114,-127, -66,  28,  49, -13, -94, -52,   1, -82,-198,-141, -33,-137,-201,-109, -86,-181,-197,-115, 121,  95,  33,  57,  79,   9, -70,  15, 202, 191, 161, 267, 255,  83, -14,  -5,},
{ 169, 248, 218, 123,  -5,-101,-138, -81,  31, 136, 224, 269, 203,  55, -50, -65, 119, 165,  93,   1, -93,-172,-177,-138, -74,  19,  68, 101,  40, -96,-171,-157, -42, -68,-112, -75, -21, -13,  12, -16, -62, -82,-131,-120, -96, -83, -40, -15,-124,-171,-162, -24, 142, 209, 220, 190,  89, -14,-112,-125, -42,  67, 206, 232,},
{-104,  10, 108,   4, -47, 109, 171,   4, -50, 114, 134, -36, -23, 149, 137, -29,-160,  -4, 114, -75,-182,  41, 138,-102,-211,  49, 119,-165,-172, 108,  59,-147,-142,  37, 171, -71,-221,  17, 137,-122,-268,  30, 149,-168,-203, 104,  42,-188, -99,  92, 230, -15,-154,  70, 188, -16,-146, 109, 250, -24, -97, 162, 132, -78,},
{ -59,  82,  92,-122,-140,  55, -22,-119,  83, 104,-109, -57,  66, -75, -75, 110,-100, 110, 165,-132,-134, 150,  17,-158, 129, 170,-144, -42, 144, -91,-110, 147,-161,  70, 207, -99, -99, 227,  42,-214, 100, 170,-183, -28, 219, -71,-150, 104,-208,   1, 150, -90, -83, 211,  16,-233,  23, 101,-188, -26, 226, -39,-139,  56,},
{ 213, 225,  84, -52,-188,-177, -20, 124, 151, 112, -11,-197,-224, -27, 117, 126, 161, 139,  15, -59,-157,-180,  -6, 158, 136,  87,   0,-165,-180,  20, 134, 123, -26, -84, -80,  45,  78,   9,  13,   8, -76, -46,  55,  42,   9,  63,   7, -73,-145,-213,-112, 136, 243, 143,  25, -98,-236,-160,  93, 185, 147,  79,-113,-242,},
{  88, -69, -13, 170,  16,  24, 215,  26, -58, 115, -45,-107,  53, -78,-117,  48, 119,-159, -93, 139,-122,-153, 159, -76,-186, 145, -15, -70, 185,   2, -85, 125, 152,-183,-130, 160,-138,-221, 145,-100,-218, 175, -10, -99, 211,  10,-119, 130, 163, -99, -43, 254,   5,-106, 224,  21,-104, 189, -37,-157, 108, -64,-172,  67,},
{  22,  94, 131, 150, 229, 237, 137, 145, 153,   4, -44, -69,-210,-201,-129,-108, -94, -60, -69,-152,-109, -69,-193,-107,  38, -34,  37, 150,  45,  82, 170,  75,-108, -68,-109,-233,-173, -96,-220,-159,   5, -80,  -3, 163,  66, 107, 187,  49,  52, 166, 179,  91, 134, 214, 115, 103, 138, -62,-111, -19,-139,-136, -43,-134,},
{ 152, 158, -37,-168,-104,  85, 200, 136, -37,-213,-188,  29, 212, 136, -92,-195, 104, 126, -63,-186,-135,  48, 160, 137,   8,-149,-136,  35, 177, 147, -38,-153, -75, -24, -23,  15,  18, -26, -79, -71,  17,  50,  63,   0, -66, -50,  51,  75,-181,-104,  48, 208, 196,   3,-169,-184, -24, 174, 214,   4,-227,-218,  50, 208,},
{-110, 100,  10, -80, 117, -36, -18, 140, -53,  14,  91,-133,  64, -16,-179,  91,-157, 154, -12,-148, 160, -80, -88, 160,-148, -28, 113,-153, 146,  68,-177, 152,-144, 196, -24,-177, 184, -92,-123, 171,-172, -39, 126,-190, 137, 119,-185, 128,-110, 188, -14,-165, 187, -30, -77, 201, -92,  14, 168,-194,  58,  75,-221,  59,},
{ 185,  83,-105,-186, -48, 205, 134, -80,-181, -67, 194, 126,-130,-197,  -9, 207, 174,  53, -66,-143, -58, 194, 115, -59,-169,-108, 197, 135, -86,-130, -40, 157, -11, -58,  61,  65, -21, -19, -68,  30,  75, -52, -14, -59,   7, 135, -45, -84,-158,-162, 107, 185,  27,-143,-204,  88, 250,  49,-130,-191,  42, 261,  11,-169,},
{   5,  94, 123, 158, 165,  93,  13, -96,-208,-255,-237,-101,  15,  90, 227, 229, -70, -14, -91, -84, -32, -59,   2,  91, 114, 143, 107,  99,  10,-136,-134, -87, -86, -56,-180,-172, -87, -66,  30, 133, 169, 223, 148,  34, -37,-144,-192,-158,  55, 146, 100, 138, 100,   0, -22,-103,-213,-161,-125, -88,  72, 173, 179, 103,},
{  22,-169,  70,   7,-101,  82, -45, 134, 109,-167, -20,-108,  18, 105,-122, 126, 115,-179, 181,  91,-123,  84,-192,  40, 145,-126, 125, -45,  40, 141,-227,  66, 193,-202, 145,  28,-145, 144,-196, -32,  81,-113, 223, -38, -43, 115,-243,  36, 183,-219,  42, -37,-119, 249, -13,  15,  19,-154, 139,-100,-127, 153, -77, 137,},
{ 199,  79,-112,-105, 114, 177,-117,-182,  55, 178,   6,-252, -52, 253,  60,-133, 135,   7,-142,-122,  93, 184, -72,-189,   4, 183,  80,-181, -29, 190, -14,-135, -51, -54,  31,  38, -23, -30,   0,  63, -16, -71,  45,  79,  30, -68,-112,  70,-125, -41, 164, 121,-119,-188,  22, 275,  43,-276, -69, 159,  67,-148,-110, 238,},
{ 178, 173, 254, 198,  16,-131,-282,-176, -77,   6, 155, 150,  94, -40,-159, -90, -13,-188,-124, -81,  -9, 135,  90, 145,  86, -34, -43, -91, -12,  34,  20, 102, -81,-276,-143,-125, -53, 179, 135, 131,  32, -92,-105,-179, -54,  92,  65, 138, 163, 133, 255,  91,-112, -72,-167,-111, -38,  59, 178,  61,  30,   3,-153, -79,},
{ 201, -27,-247,  18, 207, -61,-193,  93, 219, -30,-127,  71,  94,-163, -34, 146, 208,  20,-224, -14, 198, -33,-199, -11, 123, -73,-159,  56, 121,-138, -50,  62, -54,  50,  90,  39, -28,  28,  85, -22, -76,  47,  72,  -4,  26,  46,  46, -81,-269, -19, 218,  12,-258, -41, 259,  -8,-220,  70, 164,-115,-160,  86, 135,-116,},
{ -14, 223, 179, -81,-244, -88, -65, -54, 271, 169,-153,-143, -83,-105,  40, 234,-202, -25,  28, -20,  11, 207, 101,-119,   8, -39,-109,  79, 186,  12,-116,  -3, -96, -81,-125,  77, 117,  58,  66, -78,-141,-124,  59, 169,  82,  46, -97,-190, 211, 118,-133,  64,  -2,-245, -43, 128,  90,  29,  74, -83,-274,  21, 211,  36,},
{ 203,  37,  -4, 135, -42,-285,  15, 242, -14,  23, 163,-157,-277, 107, 203,   6,  76,-135,-157,  67,  75, -96,  37, 126,-174,-194,  91,  70, -70,  90,  -7,-170, -76, -45,  10, -17,  94, 145,  10, -66, -29,   7,   2, 150, 175, -25,-138, -56, -98, 118, 169,-159,-122, 115, -62,-179, 160, 217,-193,-160,  45, -81, -11, 224,},
{  49,-130,  95,-130,   8,  19,-132,  54, -74,  62, -90,-127,  14,-168, 117, -81, 122,-113, 211, -95,  92,  98, -28, 211, -58, 161,  56,  65, 248, -59, 271, -28, 114,-185, 151,-172,  26,  -5,-165,  88,-261,  -6,-101,-121,  55,-288,  51,-193,  86,-173, 155, -88,  63,  40,-127, 170, -60, 125,  35, -62, 143,-107, 222, -26,},
{  74, -79,-161, 113, -84,-191, 126,-101,-192,  34, -49,-157,  32, -53,-212, 110, 106,  21, -69, 243,  96, -36, 224,  65, 101, 222, 142,  39, 210, 188, -57, 178, -96,  53, -77, -71, -64, -95,-103,-150,  -7,-202,-175, -75,-229, -41,-119,-122,-163, 148,  73, -82,   4, 126,  18, -28, 195, -90,  33, 267,-120,  52, 151, -30,},
{ 142, -78,  46, 132, -41, -10, 184,  23, -92, 208,  -9, -33, 262, -42, -33, 164, 101,-193, -35,   4,-145,-115,  17,-124,-243,  60,-171,-267,  28,-186,-168, -42,  24, -34, 116, -39, 108, 105, -34, 191, 111, 121, 142, 136, 111, 111, 222, -27, -58,  42,  93,-205,  76,  89,-245,  80,  20,-215, -38,  92,-151,-146, 142,-122,},
{ 209, 200,  34,-135,-272, -78, 189, 186,   6,-200,-211, 110, 193,  98, -34,-186, -54,-141, -60, 113, 107,  46, -33, -81,  22, 133,  75,  46,-117,-141,  88,  72,-155,-142, -16, 115, 177,  42,-180,-173, -92, 117, 143, -50,-135,-112,  64, 135,  63, 181,  39,-158,-131, -30, 107, 211,  10, -89,-102, -73, 172, 145, -84,-118,},
{ -53,-256,  43, 294, -90, -35, -26,-149, 276,  74,-250,   5,  -8,  86,  60,-173, 170, -48, -49,  73,-201,  39, 136,-117,  43, -22, -37, 202, -50, -63,  86,  22, 111, 172, -57,-168,  -5,  70,  90,  52,-155,-107, 121,  44, -79, -90, -66, 133,-274,  53,  64, -41, 209, -70,-231, 118,  58,  17,  71,-207,  54, 222,-147, -11,},
{ 259,  29,-107, -43, -92, 275, 106,-275, -19,  -2,  57, 200,-213,-172, 151,  -3,  -3,-179,  -4, 180,-145, -65, -23, -31, 178,  -1,-142,  70,  37,  76, 109, -81,-167, -20, 100, 157,  52,-147, -72, 144,  55, -28, -99,-131, 122,  67,-127, -89,   5, 214, -93,-272,  94, 114,  -8,  38,-189,  -2, 237, -73,  -4, -50, -82, 189,},
{ -10,  79, 123, 147,  73,  47,  75, 122,  40, -22,   1, -22,   9,-187,-165, -69, -94,-113,-214,-219,-218,-143,-157,-124, -93, -56,  75,  15, 185, 235, 238, 164,  97, 164, 120, 225, 187, 231, 145,  69, 100,  62,  43,-150,-209,-177,-179,-192, -40, -35,-120, -15,-119,-107, -62, -44,   8, -78, -40,  93, 116,  33,  96,  86,},
{-270,-127, 239, 124,-311, -63, 148,  58, -91,-136, 133, 151, -69,-179, 109,  56, 140,  95,-107,  33,  57, 142, -67, -57,  95,  45, -40,-121,   3,   9,  65, -95, 170,  95,-287, -84, 133,  91,-212,-102,  90,  58, -35,-125, 169, 103,   6,-115,-209,  -7, 149, 144,-146, -92, 103, 170, -99,-108, 105,  12,  18,-174, -20, 140,},
{ -43,  68, 124, -37,-139, 128, 241, -22,-103, 123, -46,-221,-138, 214, 121, -86, -57, -49,-104, -35,  73,-147,-249, -47, -39, 102,  92, 263, 137,-153,-196, -24, 118,  47, -17,  35, 139,  82,  11, 188, 105,-178,-340, -31,  17, -56,  74, 248, -13, -96,  76,  22,-160,   3,  20, -77, -95, 109, 207,  31,-141, 122,  -1,-179,},
{ -21,  59,-110,  67, -97, 139,-105, -45,  39, -39, 172,-236, 141,  49,  29,-109, -37,  84,-104, 205,-162, 130, -53,  46,  53,-168, 210,-219, 150,-186,  75,  29, -76,  93,-172, 203,-194,  38, -48,  45,  47,-169, 145, -78, 234,-263, 132,  15, -67, 167,-149,  93, -83, 144, -39, -70,  61, -14,  74,-179, 129, -99, 205,-176,},
{  23,-143, 257, -33, -48, 139,-113, -43, -98, -50, 256, -29, -67, 251,-187,-105, 140,-206,  38,-129, -68, 113,  13, 120,  96, -85,-142, -96, -67,  80, -64, 252,  60,  21,   9,  60, 139,-174, -66, -81,   5, 131, -78, 273,  53,-154,-125,  18,-176, 149, -38,-108, 119,-137, 207,  18,-141, 114,-117,  -3,-129,  95, 219,-160,},
{ -87, 154, -69,  52, 142,-188,  87, -94,-173, 174, 100,  81, -40, -18, 127,-263,-131,  94,-137, -71,  57, -57, 207,  74,   5,  18,-307, -83,  68, -89, 173, 115,  49,  19,  89,  33, -58,  18,-205,-120,  89,  33,  86, 156, 135,-158,-235,  63, 118,-193,  39,  52,-117, 167, -33,  94,  44,-202, 156,-145,-181, 262,   5,  -1,},
{ 189, -77, -66, 148, 134,-167, -91, 242, -33,-319, 108,  26,-155,  50, 231, -75, -78,-117,  40,-116,-131, 102,  52, -98,  81, 171,  96, -30, 128,-139,-138, -23, -39, 119, 208, -25, -93, 121, 102,-246, -33,  59,-177,-193, 182,  54,  -7, 131,   9,  14,-198,  12, 181,-179, -38, 140,  71,-137, 137, 119,-127, -41,  56, -92,},
{ 106,  19,  68, 245,  -4, -20, -15,-178,-178,  81,-138, -40, 142,  -6, -42, 178,-166,-122,-228,-199, -88, 103,  41, 272, 190,  67,  78,  29,-156, -31, -59,-135,  51, 272, 247, 107,  18,  32,-280, -87,-142,-190,   2, 119, -40, 124, 170, -39,  12,-136,-176,  40, -63,  13, 132,  82, -20, 164, -60, -94, 100,-101, -90,  54,},
{ 241,-118,-183, 157,-148,-182, 202,  27, -51, 166, 149,-149,  33,  28,-148,  39, -86,  24, 155,  39, 108, 167, -98,-168,  -4,-244, -48,  74,  11,  70,  96, -22,-168,  50,  97,-313, -93,  93, -81,  35, 309,  59, -37,  37, -90,-147,  58, -42, 134, -52, -83, 151, 133,-196, 111,  60,-275,  35,  63, -96, 123,  39, -23,  11,},
{  -4,-134,-165,  57,  90,  76, 176,  -8,-180, -76,   2,   5, 134,  59, -87, -56,  83, 227, 169, -39,-162,-232,-181,   2, 235, 175,  33,-116,-193, -76, 126, 119,-172,-174, -52, -63, 195, 292, 149, -52,-134,-251, -46, 198, 175,  41, -99,-145, 115,  42,  23,  24, -62,-191, -43,  17,  62, 129,  40,-118, -83, -17,  37,  86,},
{  53,  -9, 172,-128,  88,-103, 146,-168,  69, -40,  30,-111, 166,-123, 108, -33,   0,-189,  95,-175, 173,-149, 157,-149, 196,-148, 189,-135,  60,-114,  80, -70, 146, -28,  78, -98, 193,-242, 159,-159, 212,-272, 161, -66,  63,  -4,  29,  10, -46, -52,  87,-113,  98,-111, 220,-228, 153, -83,  89,-105,  57, -19, -31,  19,},
{  13,-137, 119,-122,  80,   6,-143, 214,-224, 251, -75,-146, 186,-167, 145, -70, 170, -88, 168, -67, -62, 122, -98, 126,-161,  35, -35,  83,  56,-189, 160, -70, -73, -38, -98, 156,-103,  49, -34, -65, 155, -47, -56,  70,-129, 144, -34, -21,-154, 306,-225, 101, -23, -38, 106, -90,  40, -69,  63,  23,-104, 182,-260, 166,},
{ 244, -14,  43, -27,-192,  41,  13,  51,  40,  -9,-152, 176, 102, -41,  50,-217,-212,-171,   4, 102, 214,  87,-173,   2,-129, 131,  82,-155,-165, -53, 113, 214,  74, 272, -15,-199,-152, -76,  41, 236, -54, -86,  27,  29, 164, 147,-281, -74, -49, -88, -55, 181,  30,  47,  19,-197,  59,  75, -94,  96,-159, -36, 110,  53,},
{-168, 350, -18,-171, 114,-129,  18, 163, -98, -94,  69, -92, 106,  67,-206, 122, -60,-216, -99, 178,  27,  77, -33,-194, 115, 111,   2,  37,-165, -24, 146, -21, 310, -14,  26,  48,-273,   7, 143,   1,  36, -65,-193, 171,  60, -27,  60,-175,-167,  14,  49,-103, 190,  21,-160, 121,-129,  64, 124,-125,  -2,  14, -68, 123,},
{ 184,  64, -44,-172, -25, 212, -39, -18,-145,  -3, 176,  15, -55,-116,  35, 125,-209,-184, 133, 252, -31,-202,-103, 166, 136,  -3,-193,-113, 203,  92, -30,-196, 160, 202,-150,-215,  37, 187, 115,-151,-177,  54, 137,  92,-149,-142, 104, 129,-116, -44,  40, 136, -38,-109, -28,  32, 138, -39, -82,  -3,   1, 140,-110, -31,},
{-141, 254,-183, -75, 193,-133, 124,-127, -51, 217,-159,  95, -57, -23, 195,-181,  23,-139, 150,  92,-129,  -2, -40, 102,  49,-171,  65,  13,   0,  47,-199, 165, 213,-137,  -9, -51, -57, 257,-174, -36,  69, -55, 133,-129, -53, 161,-130,  62,-165, 133, -20,  12,  66,-173,  89,  78,-127, 117,-124,  70,  93,-199, 188, -94,},
{-132,  -8,  64,  45, -43,-143, 169,  89,-130, -16, -23, 200, -59,-235, 127,  70, 177,  42, -86,-110, 117, 165,-215,-121, 109, 148, -56,-233,  86, 282, -78,-206, -90,-117, 142,  86,-186, -51,  80, 170, -76,-211, 124, 152, -75,-194, -53, 287,   4,  97,-106, -17, 102,   8, -27, -86,  14, 141, -90, -52,  28,  98,  44,-157,},
{  98,-150, 191,-217, 161,-110,  95, -56,   9,  74,-150, 188,-172, 168,-208, 134,  85,-152, 134,-106, 166,-170,  64,  14, -78,  86, -72, 106,-178, 164, -59,   0, -59,  94, -56,  32,-112, 133, -12, -28,  35, -40,  32,-103, 174,-160,  78, -24, -84, 153,-209, 236,-163,  91, -79,   6,  96,-169, 222,-203, 156,-130, 131, -71,},
{ -75, -37, 158, -43,-104,  16, 109,   0,-162,  49, 160,-100,-108, 163,  66,-173, 129,   4,-177,   1, 213, -28,-153, -17, 227, -51,-267, 142, 164,-204,-121, 262,-123,  22, 140,  -4,-185, -22, 193, -26,-157,   2, 248, -94,-176, 201, 108,-235,  65, -17, -76,  19,  79,  26,-101,   2, 101, -39, -64, -18, 149,-151, -30, 103,},
{ 144,  -6,-173,  91, 129,-196,  33, 102, -42, -85,  84,  46,-172, 141, -32,  -9,-206,   9, 285,-160,-181, 243,  22,-207,  39, 200,-207,  23, 149, -82, -64,  86, 192, -26,-210,  89, 184,-174,-122, 279, -73,-146, 143,  -3,-104,   3, 141,-130,-110,  39,  52,  26,-147, 110,  68,-153,  46,  53, -28, -50,  85,  -5, -74,  61,},
{-212, 229,-159, 147,-104,   5, 123,-219, 200,-135,  51,  47,-101, 141,-135,  74, 166,-119,  18, -51,  75,  -8, -93, 178,-142,  52,   7, -71,  98,-133, 117, -46,  60,-175, 263,-165,  -8,  91,-126, 125,-133, 106, -10, -57, 108,-114, 146,-122, -79, 154,-182, 100,  32,-121, 175,-199, 182,-104,  -5,  84,-137, 148,-161, 109,},
{ -28, -11,  86,-121,  69,  29, -62, -32, 216,-213,  42,  73, -49, -60, 175,-147, 111, -75, -43,  90,  15,-174, 223, -78,-167, 150, 102,-266, 193,  18,-209, 201,-133,  89,  45, -90, -13, 127,-149,  30, 171,-152,-121, 300,-209,  -6, 188,-183,  57, -12, -76, 108, -69,  23, -14,  79,-190, 171,  -7,-111,  75,  40,-132, 106,},
{ 102,-110,  53, -41,  92,-188, 277,-253, 162,-119, 128,-141, 125, -76,  50, -15, -44, -23, 158,-202, 155, -63,  -8, -48, 203,-223, 134, -49,  30, -68,  58, -44,  47,  -8,-106, 150,-104,  31,  -7,  76,-217, 215,-114,  28, -14,  47, -40,  30, -83, 119,-104, 116,-158, 214,-246, 206,-132, 125,-164, 187,-149,  96, -65,  25,},
{-163, 111,  81,-179, 169, -70, -44,  82, -35, -52, 120,-137,  59,  17, -70,  66, 274,-208, -51, 193,-198,  15, 170,-233, 134,  15, -93, 130, -36, -85, 166,-145,-261, 201,  79,-214, 189,  -7,-160, 209, -94, -51, 101,-132,  41,  81,-165, 150, 116, -75, -96, 173,-145,  42,  40, -41, -31, 103,-114, 117, -60, -16,  67, -64,},
{ 117,-135,  76,  -2, -89, 133,-104,   7,  81,-132, 141, -76, -32,  73,-115,  96,-182, 206, -91, -45, 176,-245, 215, -72, -94, 181,-178,  81,  62,-138, 204,-154, 192,-216,  90,  63,-176, 215,-171,  34, 129,-217, 187, -81, -51, 132,-182, 130,-115, 133, -60, -25,  80, -82,  45,  36,-113, 152,-126,  63,   8, -60,  85, -59,},
{ -73, 140,-191, 198,-182, 151,-116,  66,  12, -94, 149,-187, 208,-190, 146, -78,   3, -60, 100,-109, 111, -83,  78, -62,   8,  57,-108, 143,-167, 156,-110,  43, 123,-143, 156,-164, 149,-129,  57,  13, -61,  90,-126, 149,-138, 118,-117,  90,-105, 152,-177, 193,-174, 137, -73,  -2,  68,-118, 164,-181, 169,-150, 135, -84,},
{ 109,-149, 162,-134,  59,  18, -82, 124,-122,  99, -43, -31,  85,-113, 115, -76,-168, 239,-219, 158, -60, -51, 150,-216, 195,-144,  60,  63,-142, 179,-193, 129, 151,-216, 178,-113,  19,  73,-165, 209,-165, 104, -26, -90, 172,-198, 199,-126, -78, 111, -90,  52,   9, -60, 101,-108,  77, -38,  -8,  67,-110, 120,-112,  65,},
{ -56,  82,-103, 142,-148, 155,-183, 195,-198, 199,-209, 206,-182, 144, -93,  38,  28, -48,  53, -85,  89, -90, 114,-118, 133,-142, 154,-144, 116, -94,  55, -14,  56, -81, 121,-135, 148,-175, 161,-156, 131,-114,  98, -90,  93, -67,  63, -50, -61,  98,-125, 145,-159, 183,-188, 185,-173, 153,-129, 115,-106,  79, -65,  43,},
{  83,-127, 140,-152, 128,-113,  84, -45,  15,  40, -86, 121,-138, 132,-107,  61,-119, 189,-207, 208,-168, 143,-103,  45,   7, -92, 146,-189, 205,-194, 162, -95, 109,-169, 179,-172, 140,-109,  61,  -2, -55, 134,-177, 210,-224, 205,-165,  90, -55,  90, -96,  94, -77,  51, -17, -22,  53, -92, 124,-138, 139,-123,  91, -46,},
{ -45,  57, -69,  91,-109, 116,-124, 148,-136, 115,-108, 106, -95,  74, -51,  28,  64, -88, 127,-150, 162,-186, 210,-236, 213,-187, 165,-147, 127,-101,  75, -44, -66, 104,-140, 152,-168, 186,-209, 230,-207, 186,-172, 152,-125, 104, -77,  42,  42, -69,  88, -94, 104,-113, 122,-135, 124,-111, 110, -99,  76, -61,  39, -16,},
};

extern TMatrixCoeff g_aiKLT4x16Row[4][4]=
{
{  24,  55,  79,  81,},
{  67,  79,   0, -74,},
{ -84,  15,  75, -59,},
{  66, -83,  67, -28,},
};

extern TMatrixCoeff g_aiKLT4x16Col[16][16] =
{
{  37,  54,  64,  67,  43,  63,  74,  76,  45,  66,  77,  80,  43,  63,  74,  76,},
{  68,  55, -17, -66,  82,  69, -19, -81,  84,  72, -21, -83,  73,  61, -20, -77,},
{  54,  85,  99,  90,  27,  43,  50,  45, -22, -34, -39, -34, -55, -83, -94, -86,},
{ -68,  43,  54, -56, -84,  54,  71, -63, -88,  53,  75, -59, -80,  38,  62, -51,},
{ 100,  86, -29, -89,  50,  38, -26, -60, -41, -46,  10,  41, -83, -80,  37, 106,},
{ -34, -71, -72, -82,  46,  49,  80,  51,  51,  49,  85,  64, -40, -89, -71, -56,},
{ -33,  49, -90,  37, -32,  89, -84,  52, -49,  99, -70,  45, -71,  65, -70,  21,},
{  95, -69, -66,  78,  70, -35, -38,  34, -34,  22,  53, -49, -99,  26,  98, -78,},
{ -88, -73,  37,  73,  60,  70, -11, -65,  67,  47, -42, -80, -56, -74,  33,  91,},
{ -29,  20, 115, -32,  30, -96, -89, -36,  79,  52,  10,  98, -96,  27, -12, -44,},
{ -45, -25,  30, -92,  94,  18,  62,  88, -10, -72,-120, -11, -55,  89,  43,  -3,},
{ 101,-108,  51, -11,   1, -36,  72, -52, -62,  64, -15,  -5, -26,  67,-113,  76,},
{  50,  43, -17, -50, -96, -82,  37,  95,  90,  84, -47, -92, -42, -44,  27,  45,},
{  44, -77,  91, -61, -21,  58, -80,  56, -77,  75, -51,  28,  72, -83,  64, -36,},
{ -66,  48,  22, -32, 107, -68, -57,  73, -84,  38,  87, -94,  37, -10, -57,  55,},
{ -36,  52, -52,  33,  65, -93,  92, -60, -68,  95, -90,  59,  38, -53,  48, -30,},
};

extern TMatrixCoeff g_aiKLT16x8Row[16][16] =
{
{  15,  39,  23,  55,  31,  69,  38,  82,  43,  92,  47,  98,  49,  99,  50,  96,},
{ -35,  20, -54,  28, -71,  35, -85,  41, -95,  45,-100,  47, -99,  45, -91,  41,},
{ -35, -92, -44,-112, -43,-101, -32, -65, -11, -11,  12,  43,  35,  87,  47, 105,},
{ -83,  30,-109,  42,-103,  42, -69,  28, -14,   5,  46, -21,  91, -42, 105, -47,},
{  39, 115,  29,  88,  -1,  -3, -32, -89, -42,-113, -23, -58,  16,  39,  44, 106,},
{ 112, -35,  95, -32,   9,  -2, -81,  34,-113,  43, -60,  19,  41, -21, 107, -43,},
{ -35,-115,  -3, -20,  36, 109,  23,  86, -19, -51, -35,-119,  -5, -23,  26, 100,},
{-117,  41, -31,   4, 101, -38,  91, -19, -42,  23,-119,  29, -24,  -4, 102, -26,},
{  37, 101, -21, -63, -34,-101,  17,  78,  27,  93, -13, -90, -15, -80,   8,  87,},
{ 103, -39, -50,  23,-106,  30,  65, -24, 102, -17, -85,  21, -84,   6,  89, -13,},
{ -40, -78,  47, 104,   5,  -6, -44,-105,  28, 105,  11,  14, -28,-117,  12,  72,},
{ -84,  41, 101, -52,   4,   9,-112,  37,  99, -37,  21,   3,-112,  22,  69, -15,},
{  34,  58, -57,-110,  41, 102,  -5, -45, -25, -38,  37, 105, -28,-115,  10,  53,},
{  63, -29,-111,  49, 100, -41, -40,  15, -42,  16, 106, -36,-116,  37,  54, -17,},
{ -13, -34,  25,  73, -32, -99,  34, 114, -32,-114,  28, 103, -18, -80,   6,  34,},
{  37, -12, -76,  24, 102, -31,-113,  33, 112, -32,-101,  28,  79, -20, -34,   8,},
};

extern TMatrixCoeff g_aiKLT16x8Col[8][8] =
{
{  43,  53,  61,  68,  71,  73,  71,  65,},
{ -76, -84, -68, -37,   7,  48,  74,  79,},
{  79,  60, -14, -79, -86, -34,  43,  78,},
{  76,  11, -81, -61,  43,  93,   6, -78,},
{  74, -39, -84,  52,  68, -70, -48,  63,},
{  64, -80,  -2,  80, -77,  -7,  82, -55,},
{ -52,  88, -72,  27,  31, -73,  89, -48,},
{ -29,  58, -74,  84, -83,  72, -58,  27,},
};

extern TMatrixCoeff g_aiKLT8x16Row[8][8] =
{
{  21,  37,  52,  65,  76,  81,  80,  74,},
{ -55, -80, -85, -62, -18,  31,  68,  80,},
{  80,  77,  19, -55, -85, -47,  30,  82,},
{  89,  33, -69, -72,  32,  84,  17, -74,},
{  83, -34, -80,  47,  71, -58, -61,  64,},
{  68, -75,  -7,  82, -68, -19,  88, -54,},
{  53, -88,  73, -22, -41,  83, -81,  36,},
{  28, -60,  78, -84,  84, -73,  50, -19,},
};

extern TMatrixCoeff g_aiKLT8x16Col[16][16] =
{
{  28,  42,  53,  63,  71,  76,  77,  73,  30,  44,  56,  66,  74,  79,  80,  76,},
{ -62, -83, -79, -55, -13,  32,  67,  79, -64, -85, -79, -54, -11,  35,  70,  83,},
{  78,  73,   4, -64, -86, -46,  31,  75,  79,  75,   6, -65, -87, -48,  33,  82,},
{  80,  25, -74, -66,  34,  91,  13, -74,  82,  28, -74, -70,  29,  93,  18, -75,},
{  79, -32, -82,  53,  69, -65, -48,  60,  81, -31, -91,  45,  76, -64, -54,  56,},
{  28,  47,  58,  69,  77,  82,  80,  63, -30, -44, -57, -70, -78, -77, -72, -59,},
{ -65,  76,   0, -79,  78,   6, -82,  54, -69,  78,   7, -85,  75,  14, -83,  50,},
{ -50, -79, -85, -59,  -8,  47,  82,  73,  49,  78,  79,  58,   5, -47, -79, -72,},
{  52, -86,  72, -28, -26,  72, -89,  49,  62, -87,  70, -26, -34,  76, -88,  42,},
{  70,  83,  17, -61, -93, -40,  46,  74, -66, -79, -18,  58,  88,  46, -46, -75,},
{ -86, -44,  69,  73, -38, -86,  12,  72,  79,  48, -66, -73,  31,  85,  -3, -75,},
{ -49,  55, -42,  67,-100,  94, -49,   5, -11,  57, -98,  90, -57,  46, -65,  46,},
{ -65,  -1, 105, -65, -45,  52,  58, -76,  78, -27, -62,   8,  96, -88, -29,  60,},
{ -87,  76,  16, -81,  70,   1, -70,  59,  84, -69, -23,  83, -66,  -8,  72, -57,},
{  64, -86,  69, -30, -24,  65, -88,  58, -63,  82, -61,  21,  31, -70,  91, -58,},
{ -36,  57, -74,  82, -81,  71, -59,  33,  36, -59,  75, -83,  79, -70,  57, -31,},
};

extern TMatrixCoeff g_aiKLT16x16Row[16][16] =
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

extern TMatrixCoeff g_aiKLT16x16Col[16][16] =
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
