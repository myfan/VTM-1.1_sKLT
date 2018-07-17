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
extern const TMatrixCoeff g_aiKLT8x8[64][64]=
{
{  14,  28,  39,  48,  55,  61,  65,  65,  27,  47,  63,  77,  89,  97, 101,  99,  39,  65,  86, 104, 118, 128, 132, 128,  49,  81, 107, 128, 144, 155, 158, 151,  60,  94, 123, 147, 164, 175, 179, 170,  66, 102, 133, 158, 177, 188, 192, 183,  68, 106, 137, 163, 182, 193, 196, 186,  70, 108, 136, 160, 177, 186, 189, 179,},
{  44,  68,  69,  50,  13, -29, -60, -70,  77, 115, 119,  88,  28, -38, -87,-105, 104, 153, 156, 116,  39, -49,-112,-133, 124, 181, 185, 136,  44, -59,-135,-159, 136, 199, 202, 145,  42, -70,-155,-179, 139, 201, 202, 142,  34, -81,-169,-194, 134, 190, 188, 128,  22, -91,-175,-199, 117, 161, 157, 101,   8, -93,-166,-186,},
{ -36, -70, -99,-124,-140,-151,-151,-134, -52, -96,-137,-172,-196,-207,-202,-175, -51, -96,-138,-174,-198,-207,-200,-168, -35, -67, -99,-127,-144,-151,-143,-117,   2, -10, -25, -40, -48, -51, -45, -32,  44,  56,  61,  60,  59,  60,  62,  61,  81, 113, 133, 145, 150, 151, 147, 133, 102, 143, 167, 183, 188, 187, 178, 159,},
{ -53, -49,   7,  68,  84,  42, -30, -75, -98, -92,   0,  96, 121,  57, -59,-127,-135,-128, -13, 113, 146,  65, -82,-165,-164,-161, -25, 126, 170,  75, -92,-185,-187,-186, -35, 139, 196,  98, -84,-190,-196,-195, -37, 150, 217, 122, -67,-177,-185,-183, -28, 152, 222, 134, -48,-158,-160,-153, -19, 135, 194, 118, -38,-135,},
{  89, 156, 190, 195, 176, 147, 119,  96, 111, 188, 219, 206, 161, 109,  71,  47,  77, 129, 133,  93,  26, -34, -64, -65,   9,  18,  -8, -66,-132,-179,-187,-153, -53, -76,-112,-162,-205,-219,-192,-138, -69, -92,-114,-133,-134,-108, -57, -11, -41, -38, -27,  -8,  28,  80, 133, 149,  10,  38,  69, 105, 150, 199, 238, 232,},
{ -59, -82, -62,  -5,  77, 158, 209, 204,-101,-143,-127, -49,  67, 181, 251, 243,-120,-176,-171,-105,  -2, 102, 171, 174,-110,-160,-169,-134, -74, -12,  33,  49, -57, -81, -90, -91, -90, -89, -84, -67,  26,  50,  53,  28, -30, -92,-134,-129, 107, 176, 195, 153,  56, -53,-132,-145, 147, 227, 246, 199,  97, -17,-102,-122,},
{ -60, -12,  68,  55, -38, -84, -15,  75,-110, -37,  88,  73, -72,-146, -34, 107,-151, -52, 117, 103, -83,-176, -31, 145,-179, -58, 151, 142, -75,-184, -17, 179,-198, -64, 174, 175, -64,-192, -17, 195,-209, -73, 176, 186, -63,-205, -31, 188,-207, -81, 159, 173, -65,-212, -50, 161,-182, -74, 128, 141, -60,-189, -56, 129,},
{-124,-204,-229,-209,-169,-124, -80, -42,-109,-172,-165,-109, -43,   9,  47,  64,  -7,   0,  56, 126, 168, 166, 150, 122,  77, 127, 183, 211, 175,  93,  22,  -6,  70,  97, 101,  60, -36,-146,-205,-180,   2, -20, -58,-108,-176,-228,-226,-165, -37, -73, -88, -80, -63, -31,  19,  60, -13, -22,   3,  59, 124, 187, 241, 243,},
{ 122, 114,  -5,-134,-163, -80,  49, 121, 193, 188,  12,-177,-223, -98, 101, 198, 207, 208,  33,-150,-197, -80, 117, 207, 152, 152,  31, -92,-119, -52,  63, 127,  40,  30,  -2, -16, -14, -23, -25, -10, -80,-108, -46,  64, 106,  37, -77,-124,-161,-189, -57, 134, 213, 109, -86,-180,-164,-178, -38, 158, 238, 131, -66,-172,},
{ -60, -87, -70,  -8,  96, 205, 274, 255, -84,-122,-120, -75,  17, 115, 175, 163, -57, -70, -93,-113,-117,-107, -95, -77,  32,  72,  57, -11,-102,-175,-217,-200, 114, 207, 218, 154,  53, -41,-111,-127,  97, 175, 193, 163, 125,  89,  47,   9, -38, -39, -43, -33,  14,  78, 103,  73,-165,-232,-243,-205,-110,   8,  75,  65,},
{ -44,  47,  59, -62, -71,  52,  47, -83, -84,  60,  79,-102,-108,  86,  75,-119,-117,  75, 119,-105,-110, 135, 124,-122,-155,  84, 158,-102,-124, 157, 149,-136,-201,  74, 181,-110,-162, 137, 135,-161,-225,  66, 200,-103,-182, 118, 124,-163,-228,  63, 217, -78,-177, 109, 134,-132,-196,  56, 199, -48,-143,  99, 131, -93,},
{  31, 171, 251, 145, -46,-116,   2, 126, -88,  54, 175,  57,-179,-244, -40, 159,-226,-134,  28,   9,-148,-182,  16, 195,-219,-157,  21,  98,  48,   6,  64, 134, -81, -34,  58, 125, 143,  71, -47, -90,  74,  80,  -2, -31,  42,  34,-149,-256, 162, 112,-111,-201, -33, 109,  -9,-149, 157,  92,-127,-207,  -2, 222, 185,  41,},
{-200,-210,-131,-121,-169,-147, -14,  79,-170, -93,  85, 108,  -7, -40,  86, 173, -55,  91, 293, 275,  67, -55,  12,  87, -35,  59, 161,  86,-108,-206,-148, -65, -64, -77, -84,-129,-152, -95, -23,  -1,  -9, -57,-112, -91,  62, 226, 229, 114,  91,  63, -28, -31, 122, 239, 121, -63, 136,  99, -30,-104, -50,  -9,-156,-303,},
{ 174, 139, -60,-222,-237,-124,  21,  80, 232, 194, -10,-145,-114,  19, 135, 146, 131,  88,   0,  12,  83, 114,  68,   7, -68,-136, -93,  56, 133,  45,-139,-222,-179,-236,-108,  88, 149,  24,-154,-218, -97, -93,  33, 149, 142,  65,  22,  10,  39,  83, 112,  47, -61, -55,  92, 174,  85, 107,  35,-135,-275,-217,  24, 178,},
{  11,  39,  76,  54, -46,-180,-302,-310,  14,  37,  88, 120, 103,  61,   3, -36, -40, -59, -31,  28, 109, 209, 261, 216, -77,-125,-159,-170,-130, -38,  48,  60,  31,  29, -19,-106,-183,-190,-151,-107, 163, 221, 220, 160,  62,  -3, -20, -15,  51,  58,  90, 130, 130, 100,  79,  56,-179,-252,-221,-123, -38,  -4,  -4, -14,},
{ -32,  69, -15, -70,  70, -12,-120,  64, -71,  86, -35,-103, 115,  10,-145, 106, -96, 117, -22,-112, 162,  38,-162, 125,-115, 157, -15,-152, 156,  31,-211, 109,-144, 183,  -9,-194, 146,  42,-210, 120,-178, 196,  14,-203, 155,  71,-178, 140,-209, 189,  42,-195, 138,  68,-182, 116,-194, 159,  58,-159,  97,  38,-179,  81,},
{ 129, 147, 196, 188, -29,-201, -48, 160,   3, -94, -52,  49, -84,-224, -25, 211, -54,-237,-206,  43, 101,  -3,   1,  71, 118, -17, -70, 117, 211, 101, -79,-164, 239, 157, -29, -73, -25,  19, -60,-170,  88,  59, -75,-200,-162,  74, 194,  70,-151, -72,  60,  16, -63,  75, 187,  64,-208, -50, 215, 191, -42,-118, -99,-159,},
{-144,-284,-151,  89,  65,-100, -36, 124,  31,-104,   7, 243, 135,-156,-110, 115, 246,  97,  24, 121,  22,-183,-115, 109, 212,  19,-184,-136, -41, -22,  34, 116,  43, -79,-181,-111,  90, 184,  72, -57, -46,  12,  90,  61,  73, 113, -77,-271,-106,  57, 213,  30,-140, -23,  -6,-136,-179, -47, 127, -50,-214,   8, 220, 163,},
{ 269,  47,-165, -10, 152,  73, -48, -58, 300, -11,-279, -42, 147,  15, -93, -39, 167, -76,-198,  53, 113, -78, -55, 103,   4, -84,  18, 196,  58,-173, -31, 190,-117, -73, 126, 153, -83,-217, -39, 126,-138, -28, 115,   4,-121, -28,  64,  14, -64,  59,  79,-112, -77, 212, 153,-155,   8,  94,  36,-161, -82, 216,  89,-285,},
{ -83, -25, 129, 245, 269, 179,   7, -96,-135,-138,-109,-108,-110,-104,-131,-142,  44,  58, -30,-152,-152,  -1, 144, 158, 171, 226, 161,  60,  33,  86, 142, 135, -27, -48, -49, -22, -30,-123,-223,-213,-138,-199,-120,  40, 125,  60, -47, -72,  11,   9,  46, 115, 161, 152, 160, 165, 118, 113,   5,-130,-211,-205, -86,  36,},
{ -42, -33,  47,  96,  29,-129,-280,-297,  -4, -10,  32,  92, 140, 163, 176, 132,  -9, -57,-113,-134, -89,  20, 162, 175,  58,  46, -20,-108,-191,-220,-173,-110, 103, 136, 176, 197, 159,  72,   5, -13,-121,-155, -78,  41, 129, 135,  74,   8,-156,-187,-178,-175,-132, -78, -56, -72, 164, 239, 194,  53, -40, -18,  49,  61,},
{ -78,  -8,  45,-103,-131, 195, 183,-233, -61,  26, 158,  -1,-187, 106, 186,-237,  16, -58,  66, 112,-134, -84, 131, -39, 105,-139,-148, 168,  56,-166,  26, 110, 199, -24,-231, 120, 202,-141,-106,  83, 120, 122,-160, -64, 131, -46, -78,  58,-169, 132,  46,-143,   2,  59,  30,  13,-336,  90, 223, -59, -65,  39,  24, -70,},
{-169,-246,-224,  63, 186,  50,  13, -60, 111, 172,  28,  88,  31,-148, -22,   6, 125, 241, -15, -86,-134,-162, 119, 135,-227, -78,-132, -10,  80,  46, 115, -91,-206,  30,  33, 146, 157,   6, -20,-273,  36, 223, -10, -72,-104,-118, 159,  47,   5,  82,-192,-107, -42, -95, 200, 149, -62,  32, -55, 172, 207, -97, -95,-162,},
{  63, 160,   6,  67, -36,-182,  72,  20, -82,  -7,-175,  45,  56,-107, 138, -33,-103,  37,-152, 107, 132, -69, 123,-164,   3, 242, -79,  46,   5,-169, 169, -99, -93, 193,-162, -26, -23,-168, 261,   3,-218, 122,-148,  92, 146,-146, 130,-122,-140, 222,-111, 100, 167,-182,  54,-141, -55, 202,-199, -71,  61,-129, 183,  45,},
{ 230, 175,-195,-213, 137, 184, -40,-128,  37,  81,-122,-104, 121,  65, -65, -29,-241, -18, 116, 108,  19,-160, -47, 177,-224,  12, 150,  79, -57,-171, -23, 166,  34,  79, -45,-113,  -4,  92,  31, -98, 183,  65,-159,-132,  58, 223,  75,-250,  76, -49, -48,  85, -16, -32,  60, -72, -71,-124,  77, 238, -92,-266,  51, 187,},
{-261, 122,  78,-156,  -3,  13, -44,  55,-275, 220, 117,-141, 111,  70,-105,  37,-152, 206, -42,-190, 157,  33,-161,  47, -13, 119,-165,-124, 185, -13,-100, 129,  76,  35, -98,  87, 122,-121, -58,  90,  47, -81, -38, 153, -66,-160,  59, -24,  48,-121,  -7, 149,-150,   5, 289,-124,  76, -44,  18,  90,-180,  19, 236,-263,},
{  52, -51,-161,-196,-201, -91, 113, 157, 113,  95, 175, 274, 192,  41, -46, -83, -89,-147, -72,   7, -22, -60,-106,-151, -53, -80, -88,-140, -91, 114, 247, 167, 131, 224, 207,  62, -30, -44, -50, -52, -71, -74, -50, -29,  -4, -79,-177,-142,-126,-146, -97,  30, 206, 222, 179, 176,  99, 167,  68, -71,-113,-158,-127,  -8,},
{ -13, -35,  -3,  20, -11,-117,-214,-205,  34,  34,  40,  59, 132, 228, 289, 239, -51, -58, -90,-169,-207,-188,-141, -90,  69, 109, 149, 124,  71,  13, -38, -22, -56, -84, -31,  45, 124, 153, 117,  82, -63, -92,-147,-191,-192,-173,-136,-111, 181, 244, 221, 156, 113,  95, 119, 124,-127,-149,-109, -59,  -5, -25, -56, -54,},
{ -23, -73,-110,  92, 218,-152,-198, 236,  64, 115, -58,-113, 107, -32,-148, 142, -71,  95,  94,-166, -73, 194,  40,-149,-155,-102, 178,  62,-158,  86, 129,-170, 135,-131, -28, 165,-101,-149,  99,  77, 294, -37,-256,  82, 131, -87, -39,  43, -11,  45, -41, -36, 112,  46, -69, -72,-284,  94, 220,-116,-130,  41,  64,  23,},
{-258,-245,   6, 133,  43,  55,  64,-126, 189, 212, 106, -40,-199, -89, 119,  19, 101,  26,-158, -72,  54,  24,  18, -31,-195,-142, -60, 151, 254,   4,-148, -52,   7, 192, 137,-111,-149,-118,  32, 244, -10,  25, -37,-172, -45, 111,  14, -55, -34,-108,  28, 185, 191, 118,-116,-254, 105, -15, -33, -14,-163,-139, 105, 210,},
{ -47, -40, 192, 191,-221,-250, 102, 148,  27,-134,-129,  47,  44, 106,  70,-175, 175, 103,-121,-110,  69, 160,   3,-181, -76, 101,  87, -37, -77,-133,   4, 208,-299, -70,  87,  76,  97, -91,-131, 124,  83, 119, -62,-133, 113, 149, -87, -94, 206,  42,-121,-151, -97, 100, 102, -23, -75,-178, 111, 282, -57,-148,  10,  18,},
{ 292, -94,-222, 160,  46, -69,  95, -82, 128, -86, -80, 183, -82,-149, 160, -53,-199,  33, 112,  29,-107, -32, 146, -60,-207, 176,  68,-184,  71, 114, -38, -84, -25, 158, -84,-198, 206,  26,-183, 131,  28, -25, -48,  17, 141,-120,-173, 243,  58,-134,  75, 104, -90, -24,   7, -12, 102,-107,  66,  20,-196, 143, 190,-212,},
{ 119,-185,  84,  -9,-116, 168, -98,  66, 203,-205, 176,  34,-140, 171,-223,  55,  96,-224, 137, -47,-125, 235,-157, 125,  20, -66, 123, -49, -75, 149,-192,  63, -34,  82,  -2, -21,  23,  39, -84,  54,-118, 115,-176,  37,  49, -79,  81,  -6, -83, 223,-171, 154,  19,-174, 174,-129, -88, 150,-190,  97,   2,-120, 228,-117,},
{  23,  17, 134, 157,  67,  19,-120,-181, -20,-149,-195,-221,-157,  78, 228, 219, 128, 123, 196, 190,  78, -93,-242,-115, -86,-111, -50, -63,   7,  84,   9,  51, -36, -23, -41,-114, -79,  95, 112,  81, 130, 180, 142, 163,  16,-147,-197,-151,-160,-202,-247, -42,  71, 103, 189, 136,  43, 169,  80,  12, -54, -78, -37, -83,},
{  -1, 104,-134, 113, -10,  76, -54,  73,-114,  84,-160,  88,-175,  12,-102, -33, -43, 172, -64, 225, -89, 192,  76,  43, -58,  47,-256,  94,-235,   4,-146, -65,   8, 160, -88, 269, -60, 209, -11, 137,-112,  28,-165,  83,-269,  97,-205, -22,  -5, 137, -66, 179,-143, 253, -56,  87, -33,  45,-149, 146,-155,  94,-151,  48,},
{ -40,  14,-105,  -5, -54, -19,-123, -41,   3, 131,  29, 148,  41, 161, 107, 139,-105, -49,-159, -49,-245,-107,-193,-153,  80, 150,  35, 246,  90, 263, 168, 143,-100, -55,-231, -56,-240,-142,-239, -90,  15, 187,  69, 202,  59, 235,  84, 149,-100, -38,-159, -17,-189,   5,-156, -93,  42,  63, -43, 138, -54,  85,  -5,  70,},
{  77,  19,-126, -38, 295,-103,-241, 205, -84,  16, 101,-150, -27,  29,  -9,  48, -24, -73, 232,  12,-185, 132, 160,-241, 175,-163,-113,  95,   4, -97,  98, -22,  76,  31,-100, -14, 190,-152, -58, 153,-240, 148, 201,-207,  50,  45, -63, -59, -99, -20,  99,-105, -40,  75, 125, -61, 256,-144,-118, 147,  63,-165,  16,  19,},
{ 145, 203, -81,-233,  57, 184, -28,-138,-247,-186, 107, 181,   4,-165,  -6, 152, 126,   6,   3,  16, -39, -66,  19,  48, 189,  48,-129,-154,  60, 210,   7,-229,-240, -89, 179, 127, -49,-136,  26, 130,  -2,  42,  18, -52, -11, -83, -18, 108, 187,  40,-177,-107, 185, 186, -55,-214,-108, -63, 133, 102,-136,-147,  91, 104,},
{ 316, -83,-143, 183, -63,-142, 109, -18, -99,-132,  -6, 102, -18,  30, 106,-111,-170, 228, 189,-218, -57,  57, -41,   9,  -8,  18, -64,-125, 195,  51,-183, 137, 140,-174, -40, 144,  25, -75, -82,  75, 124, -74, 109,  58,-287,  49, 229,-128,-117,  -3,  19,  -4,  38,  72,  40,-181, -67, 169, -86,-115, 192,-131,-122, 212,},
{ -20, 164, 136, -64,-109,-121, -26, 141,-138,-178,-146,  19, 255, 206, -80,-155, 240, 146,  51, -32,-232,-137,  89, 110, -92,-147,  11, 142,   1, -12,  23, -49, -73,  12, -31, -24, 123, 125, -53,-106, 131, 207, -20,-199,-149, -81,  39, 207,-199,-196,  34, 266, 187,  -2,-138,-114, 132,  68, -39,-101,-147,  39, 118,   9,},
{-195, 216,-112, -21, 111,-139, 147, -76,-108, 190,-153,  35, 118,-212, 165, -73,  52, -32, -42,  36,  11, -40,  78, -54, 117,-221, 202, -43,-119, 151,-108,  49, 166,-226, 208,-101, -73, 211,-273, 148,  20, -72,  49, -43,   8, 116,-142,  74, -96, 158, -78,  50, -18,-113, 119, -45,-103, 167,-194,  85,  77,-143, 180,-113,},
{  10,  10,-122,  -3, 212,-150,-186, 256, -41,  61, 135, -92,-129, 162,  94,-203,  21,-142,   0, 139,-120, -45, 168, -90, 133,  43,-147,  11, 200,-134,-129, 163,-212, 103, 163,-203, -39, 158, -50, -50,  49,-133,  55, 131,-112,  -2,  76, -21, 270, -62,-238, 103,  88,-103, -24,  37,-259, 140, 145,-117, -30,  64,  16, -32,},
{  91,-156, -30, 261, -80,-175, 125,  -8,   1, 164, -67,-232,  42, 187,  20,-140,-227,  53, 154,  13,  69,-118,-163, 205, 184,-123,-128,   8, -14,  65,  20,   9, 139, -30,  63,  87,-144,  77,  75,-216,-278,  30,  50, -53,  45, -92, -33, 227, 114, 156,-122,-137, 201,  68,-154, -51,  15,-148,  62, 158,-174, -61, 174, -46,},
{ 260,  -4,-224,  50,  23, -13,  66, -63,-291, -17, 207, 121, -90, -85,  23,  36,  49, 129, -86,-267,  82, 188, -81,  -6, 119,-110, -18, 178,  44,-134, -91,  95, -84, -83,  95,  41,-203,  31, 168, -52,  41, 177, -73,-157, 146, 135, -94,-138,-109, -65,  -6, 107,  63,-245, -66, 314,  91, -20,  25, -17,-104, 144,  95,-212,},
{  50,  54,  42,  23,  56,  19,-117, -59,-101,-132, -69, -68, -97,  33, 141, 163, 147, 163, 118, 135,  63, -53,-186,-209,-140,-171,-207,-147, -33,  80, 236, 193,  77, 212, 249, 137,  15,-146,-223,-150, -62,-202,-231,-137,  37, 183, 167,  88,  87, 124, 162, 116, -60,-185, -94, -25, -50, -49, -52, -65,  36,  95,  43,  -9,},
{ -44, -77, -34,  29,  82,  70, -34, -44, 105, 140,  46, -54,-176,-122,  65,  96,-163,-161, -75, 113, 236, 140, -79,-152, 173, 201,  62,-168,-254,-129,  90, 178,-150,-234, -30, 200, 240, 107,-114,-163, 134, 195,  32,-203,-220, -56, 109, 137,-109,-125, -17, 154, 181,  13,-100, -92,  53,  48,   3, -65, -86,  -3,  60,  32,},
{-162, 236,-193,  55, 125,-204, 189, -97, -52,  81, -56,  13,  13, -80, 114, -64, 169,-212, 131,  -5, -83, 163,-156,  68,  63,-147, 131, -59, -85, 166,-208, 146,-103, 174, -90,  22,  42, -33,  25, -45,-116, 176,-221, 131,  43,-228, 260,-109,  18, -20,  42, -18,  -9,  20,  35, -62, 126,-204, 175, -95, -29, 146,-210, 136,},
{  44, -61,   2, 145,-191,  33, 227,-220, -46, 120,-112, -85, 209, -90,-196, 227, -89,   6, 129, -67, -78, 155, -67, -17, 192,-140, -87, 216,-146, -53, 154,-108,-131, 168, -50,-130, 179, -47,-110, 111, -90, -20, 151, -81, -51,  89, -10, -28, 298,-196,-123, 186, -45, -96,  94, -41,-197, 141,  86,-157,  78,  29, -51,  25,},
{ 221,-144, -84,  78,  93,-168,  99, -16,-209,  50, 312,-279,   8,  86,  34, -90, 151,-132, -68, -19, 228,-169, -72, 146,  66, -25,  56,   5, -94,  -5, 165,-160,-206,  90,  52, -54,  31,  10,  -6,   6, 189, -22,-181,  74,  59, -24,-144, 143,-113,  -4, 112, 112,-307, 203,  82,-149, -19, 119,-178,  42,  77, -24,-116, 112,},
{ -14, -63, 186,-195, 126, -81, 103, -74, 174,-187,  86,-135, 246,-243,  95,  14,   1, -95, 257,-229, 112, -73, 130,-111,  18, -66,  61,-123, 166,-125,  34,  25,  62,  10, -67,  65,   0, -35,  15, -13,-146, 111, -55, 106,-185, 149, -12, -32,   2, 134,-248, 202,-105, 113,-194, 143, -55,  64, -80, 165,-240, 204, -94,   9,},
{-122,  11,  32,  58, -22, -63,  23,  47, 162,  76,-132, -95,  52, 114, -21,-116,-181,-136, 171, 167,-100,-156,  15, 174, 215, 118,-155,-207,  93, 224, -44,-196,-203,-108, 136, 201, -66,-256,  43, 218, 157, 102,-115,-184,  81, 213, -10,-222,-103, -75,  86, 127, -56,-167,   9, 171,  42,  29, -37, -47,  10,  91, -15, -72,},
{-203, 266,-179,  31, 120,-178, 159, -88, 107,-112,  36,  43, -83,  53,   0,   3, 141,-204, 164, -69, -82, 205,-237, 121,-145, 154, -65,   8,  48, -81,  45,   5, -33, 115,-152,  84,  44,-138, 208,-157, 124,-222, 196, -81, -67, 138,-159, 112,  42, -57,  76, -70,  15,  93,-158,  88,-113, 169,-152,  81,  38,-158, 207,-117,},
{ 107,-115,  34,  46, -37, -72, 202,-161, -99,  69,  97,-247, 232, -43,-187, 191, 112,-129,   1, 151,-170,  47, 125,-145, -79, 161,-139,  39,  15,  -9, -34,  48,-127,  84,  -4,   8, -51, 133,-174, 106, 209,-147, -64, 193,-188,  37,  99,-101,-224, 177,  68,-236, 226, -56,-107, 112, 154,-162,  39,  45, -26, -71, 144,-100,},
{  23, -76, 160,-229, 225,-144,  40,  16, 140,-197, 151, -77,  58,-112, 171,-131, -65,  84, -32, -49,  87, -39, -46,  70, -57,  97,-132, 190,-219, 173, -71,  -4, -32, 120,-228, 267,-240, 177,-143,  84,-114, 116, -41,  14, -57, 126,-119,  50, 147,-170,  85, -38,  64,-128, 117, -52,  -4, -47, 147,-209, 205,-148, 107, -52,},
{  81,  10,-117,  79,  32, -86,  27,  20,-168,  24, 171, -97, -90, 161, -27, -56, 233, -57,-210, 115,  98,-142, -45, 120,-263,  82, 217, -95,-127, 139, 100,-173, 256, -90,-189,  44, 175,-164,-115, 199,-194,  33, 210, -71,-140, 130, 125,-195, 111,  17,-190,  60, 134,-133, -64, 129, -42, -17,  87,  -9, -98,  99,  -2, -40,},
{ 154,-200, 126, -10,-105, 167,-179, 103,-194, 231,-122,  -8, 116,-161, 158, -87,  81, -53, -16,  27,  19, -67,  85, -55,  74,-178, 194, -88, -89, 215,-231, 135,-119, 229,-214,  99,  62,-156, 133, -68,  14, -44,  31,   6, -34,  11,  72, -67, 118,-169, 154, -96, -10, 131,-235, 163, -98, 149,-125,  62,  33,-126, 181,-114,},
{  98,-156, 195,-209, 195,-166, 129, -65,   8, -41,  63, -84,  95, -90,  66, -26, -65, 126,-160, 197,-199, 162, -92,  28, -71, 127,-162, 164,-169, 177,-173, 104,  58, -91,  97, -78,  73, -83,  88, -56,  76,-141, 193,-240, 251,-216, 147, -63,  -5,  14, -28,  37, -35,  14,   4,  -8, -78, 134,-159, 175,-181, 175,-142,  68,},
{  52, -30, -37,  84, -69, -11, 104, -93,-114,  82,  39,-134, 114,  29,-186, 168, 174,-151, -14, 164,-157, -19, 201,-187,-188, 173,  19,-208, 211, -30,-163, 167, 178,-170, -18, 220,-231,  59, 127,-143,-184, 197, -36,-144, 161, -13,-138, 136, 158,-177,  41, 115,-139,  21, 103,-104, -71,  78,   0, -87, 100, -35, -36,  44,},
{  95,-126,  79,  -5, -89, 143,-165, 106,-157, 217,-138,  23, 117,-199, 233,-151, 168,-226, 149, -58, -52, 106,-132,  93,-113, 132, -60,  10,  33, -26,  14, -15, -13,  54, -92,  84, -37, -37,  96, -66, 106,-178, 166, -95, -26, 137,-214, 142,-137, 221,-193,  89,  74,-200, 261,-167,  89,-146, 128, -65, -36, 115,-143,  89,},
{ 109,-176, 209,-217, 190,-154, 113, -54, -75, 108,-100,  85, -55,  28,  -3,  -8, -55, 117,-190, 232,-245, 222,-178, 100,  48, -87, 126,-138, 121, -83,  51, -35,  45, -83, 112,-141, 166,-183, 160, -77, -70, 124,-158, 182,-197, 196,-156,  78, -26,  51, -74,  83, -82,  69, -60,  30,  60,-108, 141,-161, 161,-142, 107, -50,},
{ -14,  42, -81, 120,-157, 171,-162,  95, -11, -10,  64,-129, 196,-227, 226,-140,  87,-124,  97, -30, -59, 119,-148,  98,-157, 248,-245, 179, -74,  -9,  61, -49, 161,-254, 254,-187,  82,  -1, -57,  47,-116, 171,-149,  82,  14, -83, 124, -83,  48, -59,  12,  62,-143, 187,-192, 115,  -7,   0,  36, -83, 124,-139, 127, -71,},
{ -90, 143,-159, 148,-106,  58, -17,  -5, 125,-197, 205,-172,  93, -16, -45,  48, -93, 131,-100,  29,  71,-143, 188,-131,  37, -36, -21, 100,-201, 263,-282, 181, -25,  18,  30, -96, 186,-244, 254,-159,  60, -82,  60, -12, -71, 125,-146,  94,-114, 181,-189, 159, -92,  33,  10, -15,  87,-147, 161,-143, 105, -67,  33, -13,},
{ -47,  84,-114, 124,-117, 102, -80,  42,  78,-143, 189,-198, 178,-150, 120, -67, -65, 117,-154, 154,-137, 113, -87,  50,  37, -65,  81, -71,  56, -36,  14,  -7,  12, -24,  40, -63,  79, -84,  83, -47, -59, 108,-145, 168,-179, 172,-151,  86,  96,-174, 232,-257, 256,-237, 200,-110, -68, 128,-168, 181,-174, 156,-127,  66,},
{  50, -82,  99,-103, 100, -88,  72, -39, -87, 146,-176, 178,-165, 143,-117,  66, 101,-172, 210,-211, 196,-170, 138, -79, -91, 163,-202, 207,-194, 168,-133,  74,  77,-139, 175,-183, 175,-150, 118, -65, -68, 120,-152, 164,-157, 134,-105,  58,  59,-104, 130,-139, 135,-116,  91, -48, -33,  60, -77,  79, -76,  66, -49,  23,},
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
