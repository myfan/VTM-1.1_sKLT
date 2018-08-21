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
extern TMatrixCoeff g_aiKLT4[4][4]=
{
{  38,  61,  75,  75,},
{  74,  69, -17, -77,},
{ -82,  39,  69, -59,},
{  53, -80,  76, -37,},
};

extern TMatrixCoeff g_aiKLT4HP[4][4] =
{
{ 151, 245, 299, 300,},
{ 296, 276, -68,-307,},
{-326, 154, 276,-237,},
{ 213,-320, 303,-149,},
};

extern TMatrixCoeff g_aiKLT8[8][8]=
{
{  26,  42,  55,  66,  74,  78,  79,  73,},
{  56,  82,  82,  58,  13, -35, -71, -79,},
{ -76, -76, -13,  62,  89,  46, -33, -78,},
{ -85, -32,  69,  70, -32, -89, -13,  75,},
{ -84,  31,  82, -44, -70,  61,  57, -65,},
{ -72,  77,   5, -80,  70,  15, -84,  55,},
{ -53,  87, -75,  26,  36, -79,  84, -41,},
{ -28,  56, -75,  85, -85,  73, -53,  23,},
};

extern TMatrixCoeff g_aiKLT8HP[8][8] =
{
{ 105, 167, 219, 262, 295, 313, 315, 292,},
{ 226, 327, 329, 230,  53,-141,-285,-317,},
{-306,-305, -52, 247, 355, 184,-132,-311,},
{-340,-129, 278, 282,-130,-354, -52, 301,},
{-337, 124, 328,-177,-281, 242, 229,-260,},
{-288, 307,  19,-322, 279,  59,-337, 220,},
{-211, 347,-299, 105, 144,-315, 336,-164,},
{-112, 224,-302, 341,-339, 293,-213,  90,},
};

extern TMatrixCoeff g_aiKLT16[16][16] =
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
};

extern TMatrixCoeff g_aiKLT16HP[16][16] =
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
};

extern TMatrixCoeff g_aiKLT32[32][32] =
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
};

extern TMatrixCoeff g_aiKLT32HP[32][32] =
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
};

extern TMatrixCoeff g_aiKLT64[64][64] =
{
{  19,  21,  23,  24,  26,  28,  30,  32,  34,  36,  38,  40,  43,  44,  46,  47,  48,  50,  51,  53,  55,  56,  57,  59,  62,  64,  65,  66,  67,  68,  69,  71,  71,  72,  73,  73,  74,  75,  75,  76,  76,  77,  77,  76,  77,  78,  78,  79,  79,  79,  79,  79,  79,  78,  78,  77,  77,  76,  75,  74,  73,  73,  72,  71,},
{ -50, -55, -60, -64, -69, -73, -76, -80, -83, -86, -88, -89, -90, -91, -91, -91, -91, -89, -88, -87, -83, -80, -78, -74, -69, -64, -58, -52, -48, -42, -36, -30, -24, -18, -11,  -3,   3,   9,  14,  19,  25,  29,  34,  38,  43,  47,  51,  54,  57,  60,  62,  64,  67,  69,  70,  71,  71,  70,  70,  69,  68,  66,  65,  64,},
{ -59, -65, -70, -74, -77, -79, -81, -81, -80, -78, -74, -70, -64, -57, -50, -42, -32, -23, -15,  -5,   3,  13,  24,  35,  45,  54,  65,  73,  78,  84,  88,  90,  92,  92,  91,  89,  86,  81,  75,  69,  62,  55,  47,  38,  28,  19,   8,  -3, -12, -20, -29, -37, -45, -53, -60, -66, -72, -77, -80, -82, -83, -85, -84, -81,},
{ -75, -83, -88, -91, -93, -92, -88, -82, -73, -61, -47, -32, -17,  -2,  13,  28,  42,  54,  64,  72,  78,  85,  89,  90,  89,  83,  76,  67,  56,  43,  30,  14,  -2, -19, -34, -46, -58, -67, -74, -81, -85, -88, -88, -84, -78, -71, -64, -54, -43, -30, -19,  -7,   4,  17,  29,  39,  49,  58,  64,  70,  74,  76,  76,  72,},
{ -79, -86, -88, -86, -80, -72, -59, -42, -24,  -5,  14,  32,  48,  61,  72,  81,  86,  88,  84,  79,  71,  60,  45,  26,   5, -18, -40, -57, -71, -82, -88, -92, -91, -85, -75, -60, -45, -28, -10,  10,  28,  44,  57,  68,  80,  87,  88,  87,  84,  77,  65,  53,  39,  23,   6, -11, -27, -43, -56, -68, -75, -78, -79, -76,},
{ -76, -82, -82, -77, -67, -54, -36, -15,   8,  33,  55,  71,  83,  89,  88,  83,  70,  52,  32,  11, -12, -34, -54, -72, -89, -95, -90, -80, -65, -46, -26,  -3,  22,  45,  62,  77,  84,  88,  85,  77,  65,  49,  29,  10, -14, -38, -57, -75, -86, -92, -93, -88, -78, -63, -42, -22,  -3,  18,  37,  54,  69,  80,  85,  83,},
{ -82, -86, -81, -71, -53, -31,  -5,  26,  53,  75,  90,  97,  94,  84,  62,  35,   4, -26, -51, -73, -87, -96, -95, -83, -59, -30,   1,  30,  57,  77,  86,  89,  84,  68,  47,  24,   0, -26, -50, -70, -81, -85, -82, -74, -57, -35,  -8,  19,  40,  58,  72,  79,  80,  76,  68,  52,  32,  12, -10, -33, -55, -73, -82, -83,},
{ -70, -73, -64, -50, -30,  -4,  23,  48,  65,  76,  80,  71,  52,  28,  -1, -31, -55, -78, -86, -82, -71, -50, -25,   9,  44,  71,  87,  91,  84,  64,  37,   6, -29, -61, -84, -99, -95, -79, -55, -23,  10,  42,  69,  89,  94,  87,  75,  53,  28,  -5, -42, -66, -82, -93, -94, -84, -65, -41, -10,  22,  51,  75,  88,  93,},
{ -99, -97, -79, -49, -10,  31,  64,  88, 100,  93,  74,  42,   4, -35, -65, -84, -92, -89, -67, -33,   5,  44,  71,  92,  98,  82,  51,  13, -30, -60, -80, -88, -79, -60, -28,  11,  42,  70,  83,  80,  66,  47,  21,  -7, -40, -67, -80, -81, -70, -54, -29,   1,  32,  61,  78,  82,  77,  59,  31,   6, -26, -55, -70, -76,},
{ -97, -92, -65, -30,  15,  52,  76,  88,  84,  62,  24, -15, -48, -75, -84, -75, -52, -17,  18,  45,  70,  81,  70,  47,  12, -28, -63, -84, -83, -65, -34,   6,  47,  80,  94,  85,  62,  25, -14, -52, -79, -90, -84, -63, -30,  10,  43,  67,  80,  82,  67,  40,   6, -29, -60, -84, -93, -87, -67, -34,  12,  63,  95, 108,},
{ -97, -90, -57, -11,  40,  76,  94,  91,  65,  25, -25, -71, -92, -94, -72, -27,  24,  67,  92,  90,  67,  35,  -6, -51, -90, -95, -72, -33,  17,  55,  75,  84,  70,  41,   2, -45, -77, -85, -77, -53, -12,  27,  64,  88,  83,  62,  24, -15, -48, -69, -70, -66, -46, -11,  24,  56,  69,  80,  77,  52,  13, -42, -78, -96,},
{ -93, -78, -38,  10,  60,  85,  86,  61,  20, -29, -68, -84, -74, -43,   1,  50,  79,  88,  73,  29, -22, -66, -87, -78, -58, -14,  44,  77,  85,  68,  32, -10, -49, -83, -87, -64, -27,  26,  67,  85,  82,  58,  14, -28, -66, -87, -87, -59, -17,  26,  63,  86,  83,  61,  26, -21, -58, -90,-101, -78, -23,  31,  77, 105,},
{ -99, -73, -22,  32,  76,  92,  78,  31, -27, -75, -86, -68, -31,  14,  53,  78,  77,  58,  11, -40, -72, -87, -63, -26,  27,  83, 100,  73,  14, -45, -83, -96, -72, -16,  35,  71,  86,  69,  30, -15, -51, -76, -75, -56, -10,  39,  74,  88,  66,  32,  -7, -47, -77, -88, -69, -30,  23,  71,  94,  98,  54, -16, -73, -98,},
{ -84, -55,  -2,  46,  77,  73,  40,  -7, -54, -78, -67, -29,  12,  47,  72,  77,  48,  -4, -53, -84, -85, -46,   9,  69,  98,  75,  18, -45, -80, -80, -53,  -1,  57,  95,  84,  41, -11, -67, -94, -84, -41,   8,  54,  84,  86,  57,   5, -57, -87, -85, -61, -14,  35,  82,  99,  76,  30, -44, -96,-103, -74,   3,  65,  89,},
{ -95, -64,  -2,  57,  91,  84,  35, -30, -82, -89, -51,   4,  52,  80,  76,  32, -28, -73, -85, -56,  -9,  50,  79,  72,  41,  -6, -75,-101, -71, -12,  57, 102,  90,  40, -39,-100, -94, -42,  14,  61,  82,  69,  30, -15, -61, -86, -73, -24,  37,  75,  81,  58,  15, -38, -78, -87, -56,  12,  73,  89,  67,  13, -48, -84,},
{ -69, -43,   6,  49,  68,  60,  15, -38, -67, -55, -14,  24,  40,  50,  36,  -7, -42, -60, -49,  -5,  43,  66,  46,  19, -16, -66, -91, -45,  39,  89,  94,  49, -29, -96, -97, -42,  25,  80,  93,  60,  -7, -64, -93, -83, -22,  44, 101,  98,  52,  -9, -88,-117, -86, -21,  62, 113,  96,  39, -38,-108,-107, -28,  47,  88,},
{ -95, -54,  23,  83,  98,  53, -21, -79, -90, -59,  11,  77,  92,  57,   4, -57, -97, -78, -10,  57,  95,  82,  15, -58, -99, -71,   0,  68,  96,  64,  -3, -77, -99, -64,  13,  97, 105,  34, -49, -90, -78, -27,  40,  82,  65,  25, -15, -61, -80, -53,  11,  64,  86,  60,  -5, -59, -74, -50,   2,  55,  59,  15, -15, -37,},
{ -16,  32,  15,   2, -47, -76,  23,  76,  75,   3, -94, -47,  10,  27,  39, -17, -19,  44,  27, -11, -56,-102,  19, 103,  85,  36,-128,-155,  40, 104, 103,  40,-125,-109,  16,  62,  80,   1, -81,  -4,  23,  18,  12, -71, -16,  76,  48,  13, -74,-110,  11,  74,  86,  44, -76,-114,  -7,  47,  63,  43, -61, -49,  21,  23,},
{  95,  13, -46, -69, -45,  19,  23,  36,  39,  17,   3, -62, -80, -42,  29, 119, 119,  -3,-102,-132, -75,  74, 123,  59, -22, -65, -53, -10,   5,  37,  52,  26,  11, -65, -93, -14,  36, 100,  97, -22, -89,-103, -52,  62,  90,  58,   4, -54, -51,  -8, -11,   3,  27,  50,  58,   8, -86, -94, -32,  46, 120,  63, -38, -80,},
{ 105,  34, -60,-108, -89,   6, 108, 127,  50, -81,-133, -78,   4,  86, 114,  57, -28, -93,-105, -36,  57, 116,  81, -28, -89, -85, -17,  81,  87,  29, -46, -91, -59,  40,  90,  53,  -7, -66, -66, -19,  25,  58,  46,   7, -29, -58, -35,  12,  36,  35,  19,  -4, -27, -42, -26,  14,  42,  51,  13, -42, -55, -28,  14,  50,},
{  50,  28, -28, -66, -53,  -7,  66,  95,  26, -77,-110, -33,  61, 106,  61, -45,-114, -70,  26,  96,  97,   2, -90, -93,  -2,  78,  79,   0, -72, -74, -11,  61,  64,  32, -36, -82, -35,  20,  49,  61,  26, -39, -69, -66,  -2,  75, 100,  46, -46,-111, -88,  -8,  71, 102,  57, -16, -77, -90, -12,  51,  56,  43, -14, -54,},
{  32,   7, -18, -29, -11,   2,   4,  29,  24,   1, -23, -39, -27,   7,  47,  50, -22, -68, -18,  55,  83,   9,-113, -81,  55, 111,  50, -85,-114, -22,  85, 118,  21,-142,-117,  66, 132,  72, -36,-120, -97,  11, 105, 124,   9,-120, -96,  -3,  79,  79,   3, -44, -31, -23,   2,  10,  11,  16,  16,  -3, -27, -33,   5,  34,},
{ 104,  17, -86,-102, -30,  83, 113,  21, -84,-114, -13,  93,  89,  22, -59, -91, -27,  44,  71,  33, -47, -45,  -7,   8,  23,  12,  -9,  -1, -14, -13,   7,   8,  25,  24, -47, -58,  -3,  59,  79,   8, -64, -91, -44,  58, 117,  69, -40,-131, -95,  45, 117,  92,  -5,-107, -99,   0,  80,  79,  18, -40, -70, -32,  17,  48,},
{  55,  12, -45, -60, -18,  50,  64,  16, -42, -73, -19,  54,  59,  30, -37, -75, -32,  30,  73,  50, -50, -80, -31,  52,  92,  10, -99, -60,  41,  73,  41, -38, -74, -36,  45,  87,  15, -66, -57,  -4,  41,  41,  17,  -2, -28, -46, -27,   5,  54,  74,  13, -70, -99, -20,  96, 121,   9,-125,-119,  -2, 124, 151,  -7,-151,},
{-117,   4, 102,  96,   1,-115,-105,  38, 139,  69, -77,-117, -33,  71, 100,  17, -70, -69,  -6,  54,  46, -13, -46,  -8,  33,  10, -30, -10,  33,  15, -16, -22,  -7,  30,  13, -41, -25,  25,  36,  30, -10, -69, -51,  33,  80,  65, -27,-121, -68,  66, 125,  52, -83,-120, -14,  82,  81, -13, -76, -35,  31,  70,  20, -70,},
{ -54,   5,  62,  46, -31, -81, -13,  78,  54, -45, -82, -20,  66,  77,  -8, -89, -50,  45,  88,  31, -90, -98,  55, 116,   8,-104, -36,  61,  33,   1, -38, -42,  35,  59, -16, -79, -19,  81,  73, -26, -89, -69,  45, 117,  23, -81,-102,  22, 129,  44, -64, -99, -27, 112,  61, -46, -67, -23,  63,  53, -44, -60,   2,  45,},
{  75, -11, -79, -50,  55,  79, -21, -72, -28,  57,  72, -31, -99, -15,  71,  64, -26, -74,  -9,  57,  65,  -5,-144, -34, 174,  45,-141, -71,  94, 105, -27,-119, -41,  81,  91, -32,-106,  -2,  90,  31, -29, -64, -33,  73,  36, -49,  -5,   9,  15,   0, -43,  -4,  34,  26,  20, -33, -65,  -5,  68,  44, -38, -55,  -7,  44,},
{ 108, -23,-130, -54, 108, 100, -55,-117,   9, 132,  17,-133, -57,  97,  98, -15,-110, -39,  75,  67, -34,-125,  28, 160, -39,-130,  44,  83, -19, -62, -17,  51,  45, -60, -23,  29,  12,  13, -24, -19,   7,   3,  17,   7, -35,   0,  17,  23,  14, -43, -26,  37,  29,  -6, -43, -32,  57,  52, -47, -46,  21,  40,  13, -37,},
{ -39,   9,  56,  15, -60, -26,  32,  33,  -4, -44,  -1,  45,   3, -39, -28,  25,  46, -16, -25,   5,  17,  11, -51,  16,  62, -30, -50,   9,  35,  11, -21, -34,  21,  65, -26, -78,  -1,  77,  56, -56, -95,   8,  87,  56, -57,-116,  42, 126,   7,-104, -83,  69, 138, -30,-137, -18, 103,  85, -69,-134,  26, 168,  34,-132,},
{ -71,  27,  90,  -3, -84, -26,  75,  49, -65, -59,  72,  49, -54, -67,  27,  83,  -6, -76, -12,  53,  43, -50, -53,  65,  37, -81, -12,  80, -10, -58,  -7,  59,  33,-111,  -1, 155, -34,-140,   0, 120,  62, -86,-111,  28, 103,  37, -79, -60,  71,  49, -71, -46,  62,  69, -44, -83,  22,  69,  -6, -39,  -7,  33,  18, -35,},
{  22, -18, -34,  15,  42,  11, -42, -41,  45,  41, -56, -24,  44,  34, -19, -43, -10,  49,  22, -32, -59,  45,  98, -95,-105, 108,  94, -73, -96,  21, 122,  24,-152, -40, 164,  33,-122, -64,  87,  96, -42,-101, -21,  82,  65, -51, -84,  42,  73, -33, -56,  12,  27,  27, -29, -32,  26,  47,  -8, -70,  -9,  66,  24, -48,},
{ -99,  74, 105, -55, -98,   2, 114,   8,-115,  12,  79,  -5, -40, -46,  47,  81, -58, -69,  29,  62, -11, -80,  33,  89, -76, -28,  78, -22, -60,   6,  62,  24, -83, -25, 110, -19, -92,  37,  63, -15, -62, -12,  54,  42, -59, -46,  47,  55, -29, -77,  31, 100, -41, -99,  35,  80,  12, -84, -45,  84,  57,-102, -46,  80,},
{ -22, -13,  39,  18, -31,   7, -12,   0,  23,  -2,   8, -31, -47,  92,  34, -97, -28,  48,  90, -43,-141, 120,  36,-118, 104,   3,-110,  91,  33, -71, -37,  41,  99,-110, -50, 154, -49, -91,  65,  14, -16,  -1, -16,  46,   0, -94,  55,  74, -51, -46, -21,  71,  59,-102, -35, 103, -19, -71,  38,  41, -14, -34, -32,  51,},
{  84, -63, -96,  50, 126, -48,-150,  88, 124,-120, -79,  95,  75, -69, -74,  48,  68, -30, -65,  25,  68, -47, -63,  78,  40, -82,  -1,  61, -44, -36,  49,  27, -30, -38,  42,  53, -68, -48,  59,  33, -32, -18,  13,  29, -39, -37,  65,  15, -42,   5,   3,  -2,  34, -20, -58,  29,  79, -54, -80,  83,  67, -93, -61,  81,},
{ -28,  45,   3, -35,   9, -11,  39,   1, -58,  50,   2, -77,  54,  72, -38, -82,  -8, 166, -32,-218, 154,  98,-169,  51,  66, -69,   0,  12,  28, -22, -43,  73,  -7, -79,  87, -19, -55,  68, -17, -13,  11, -29,  19,   1,  -8,  59, -69, -36,  99, -42, -46,  69, -24, -21,  25, -33,  53,   0, -80,  43,  49, -45,  -6,  12,},
{  14, -49,  36,  31, -44,   3,   8,   8,  -2,  -3,  21, -38, -17,  75, -33, -44,  40,  11, -13, -23,  40,   7, -90,  52,  85,-126,  31, 117,-154, -26, 165, -13,-123,  17,  56,   2, -31,   4,  50, -70, -20,  86, -15, -34,  -4,   0,  74, -43, -86,  83,  -4, -15,  27, -54,  63,  41,-167,  49, 151,-102, -57,  72,  11, -28,},
{  -3,  16,  -9, -21,  53, -37, -45,  86, -11, -73,  39,  27,  -8, -37,  23,  55, -81, -27, 106, -25, -79,  64,  11, -37,  -7,  27,   9, -44,  45,  20, -85,  42,  62,-134, 102,  45,-157,  89,  78,-104, -28,  48,  56,  -6,-118,  45, 114, -89, -60,  89,  27, -55, -52,  64,  67,-119, -19, 121, -27, -63,  18,  25,  20, -32,},
{  74, -59, -94,  97,  45, -96,  24,  51, -43, -41,  43,  72, -62, -69,  65,   9, -17,  30, -42,  -7,  54, -42,   7,  23, -34,  10, -21,  25,  93,-108, -66, 131, -29, -59,  66, -31, -13,  29, -18,  42, -32, -50,  61,  -2,   0,  -3, -42,  51,   7, -89,  77,  51, -81, -20,  31,  69, -52, -98, 134,  20,-164,  83, 122,-110,},
{ -35,  50,  20, -73,  31,  32, -42,   0,  33,  18, -79, -26, 127, -10,-107,  45,  34,  -7, -27, -20,  68, -45, -37,  87, -47, -19,  80, -96,  17, 103, -92, -57, 131, -48, -74, 104, -14, -77,  20,  59,   6, -53, -24,  69,   1, -75,  38,  63, -36, -72,  32, 101, -66, -92, 130,   3,-138,  72,  80, -63, -30,  16,  47, -35,},
{ -69,  72,  55, -73, -82, 133,  42,-173,  78,  85,-114,   4,  65,  -2, -48, -23,  92, -10, -97,  55,  41, -36,   3, -27,  29,  16, -84,  74,  66,-117, -14,  94, -28, -41,  42,  12, -43, -16,  56,  25, -86,  -5,  76, -12, -35,  -7,  36,  24, -90,  16, 124, -89, -84, 128, -13, -61,  34, -12,   7,  14, -30,  17,   8, -10,},
{  16, -17, -24,  49,  -8, -59,  47,  38, -66,  -5,  71,  -6, -94,  34,  99, -94, -34, 107, -22, -74,  41,  30, -38,  20,  -5, -25,  54, -11, -73,  51,  52, -74,   7,  63, -77,  30,  47, -83, -15, 105,  -6,-113,  36, 113, -74, -98, 116,  51,-139,  12, 136, -77, -85, 118,  -4, -88,  59,  27, -57,  16,  12, -11,  19, -13,},
{  44, -70, -12,  88, -47, -51,  79,  -5, -64,  49,  30, -82,  -5, 106, -40, -85,  99,   1,-100,  55,  58, -71,  15,  35, -71,  38,  61,-101,  11, 100, -69, -57,  93, -47, -17, 102, -97, -31, 104, -12, -91,  29,  83, -52, -64,  86,   8, -79,  14,  75, -43, -65, 100,   4,-106,  71,  39, -84,  29,  37, -41,   9,  15, -11,},
{  77,-109, -23, 114, -26, -81,  53,  46, -69,   2,  57, -60, -13,  92, -49, -61,  95, -29, -87,  97,  21, -76,  49, -36,   3,  69, -85,  20,  55, -61,  -7,  38,  -6, -10,   4,   7, -11,  -2,  31, -32, -12,  49, -45, -15,  64, -25, -45,  67,   1, -80,  42,  65, -89, -15, 126, -97, -45, 147, -87, -78, 136, -31, -93,  72,},
{ -50,  77, -21, -36,  25,  29, -39, -16,  68, -59, -12, 106, -91, -63, 158, -55,-103, 120, -26, -57,  82, -58,   8,  32, -51,  51, -28,  -3,  20, -27,  30, -30,   3,  55, -76,  49,  15,-112, 128,  27,-177,  90,  92,-120,  25,  72, -67,  -9,  38,  -6, -27,  12,  16, -46,  51,   7, -64,  60, -12, -53,  63,   5, -54,  30,},
{  19, -30,   7,  20, -15, -12,  19,   1, -26,  32,  -4, -37,  37,   4, -25,  26, -31,   1,  54, -50,  16,  -6, -21,  56, -61,  52, -15, -63, 134, -73, -81, 127, -71,   4,  51, -92, 106, -50, -66, 130, -34, -93,  56,  54, -41, -49,  83, -41, -67, 134, -60, -82, 125, -59, -16,  79, -88,  35,  40,-102,  98,   1,-113,  76,},
{ -57,  70,  28, -81,  23,  57, -78,  27,  50, -94,  58,  45,-111,  62,  59,-126,  48,  70, -84,   3,  68, -68,  13,  58, -81,  31,  40, -67,  42,  13, -58,  42,  11, -59,  70, -22, -52,  97, -30,-102, 103,  58,-151,  34, 126,-118,  -6,  87, -87,  38,  28, -81,  65,  14, -58,  40,  -2, -20,  21, -19,   8,  13, -28,  17,},
{  72,-111,   3, 105, -82, -15,  72, -66,  14,  48, -73,  21,  57, -73,  14,  93,-113,  -8, 107, -82,   0,  55, -80,  56,  10, -47,  58, -45, -12,  58, -38, -20,  45, -34,  11,  24, -47,  21,  29, -36, -16,  70, -37, -80, 125, -18, -98, 120, -54, -58, 128, -85, -21,  79, -80,  41,  10, -33,  45, -66,  50,  37,-112,  66,},
{  71,-117,  19, 111,-121,   9, 107,-144,  78,  49,-140, 123,  -2,-110, 111, -33, -52,  84, -48,  -3,  36, -45,  26,   2, -22,  18,  -3,   0,  -2,   2,  -4,  -9,  26, -15,   0,  16, -45,  58, -18, -67,  94, -16, -69,  81, -28, -53,  91, -65,   6,  75,-112,  54,  32, -79,  77, -32, -16,  43, -63,  54,  -3, -47,  58, -27,},
{ -49, 115,-101,   0, 105,-132,  67,  46,-132, 132, -39, -78, 117, -57, -29,  72, -67,  37,   3, -34,  33, -31,  34,  -8, -22,  37, -53,  34,  33, -65,  37,   2, -26,  42, -39,  23,  -2, -41,  82, -63, -31, 120,-101, -15, 109,-106,  37,  34, -80,  88, -25, -60,  64, -16,   2, -10,  18,   2, -53,  73, -38, -28,  89, -57,},
{  73,-147, 118, -10, -87, 107, -71,  10,  44, -68,  63, -30, -33,  67, -48,  17,  11, -58,  92, -59, -10,  53, -63,  73, -58,   3,  51, -80,  78, -33, -32,  72, -75,  55, -19, -22,  62, -98,  81,   1, -81,  97, -32, -45,  68, -48,  12,  13, -19,  23, -20,  16, -13,  -8,  45, -63,  44,  12, -88, 124, -62, -65, 144, -80,},
{ -32,  76, -76,  24,  38, -65,  65, -44,   7,  29, -57,  67, -45, -11,  60, -85,  88, -41, -37,  98,-116,  83, -27, -31,  98,-139, 109, -29, -60, 117,-115,  59,  17, -75,  97, -97,  73, -27, -35,  99,-106,  44,  26, -81,  86, -26, -26,  40, -44,  42, -19, -10,  32, -50,  56, -18, -32,  52, -67,  45,  13, -40,  42, -23,},
{ -45,  98, -93,  39,  33, -76,  72, -51,  24,  19, -58,  75, -68,  37,   3, -47,  81, -78,  41,   7, -52,  78, -79,  64, -25, -35,  83, -97,  87, -48, -18,  73, -95,  87, -73,  43,  18, -73, 100, -95,  40,  46, -91,  74, -41,  -9,  61, -79,  65, -18, -46,  95, -98,  61, -13, -33,  55, -61,  70, -39, -16,  71,-105,  54,},
{  31, -74,  77, -41,  -8,  45, -64,  68, -56,  26,  16, -50,  66, -68,  43,   5, -58,  90, -68,  15,  36, -69,  75, -66,  53, -29,  -1,  22, -55,  89, -77,  30,  24, -61,  66, -80,  95, -69,  22,  30, -86, 105, -72,  16,  31, -68,  96, -79,  22,  28, -77, 112, -99,  63, -11, -62, 101, -96,  75, -23, -43, 102,-114,  48,},
{ -45, 101,-125, 108, -52, -14,  66,-100, 116,-102,  62,   8, -82, 125,-132,  94, -24, -46,  85, -91,  77, -52,  29,  10, -45,  58, -53,  44, -42,  18,  34, -72,  80, -61,  45, -34,   5,  37, -69,  73, -50,  10,  27, -46,  62, -62,  37,  -1, -38,  63, -70,  60, -28, -16,  55, -73,  63, -38,   0,  42, -64,  69, -59,  24,},
{  30, -72,  95, -93,  66, -19, -33,  69, -90,  97, -84,  53, -13, -31,  58, -74,  79, -62,  28,   5, -30,  49, -56,  60, -57,  29,  -6,  -8,  34, -62,  73, -59,  21,  17, -38,  61, -83,  90, -80,  52,  -4, -55,  95, -96,  74, -23, -30,  56, -78,  87, -71,  34,   7, -55,  97,-104,  76, -41,   4,  48, -90, 118,-114,  47,},
{  19, -42,  57, -68,  69, -56,  28,   8, -41,  71, -95,  93, -64,  19,  26, -65,  98,-108,  96, -69,  31,   8, -46,  70, -90, 109, -99,  74, -46,   1,  50, -86,  96, -89,  79, -68,  45,  -9, -21,  47, -68,  73, -56,  17,  13, -39,  67, -76,  69, -49,  17,  23, -55,  76, -84,  66, -32,  -1,  32, -62,  85, -99,  92, -39,},
{ -23,  60, -87,  99, -95,  76, -49,  19,  17, -48,  79,-110, 112, -86,  60, -33, -12,  59, -90, 114,-119, 116,-113,  87, -57,  25,  12, -40,  69, -98, 101, -82,  49, -18,   0,  23, -50,  78, -99,  95, -79,  65, -33, -16,  41, -47,  53, -49,  33,   0, -31,  44, -45,  42, -29,  16,  -7, -11,  33, -37,  38, -44,  36, -13,},
{  19, -49,  70, -81,  79, -68,  53, -37,  25,  -9, -13,  45, -74,  85, -81,  75, -71,  63, -41,  16,   7, -37,  61, -72,  79, -75,  65, -55,  42, -19, -13,  41, -63,  80, -89,  97,-106, 105, -90,  69, -42,  -4,  59, -98, 107, -93,  77, -57,  19,  21, -44,  55, -62,  64, -54,  26,   5, -29,  51, -82, 103, -97,  73, -29,},
{  22, -52,  75, -83,  81, -77,  68, -55,  37, -17,  -1,  21, -40,  53, -59,  58, -61,  68, -61,  45, -40,  33, -19,  11,  -4, -11,  22, -28,  37, -46,  44, -32,  21, -12,   6,   0, -13,  37, -64,  89,-106, 113,-114,  93, -54,   8,  41, -77,  99,-113, 117,-108,  83, -55,  27,  11, -44,  63, -80, 101,-115, 115, -93,  37,},
{ -21,  49, -70,  84, -94,  96, -83,  72, -61,  43, -26,   3,  24, -48,  67, -82, 100,-119, 131,-125, 107,-102, 103, -94,  81, -70,  57, -40,  24,  -8, -10,  27, -41,  47, -51,  64, -80,  86, -86,  83, -74,  59, -42,  20,   8, -33,  54, -68,  70, -63,  56, -45,  27, -11,  -5,  23, -35,  41, -43,  34, -24,  25, -21,   8,},
{  -8,  19, -24,  23, -24,  29, -30,  25, -22,  21, -16,  11, -12,  15, -18,  19, -21,  21, -24,  35, -44,  55, -73,  89,-102, 116,-126, 138,-156, 172,-169, 148,-119,  92, -82,  80, -73,  63, -49,  35, -26,  13,  -4,  -5,  15, -20,  22, -19,  15,  -7,  -7,  20, -27,  32, -37,  45, -54,  54, -50,  46, -42,  36, -27,  12,},
{  -4,   6,  -6,  11, -17,  22, -28,  32, -32,  28, -20,  17, -20,  18, -19,  20, -17,  19, -21,  12,   0,  -5,   4,  -6,  12, -24,  33, -38,  41, -45,  48, -43,  42, -42,  39, -37,  37, -43,  53, -67,  81, -91,  99, -94,  85, -74,  59, -49,  23,   7, -36,  71, -97, 117,-134, 149,-157, 153,-143, 123, -95,  77, -65,  29,},
{  -9,  23, -39,  60, -79,  90, -98, 109,-111, 112,-126, 137,-129, 113,-101,  90, -73,  58, -47,  30, -14,   5,   1,  -9,  16, -21,  23, -28,  35, -34,  27, -24,  21, -15,  10,  -8,   1,  18, -35,  48, -60,  72, -85,  96,-104, 106,-100,  94, -87,  73, -60,  50, -40,  32, -27,  20, -13,   8,  -4,  -2,  12, -20,  20,  -8,},
{ -16,  34, -46,  61, -75,  80, -80,  80, -80,  79, -80,  80, -77,  68, -62,  52, -46,  50, -48,  42, -37,  29, -22,  24, -19,   6,  -4,   9,  -8,   5,   1,  -8,  14, -21,  26, -27,  30, -40,  51, -57,  63, -72,  78, -87,  94, -98, 108,-118, 118,-113, 111,-108,  98, -90,  88, -86,  73, -59,  49, -39,  29, -17,  10,  -5,},
};

extern TMatrixCoeff g_aiKLT64HP[64][64] =
{
{  74,  82,  91,  98, 106, 114, 122, 129, 138, 145, 151, 161, 170, 178, 184, 188, 192, 199, 206, 213, 219, 224, 229, 237, 247, 256, 260, 263, 269, 273, 278, 283, 286, 289, 292, 294, 296, 299, 300, 303, 306, 306, 306, 305, 309, 313, 314, 315, 316, 317, 317, 316, 315, 313, 311, 309, 307, 303, 300, 298, 293, 292, 288, 283,},
{-200,-222,-241,-258,-275,-291,-305,-320,-330,-344,-351,-356,-361,-364,-364,-364,-363,-357,-353,-346,-333,-321,-313,-298,-275,-254,-232,-210,-190,-168,-144,-120, -97, -70, -42, -13,  13,  34,  54,  78,  99, 117, 135, 152, 171, 189, 204, 217, 227, 239, 248, 257, 267, 275, 281, 284, 284, 282, 280, 275, 271, 265, 259, 254,},
{-235,-260,-281,-298,-308,-316,-322,-323,-321,-310,-296,-281,-258,-230,-201,-168,-130, -93, -58, -21,  14,  53,  94, 139, 180, 218, 261, 292, 313, 335, 351, 361, 366, 369, 364, 356, 344, 325, 300, 276, 249, 222, 189, 154, 112,  75,  33, -11, -47, -82,-115,-147,-181,-211,-241,-265,-287,-308,-321,-328,-330,-340,-336,-326,},
{-298,-331,-351,-366,-370,-367,-353,-327,-292,-243,-188,-128, -68, -10,  53, 111, 167, 216, 255, 287, 313, 339, 356, 361, 354, 334, 304, 267, 222, 172, 120,  57,  -9, -76,-134,-184,-231,-268,-298,-325,-340,-350,-351,-338,-314,-283,-255,-217,-172,-119, -74, -30,  15,  67, 115, 157, 197, 234, 257, 279, 294, 306, 303, 290,},
{-314,-343,-352,-343,-321,-289,-237,-169, -96, -18,  57, 130, 192, 245, 289, 323, 345, 350, 337, 316, 283, 238, 180, 103,  19, -71,-162,-227,-286,-328,-351,-368,-363,-342,-299,-239,-180,-110, -38,  38, 112, 176, 229, 274, 318, 349, 354, 350, 335, 307, 261, 213, 155,  92,  26, -43,-110,-170,-226,-272,-300,-313,-314,-304,},
{-302,-326,-327,-307,-269,-218,-145, -59,  33, 131, 221, 286, 332, 357, 352, 330, 282, 207, 130,  44, -48,-138,-217,-290,-356,-382,-361,-319,-259,-185,-104, -10,  88, 179, 250, 306, 338, 350, 341, 308, 260, 197, 118,  39, -55,-152,-228,-300,-346,-369,-371,-350,-310,-251,-168, -88, -12,  73, 148, 217, 276, 320, 341, 331,},
{-326,-345,-323,-283,-214,-126, -20, 106, 213, 299, 360, 390, 377, 335, 250, 139,  17,-105,-203,-290,-349,-382,-382,-331,-238,-122,   3, 122, 228, 306, 345, 357, 336, 272, 187,  97,  -2,-105,-202,-278,-323,-338,-330,-297,-227,-138, -30,  75, 161, 234, 287, 316, 322, 306, 272, 210, 127,  48, -39,-133,-218,-290,-329,-334,},
{-278,-293,-256,-200,-118, -16,  92, 194, 261, 305, 321, 285, 210, 112,  -5,-123,-222,-310,-343,-328,-283,-199,-101,  38, 176, 283, 347, 365, 338, 255, 149,  24,-116,-244,-337,-394,-378,-318,-218, -94,  41, 167, 276, 355, 374, 348, 301, 214, 110, -22,-167,-263,-327,-373,-377,-337,-259,-162, -38,  88, 203, 299, 353, 371,},
{-394,-389,-316,-197, -39, 124, 254, 353, 399, 374, 294, 167,  14,-139,-258,-337,-368,-354,-268,-131,  21, 176, 286, 369, 393, 327, 205,  53,-119,-241,-319,-352,-317,-241,-111,  44, 169, 279, 332, 319, 263, 188,  83, -29,-162,-269,-321,-324,-280,-215,-114,   5, 128, 245, 310, 327, 309, 237, 126,  22,-105,-222,-279,-305,},
{-388,-366,-261,-120,  59, 207, 304, 353, 337, 248,  94, -61,-190,-300,-336,-300,-208, -68,  73, 181, 280, 323, 280, 188,  48,-113,-250,-336,-330,-259,-134,  25, 189, 321, 377, 340, 247, 101, -58,-208,-314,-361,-336,-253,-122,  39, 172, 268, 321, 329, 266, 160,  25,-117,-241,-338,-371,-348,-270,-136,  47, 253, 379, 431,},
{-386,-359,-227, -44, 160, 303, 378, 362, 258,  98,-101,-283,-367,-374,-287,-107,  95, 266, 368, 359, 269, 140, -25,-204,-358,-382,-287,-130,  66, 221, 302, 336, 282, 162,   7,-181,-310,-340,-309,-210, -49, 110, 255, 353, 331, 246,  96, -60,-194,-277,-282,-262,-182, -46,  96, 222, 277, 320, 307, 207,  52,-168,-312,-386,},
{-373,-312,-151,  40, 241, 342, 344, 244,  79,-117,-273,-336,-295,-174,   5, 199, 315, 351, 291, 115, -89,-263,-348,-313,-233, -57, 174, 308, 339, 273, 127, -40,-196,-331,-349,-255,-107, 102, 269, 341, 327, 234,  58,-112,-265,-346,-346,-237, -66, 102, 250, 342, 333, 243, 103, -85,-233,-358,-406,-311, -94, 125, 308, 420,},
{-394,-292, -89, 127, 303, 369, 312, 125,-106,-299,-342,-274,-123,  57, 213, 310, 307, 232,  45,-158,-290,-347,-253,-103, 109, 333, 402, 293,  56,-180,-331,-384,-288, -63, 141, 282, 346, 275, 120, -60,-206,-303,-300,-223, -40, 157, 296, 354, 265, 129, -29,-189,-306,-351,-278,-120,  91, 284, 378, 392, 215, -63,-292,-390,},
{-335,-220,  -9, 183, 309, 293, 159, -30,-214,-313,-267,-118,  48, 186, 287, 309, 193, -15,-213,-337,-340,-185,  35, 274, 392, 302,  72,-181,-320,-321,-214,  -5, 230, 378, 335, 162, -45,-267,-376,-336,-165,  32, 217, 338, 342, 227,  20,-227,-348,-342,-243, -56, 140, 327, 395, 304, 119,-177,-382,-413,-295,  11, 261, 357,},
{-379,-255,  -7, 226, 366, 334, 139,-121,-327,-357,-205,  14, 207, 321, 305, 126,-111,-291,-340,-225, -34, 202, 318, 288, 165, -24,-301,-406,-286, -50, 229, 409, 359, 161,-154,-400,-377,-170,  55, 243, 329, 275, 118, -59,-243,-342,-293, -97, 149, 299, 326, 231,  59,-150,-312,-347,-223,  48, 291, 357, 268,  53,-190,-335,},
{-275,-173,  22, 196, 274, 238,  60,-153,-267,-222, -58,  95, 160, 199, 144, -26,-168,-242,-197, -19, 172, 265, 185,  76, -62,-265,-366,-181, 157, 355, 375, 197,-118,-383,-387,-167, 102, 322, 373, 238, -26,-257,-372,-332, -87, 177, 404, 392, 209, -37,-353,-469,-346, -83, 249, 450, 384, 155,-151,-432,-429,-111, 189, 352,},
{-382,-215,  92, 334, 392, 212, -83,-316,-361,-236,  45, 308, 366, 226,  15,-227,-388,-314, -41, 227, 378, 329,  59,-233,-394,-283,   1, 272, 385, 256, -12,-308,-397,-256,  53, 389, 419, 136,-196,-360,-312,-110, 161, 327, 261, 100, -58,-245,-318,-211,  42, 257, 344, 241, -19,-235,-296,-200,   8, 219, 237,  58, -59,-146,},
{ -63, 128,  61,   8,-190,-304,  91, 306, 298,  13,-377,-188,  39, 106, 157, -68, -75, 175, 108, -43,-225,-407,  76, 410, 339, 144,-510,-621, 162, 416, 413, 158,-502,-437,  66, 250, 322,   5,-325, -17,  94,  72,  50,-283, -65, 306, 191,  51,-295,-442,  43, 296, 344, 176,-306,-454, -26, 188, 250, 172,-243,-197,  82,  90,},
{ 380,  51,-182,-277,-179,  78,  91, 142, 157,  66,  13,-248,-319,-168, 117, 478, 477, -11,-408,-530,-301, 297, 491, 235, -89,-259,-211, -40,  18, 147, 209, 102,  42,-261,-374, -56, 145, 399, 388, -90,-355,-411,-207, 246, 362, 233,  16,-217,-203, -32, -46,  12, 110, 201, 231,  32,-345,-374,-126, 183, 480, 251,-153,-320,},
{ 421, 135,-240,-430,-356,  26, 430, 507, 200,-324,-532,-312,  15, 342, 457, 226,-112,-371,-421,-144, 227, 465, 323,-114,-357,-340, -66, 325, 350, 117,-184,-364,-234, 161, 358, 214, -28,-266,-265, -77,  99, 231, 184,  29,-114,-233,-140,  47, 144, 142,  75, -15,-109,-167,-106,  56, 167, 202,  52,-170,-221,-111,  55, 202,},
{ 202, 111,-111,-263,-214, -29, 264, 380, 104,-308,-440,-132, 245, 423, 242,-178,-458,-279, 106, 383, 388,   9,-362,-371,  -8, 310, 318,   1,-289,-294, -44, 242, 257, 129,-143,-330,-140,  82, 196, 245, 105,-158,-277,-266,  -7, 299, 402, 184,-185,-443,-351, -32, 283, 409, 228, -64,-307,-362, -50, 206, 224, 171, -55,-215,},
{ 130,  29, -72,-118, -42,   7,  17, 117,  94,   4, -94,-157,-109,  28, 188, 201, -87,-271, -71, 221, 331,  36,-453,-323, 220, 445, 200,-341,-454, -88, 342, 474,  84,-567,-467, 263, 527, 290,-145,-480,-388,  44, 419, 497,  37,-481,-382, -11, 314, 315,  12,-178,-124, -93,   8,  41,  45,  63,  65, -10,-110,-131,  21, 138,},
{ 416,  66,-346,-409,-119, 331, 451,  85,-336,-455, -53, 371, 356,  86,-234,-363,-109, 175, 285, 132,-186,-179, -28,  32,  93,  50, -34,  -4, -56, -52,  29,  33, 102,  94,-187,-233, -11, 235, 315,  34,-257,-363,-177, 230, 469, 277,-159,-526,-382, 178, 468, 367, -20,-428,-396,  -1, 319, 315,  70,-161,-280,-127,  67, 192,},
{ 221,  47,-178,-240, -74, 200, 257,  63,-170,-291, -77, 214, 236, 119,-147,-298,-126, 120, 294, 200,-202,-319,-124, 207, 369,  40,-396,-241, 162, 291, 163,-152,-296,-145, 181, 350,  59,-264,-227, -18, 166, 165,  66,  -8,-110,-184,-107,  21, 216, 297,  50,-280,-397, -82, 384, 484,  35,-498,-475,  -6, 497, 603, -26,-606,},
{-469,  15, 410, 385,   2,-460,-422, 153, 556, 275,-308,-469,-134, 285, 399,  69,-281,-275, -23, 216, 185, -52,-185, -33, 132,  42,-121, -39, 134,  59, -63, -87, -28, 119,  51,-162, -99, 101, 145, 120, -41,-276,-206, 132, 318, 259,-108,-484,-274, 265, 501, 208,-333,-479, -55, 327, 325, -52,-302,-142, 124, 281,  81,-281,},
{-217,  21, 248, 183,-124,-325, -50, 313, 214,-178,-329, -79, 265, 307, -32,-356,-202, 178, 351, 123,-359,-392, 219, 466,  33,-417,-142, 244, 132,   4,-153,-167, 138, 238, -63,-318, -78, 323, 294,-104,-355,-276, 180, 469,  94,-326,-409,  87, 515, 178,-254,-395,-110, 447, 243,-186,-267, -92, 253, 212,-175,-240,   8, 181,},
{ 298, -42,-317,-200, 218, 316, -85,-290,-114, 227, 289,-125,-396, -60, 284, 256,-105,-298, -38, 230, 259, -22,-575,-136, 697, 178,-564,-285, 377, 420,-107,-477,-164, 326, 363,-129,-425,  -8, 358, 125,-115,-256,-131, 291, 145,-196, -22,  35,  60,  -1,-170, -15, 138, 104,  79,-133,-260, -19, 273, 177,-152,-218, -27, 175,},
{ 434, -91,-521,-218, 431, 399,-219,-469,  35, 529,  67,-534,-229, 387, 393, -59,-440,-156, 299, 267,-136,-499, 111, 639,-157,-521, 174, 331, -77,-246, -68, 205, 182,-238, -92, 114,  50,  51, -94, -75,  28,  13,  66,  28,-142,  -1,  70,  90,  55,-174,-104, 149, 118, -23,-172,-130, 226, 210,-188,-186,  84, 161,  51,-149,},
{-157,  38, 222,  59,-238,-104, 129, 132, -15,-175,  -2, 178,  13,-155,-112,  98, 183, -65,-100,  19,  69,  45,-205,  63, 248,-121,-200,  37, 141,  43, -85,-135,  84, 259,-104,-313,  -6, 308, 225,-223,-381,  31, 346, 225,-226,-464, 169, 504,  29,-417,-332, 277, 553,-121,-548, -71, 412, 341,-277,-535, 103, 672, 134,-528,},
{-284, 109, 361, -11,-337,-105, 299, 197,-259,-235, 287, 197,-216,-267, 109, 332, -23,-306, -49, 212, 171,-202,-211, 261, 147,-324, -48, 320, -40,-232, -27, 237, 133,-445,  -3, 622,-138,-561,  -1, 480, 247,-343,-443, 114, 411, 148,-315,-238, 286, 196,-284,-185, 246, 277,-176,-334,  88, 276, -23,-158, -28, 133,  72,-140,},
{  87, -74,-137,  59, 167,  42,-168,-164, 178, 166,-224, -96, 175, 136, -74,-172, -41, 196,  88,-129,-235, 181, 393,-379,-420, 430, 375,-292,-384,  85, 489,  98,-610,-162, 655, 132,-490,-257, 347, 384,-167,-406, -84, 330, 262,-202,-337, 170, 294,-133,-223,  50, 108, 106,-116,-129, 104, 188, -33,-279, -34, 264,  97,-192,},
{-394, 298, 419,-220,-391,   9, 457,  33,-461,  47, 316, -22,-160,-183, 188, 325,-230,-278, 115, 247, -44,-319, 133, 355,-305,-112, 311, -88,-239,  23, 248,  96,-331,-100, 439, -76,-369, 147, 252, -60,-248, -47, 215, 166,-234,-185, 188, 219,-116,-307, 125, 400,-166,-396, 142, 321,  46,-335,-179, 336, 229,-409,-183, 320,},
{ -89, -50, 157,  72,-124,  26, -50,  -1,  93, -10,  31,-123,-187, 369, 136,-388,-110, 194, 362,-171,-566, 481, 143,-474, 416,  13,-441, 363, 133,-286,-147, 166, 394,-442,-200, 615,-196,-365, 260,  56, -66,  -4, -62, 182,   2,-378, 222, 295,-204,-184, -84, 282, 235,-407,-139, 411, -78,-284, 151, 164, -54,-134,-130, 202,},
{ 338,-254,-385, 199, 505,-194,-601, 352, 495,-481,-318, 382, 300,-276,-294, 194, 272,-120,-262, 101, 273,-187,-252, 313, 159,-326,  -5, 244,-174,-145, 195, 109,-121,-152, 169, 213,-273,-193, 234, 131,-129, -70,  52, 114,-156,-149, 260,  60,-169,  20,  11,  -7, 137, -80,-232, 117, 316,-215,-322, 332, 268,-372,-244, 323,},
{-113, 180,  13,-142,  35, -43, 155,   4,-232, 200,   7,-307, 214, 286,-151,-329, -30, 664,-127,-870, 615, 391,-677, 204, 265,-276,  -1,  49, 113, -89,-171, 291, -29,-316, 349, -78,-220, 273, -66, -52,  45,-117,  77,   5, -30, 238,-274,-143, 397,-170,-183, 277, -96, -85,  99,-130, 213,  -2,-322, 172, 197,-180, -25,  46,},
{  56,-195, 144, 125,-175,  13,  34,  33,  -7, -14,  84,-154, -67, 300,-132,-176, 162,  44, -53, -92, 160,  28,-361, 208, 338,-503, 123, 468,-616,-106, 658, -51,-490,  66, 224,   9,-126,  17, 200,-282, -82, 342, -59,-138, -15,  -1, 297,-172,-345, 333, -16, -62, 110,-216, 250, 163,-668, 197, 605,-407,-229, 288,  43,-112,},
{ -11,  65, -37, -83, 211,-148,-181, 346, -45,-293, 156, 107, -34,-148,  92, 220,-323,-108, 424, -99,-316, 255,  45,-148, -27, 108,  37,-174, 180,  78,-342, 168, 247,-537, 409, 180,-628, 355, 312,-414,-113, 191, 226, -23,-473, 181, 455,-357,-241, 355, 107,-219,-209, 255, 270,-477, -78, 484,-109,-253,  72, 101,  81,-127,},
{ 297,-237,-376, 389, 179,-383,  97, 203,-173,-162, 171, 286,-248,-277, 259,  37, -69, 120,-168, -28, 215,-167,  30,  94,-136,  39, -86, 100, 374,-432,-264, 523,-115,-235, 264,-124, -53, 116, -72, 168,-126,-199, 242,  -8,  -1, -12,-168, 204,  28,-355, 309, 205,-322, -79, 125, 275,-208,-393, 538,  80,-656, 330, 486,-439,},
{-142, 200,  82,-293, 126, 130,-166,   2, 131,  72,-315,-102, 509, -42,-429, 178, 135, -28,-109, -81, 274,-179,-147, 349,-189, -76, 322,-385,  67, 414,-367,-228, 524,-192,-296, 416, -55,-310,  78, 234,  25,-212, -96, 275,   3,-302, 153, 253,-145,-289, 128, 405,-266,-369, 519,  11,-552, 289, 321,-251,-122,  62, 186,-141,},
{-274, 287, 221,-294,-327, 532, 169,-694, 311, 341,-456,  16, 259,  -7,-190, -92, 367, -39,-389, 218, 164,-142,  13,-107, 118,  62,-338, 294, 265,-470, -55, 376,-113,-165, 168,  48,-172, -64, 223, 101,-343, -19, 306, -47,-141, -29, 143,  96,-360,  65, 497,-358,-335, 513, -52,-243, 135, -47,  29,  55,-118,  69,  34, -40,},
{  64, -66, -97, 195, -31,-238, 189, 150,-263, -21, 285, -24,-377, 137, 396,-376,-135, 427, -89,-297, 163, 120,-153,  79, -21,-101, 215, -44,-290, 204, 209,-297,  29, 250,-310, 118, 189,-332, -60, 420, -25,-454, 144, 453,-296,-392, 465, 203,-555,  47, 542,-306,-341, 471, -18,-354, 238, 106,-227,  64,  47, -43,  75, -54,},
{ 177,-278, -47, 351,-189,-203, 318, -19,-257, 195, 121,-329, -21, 426,-159,-339, 396,   3,-400, 221, 234,-284,  61, 141,-285, 150, 245,-402,  43, 401,-278,-227, 371,-190, -69, 406,-390,-125, 415, -47,-362, 115, 333,-208,-255, 345,  32,-316,  56, 301,-173,-260, 402,  14,-425, 284, 156,-337, 117, 149,-162,  34,  59, -45,},
{ 307,-435, -90, 455,-104,-324, 213, 182,-275,   9, 229,-239, -52, 369,-196,-246, 380,-116,-346, 386,  83,-305, 195,-143,  14, 274,-342,  80, 220,-245, -30, 151, -24, -40,  15,  28, -43,  -7, 124,-127, -49, 196,-182, -60, 256,-102,-180, 267,   5,-321, 168, 260,-356, -59, 505,-390,-181, 588,-350,-310, 544,-125,-374, 288,},
{-200, 308, -85,-143,  99, 115,-157, -64, 273,-237, -49, 425,-365,-253, 633,-219,-413, 481,-102,-228, 329,-233,  33, 129,-202, 204,-113, -11,  79,-107, 119,-121,  13, 222,-304, 194,  60,-449, 511, 108,-709, 359, 369,-479, 101, 290,-269, -36, 151, -25,-107,  48,  64,-185, 204,  30,-257, 242, -48,-211, 252,  21,-215, 118,},
{  75,-121,  27,  79, -59, -49,  78,   4,-106, 127, -16,-146, 147,  14, -99, 106,-125,   3, 216,-202,  65, -23, -83, 225,-245, 208, -60,-253, 538,-292,-324, 508,-283,  16, 202,-370, 423,-199,-266, 520,-135,-372, 223, 217,-164,-196, 332,-165,-269, 537,-239,-326, 498,-234, -64, 316,-352, 141, 159,-409, 391,   5,-450, 304,},
{-228, 280, 110,-325,  93, 230,-311, 107, 201,-375, 230, 181,-443, 250, 236,-503, 194, 279,-336,  13, 271,-272,  52, 233,-324, 124, 161,-266, 170,  52,-232, 166,  43,-238, 278, -89,-208, 389,-122,-409, 410, 234,-606, 134, 505,-473, -22, 347,-348, 153, 112,-324, 262,  56,-231, 161,  -8, -82,  82, -77,  31,  50,-111,  69,},
{ 289,-446,  13, 420,-328, -60, 289,-264,  57, 190,-290,  83, 226,-293,  55, 374,-452, -32, 426,-328,   2, 220,-319, 224,  39,-186, 232,-179, -47, 233,-154, -80, 181,-137,  45,  97,-189,  84, 115,-144, -65, 282,-148,-319, 500, -74,-393, 478,-217,-230, 514,-342, -84, 314,-321, 163,  41,-133, 178,-263, 198, 148,-450, 266,},
{ 283,-467,  75, 445,-485,  38, 427,-575, 311, 196,-560, 492,  -9,-440, 442,-132,-207, 336,-193, -10, 144,-180, 105,   9, -89,  73, -13,   0,  -9,   7, -16, -35, 105, -60,  -2,  66,-180, 233, -71,-266, 377, -63,-277, 324,-111,-211, 363,-259,  22, 299,-448, 217, 126,-316, 310,-127, -63, 172,-252, 214, -13,-187, 230,-107,},
{-195, 460,-405,  -2, 422,-528, 270, 183,-528, 529,-155,-311, 468,-228,-117, 289,-270, 146,  11,-136, 133,-126, 136, -31, -90, 148,-212, 134, 131,-258, 150,   8,-103, 166,-157,  91,  -6,-163, 329,-251,-125, 482,-402, -61, 434,-423, 150, 134,-321, 351, -98,-238, 256, -65,   7, -41,  72,   8,-214, 292,-153,-113, 356,-229,},
{ 290,-588, 470, -38,-350, 428,-285,  39, 177,-272, 254,-121,-130, 268,-192,  66,  44,-232, 369,-234, -40, 212,-253, 291,-234,  11, 202,-320, 313,-133,-129, 288,-300, 218, -76, -86, 249,-394, 325,   5,-326, 386,-130,-178, 274,-190,  48,  53, -76,  91, -78,  64, -53, -32, 180,-253, 177,  49,-351, 495,-246,-261, 576,-321,},
{-128, 303,-303,  94, 152,-261, 259,-175,  29, 116,-226, 267,-178, -44, 240,-341, 350,-163,-147, 391,-463, 332,-107,-123, 391,-555, 437,-117,-241, 468,-460, 237,  69,-298, 389,-389, 293,-107,-140, 395,-422, 174, 102,-324, 345,-103,-103, 159,-177, 168, -78, -40, 127,-201, 225, -72,-127, 209,-267, 182,  53,-160, 167, -92,},
{-181, 392,-371, 156, 133,-305, 287,-204,  95,  76,-232, 301,-271, 149,  13,-189, 324,-311, 165,  29,-209, 311,-315, 256,-100,-141, 330,-388, 349,-194, -70, 294,-379, 347,-293, 171,  72,-291, 400,-380, 159, 184,-363, 297,-163, -36, 245,-318, 258, -70,-185, 380,-391, 243, -53,-134, 218,-245, 282,-157, -62, 284,-421, 216,},
{ 124,-297, 307,-164, -30, 178,-257, 272,-224, 104,  64,-198, 265,-273, 170,  21,-233, 358,-272,  61, 145,-278, 302,-263, 211,-118,  -4,  88,-222, 357,-308, 121,  96,-243, 265,-322, 381,-275,  89, 122,-342, 420,-286,  65, 122,-272, 382,-316,  87, 113,-307, 448,-396, 251, -43,-249, 404,-384, 299, -91,-171, 409,-455, 193,},
{-179, 405,-499, 433,-207, -58, 262,-402, 462,-407, 250,  30,-328, 499,-527, 375, -95,-184, 341,-364, 306,-209, 115,  38,-182, 232,-211, 178,-167,  73, 135,-288, 319,-242, 180,-137,  19, 148,-278, 291,-200,  38, 107,-184, 250,-250, 147,  -3,-154, 253,-280, 240,-113, -65, 222,-293, 254,-152,  -2, 170,-255, 275,-235,  95,},
{ 118,-286, 380,-374, 266, -75,-133, 275,-358, 390,-336, 214, -52,-125, 232,-295, 317,-248, 111,  19,-119, 195,-223, 242,-227, 116, -25, -32, 137,-247, 290,-235,  84,  69,-154, 245,-333, 358,-318, 207, -15,-222, 380,-384, 295, -91,-121, 224,-313, 349,-283, 138,  28,-222, 388,-417, 305,-162,  14, 192,-359, 473,-457, 190,},
{  77,-168, 230,-273, 276,-225, 113,  34,-166, 283,-378, 370,-257,  75, 102,-258, 391,-431, 386,-275, 123,  32,-183, 282,-361, 435,-398, 294,-182,   4, 201,-344, 383,-355, 316,-272, 180, -37, -83, 190,-272, 292,-226,  68,  53,-155, 269,-305, 276,-196,  66,  91,-219, 304,-337, 262,-126,  -5, 129,-248, 341,-397, 368,-158,},
{ -90, 238,-347, 397,-381, 304,-194,  75,  67,-192, 315,-439, 449,-343, 240,-132, -48, 237,-360, 456,-477, 464,-451, 348,-227, 100,  47,-159, 278,-392, 405,-329, 195, -71,   1,  94,-201, 312,-395, 378,-318, 261,-132, -63, 163,-189, 210,-197, 130,   1,-125, 176,-180, 167,-117,  63, -29, -46, 133,-149, 152,-175, 142, -51,},
{  77,-195, 279,-325, 317,-273, 212,-149,  98, -38, -50, 180,-297, 338,-326, 299,-286, 253,-164,  63,  30,-149, 243,-286, 317,-301, 260,-220, 166, -76, -54, 164,-251, 320,-355, 387,-424, 419,-358, 276,-170, -18, 235,-390, 429,-374, 307,-226,  77,  86,-177, 220,-248, 258,-216, 105,  18,-114, 206,-327, 413,-390, 293,-115,},
{  86,-210, 298,-332, 324,-309, 271,-222, 150, -70,  -4,  85,-160, 210,-234, 233,-244, 273,-244, 179,-159, 131, -74,  45, -17, -43,  89,-114, 150,-184, 178,-128,  82, -49,  22,  -1, -52, 149,-257, 355,-422, 451,-457, 372,-216,  34, 165,-306, 395,-452, 467,-430, 333,-220, 107,  42,-176, 252,-319, 404,-462, 460,-372, 148,},
{ -82, 197,-281, 334,-374, 383,-331, 288,-244, 171,-104,  13,  94,-193, 266,-326, 400,-476, 524,-498, 428,-407, 413,-375, 322,-278, 227,-159,  95, -31, -40, 110,-166, 189,-203, 254,-319, 344,-344, 330,-296, 236,-168,  79,  33,-132, 218,-272, 279,-252, 224,-179, 108, -45, -19,  94,-142, 162,-172, 135, -96,  99, -86,  33,},
{ -31,  76, -95,  93, -97, 116,-122, 101, -87,  85, -64,  45, -50,  61, -72,  76, -83,  85, -94, 138,-177, 222,-293, 354,-409, 465,-505, 551,-623, 690,-676, 591,-474, 367,-329, 320,-290, 253,-196, 139,-104,  53, -15, -19,  60, -82,  89, -77,  60, -27, -30,  81,-109, 129,-149, 181,-217, 215,-199, 183,-166, 142,-107,  48,},
{ -18,  23, -23,  46, -69,  87,-111, 129,-127, 111, -80,  69, -81,  72, -77,  78, -69,  78, -82,  48,   1, -18,  16, -22,  47, -95, 133,-153, 163,-179, 191,-172, 166,-168, 156,-147, 148,-172, 213,-268, 324,-366, 397,-376, 341,-294, 238,-195,  93,  28,-144, 282,-387, 467,-536, 595,-628, 612,-574, 491,-382, 310,-260, 117,},
{ -38,  92,-154, 240,-316, 361,-393, 436,-444, 448,-504, 548,-517, 453,-405, 360,-293, 233,-187, 122, -57,  20,   3, -34,  66, -83,  92,-113, 139,-136, 107, -96,  85, -59,  38, -31,   2,  71,-139, 192,-241, 288,-339, 384,-416, 426,-402, 378,-350, 294,-242, 199,-158, 128,-109,  81, -52,  33, -16,  -9,  48, -79,  79, -34,},
{ -63, 134,-184, 243,-299, 321,-319, 321,-318, 316,-318, 319,-306, 272,-248, 209,-185, 202,-194, 169,-149, 115, -87,  96, -75,  25, -18,  35, -31,  19,   2, -32,  56, -85, 104,-107, 121,-160, 206,-228, 254,-288, 313,-348, 377,-394, 431,-473, 472,-450, 446,-431, 393,-358, 353,-342, 293,-237, 197,-158, 116, -70,  42, -20,},
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
