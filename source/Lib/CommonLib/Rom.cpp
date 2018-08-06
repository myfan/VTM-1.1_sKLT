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
{  15,  20,  24,  29,  33,  37,  41,  45,  49,  52,  56,  59,  62,  65,  68,  70,  72,  73,  75,  77,  78,  79,  79,  80,  80,  80,  78,  77,  76,  75,  73,  70,},
{ -42, -52, -61, -69, -75, -80, -83, -85, -86, -85, -82, -77, -71, -62, -53, -43, -33, -21,  -9,   2,  14,  26,  38,  48,  58,  67,  74,  78,  81,  82,  81,  76,},
{ -65, -77, -84, -88, -87, -82, -73, -59, -42, -24,  -5,  14,  34,  52,  67,  78,  85,  87,  85,  78,  67,  53,  36,  17,  -4, -24, -41, -57, -70, -79, -80, -77,},
{ -74, -86, -86, -80, -65, -45, -20,   9,  35,  59,  78,  88,  89,  80,  62,  35,   6, -23, -48, -69, -82, -89, -86, -74, -55, -29,  -1,  28,  53,  69,  78,  81,},
{ -78, -87, -79, -58, -27,   9,  43,  69,  84,  88,  75,  48,  11, -30, -61, -79, -88, -83, -62, -29,   8,  45,  72,  87,  88,  74,  48,  12, -26, -57, -77, -85,},
{ -85, -88, -66, -27,  19,  58,  82,  85,  69,  37,  -8, -51, -81, -90, -75, -40,   6,  46,  74,  85,  74,  42,   0, -41, -76, -90, -82, -53,  -6,  39,  75,  91,},
{ -87, -81, -41,   9,  57,  86,  83,  50,   1, -43, -72, -82, -60, -16,  32,  71,  85,  71,  29, -25, -71, -95, -78, -33,  20,  72,  97,  84,  39, -18, -67, -90,},
{ -95, -78, -18,  48,  90,  86,  47, -12, -67, -89, -70, -22,  39,  81,  83,  46, -12, -60, -87, -70, -23,  37,  79,  82,  49, -10, -70, -91, -64, -12,  49,  86,},
{ -85, -60,  10,  69,  89,  53, -13, -67, -84, -54,  12,  70,  81,  48, -11, -67, -92, -60,  18,  81,  97,  55, -26, -82, -84, -43,  32,  88,  78,  25, -37, -82,},
{ -86, -45,  38,  91,  76,   1, -75, -99, -49,  41,  94,  77,   9, -64, -88, -47,  18,  73,  75,  22, -46, -85, -60,  14,  76,  80,  20, -59, -84, -50,  25,  82,},
{ -50, -31,  28,  64,  39, -14, -58, -56,   0,  50,  63,  23, -49, -79, -32,  43,  89,  50, -35, -92, -72,  27, 108,  79, -28,-108, -85,  25, 110,  91, -16,-106,},
{ -79, -22,  66,  78,  28, -64,-100, -18,  73, 102,  25, -94, -96, -11,  73,  95,  16, -78, -83, -12,  73,  79, -13, -77, -49,  21,  73,  35, -45, -65,  -6,  56,},
{  94,  -1, -78, -64,  13,  90,  54, -52, -88, -27,  64,  80,  -1, -84, -58,  44,  89,  18, -62, -70,   0,  86,  52, -50, -88, -13,  89,  64, -52, -90, -10,  79,},
{ -86,   5,  86,  52, -51, -83,   2,  80,  52, -49, -80,   3,  81,  47, -52, -84,   1,  82,  46, -53, -78,  12,  95,  26, -86, -65,  63,  89, -25, -95, -24,  78,},
{ -94,  22, 106,  20, -87, -61,  56,  91, -21,-106, -13, 102,  40, -85, -62,  48,  87, -10, -84, -25,  68,  57, -54, -67,  29,  70,  -3, -71, -15,  67,  31, -53,},
{  56, -27, -70,  19,  66,   3, -62, -30,  69,  44, -73, -54,  69,  71, -54, -84,  32,  94, -14, -89, -12,  96,  33,-112, -35, 112,  28, -91, -31,  82,  42, -68,},
{ -95,  66,  90, -61, -85,  56,  88, -55, -87,  52,  77, -38, -66,  25,  69, -14, -76,  16,  70, -14, -71,  16,  82, -25, -89,  45,  75, -63, -59,  66,  51, -58,},
{ -56,  36,  66, -48, -65,  59,  59, -68, -44,  70,  25, -70,  -6,  87, -26, -85,  38,  80, -45, -77,  55,  73, -83, -41, 103,  -5,-101,  38,  91, -55, -75,  65,},
{  36, -34, -32,  49,  13, -57,  -4,  70, -13, -76,  48,  71, -91, -42, 107,   0,-100,  30,  92, -49, -88, 102,  31,-120,  45,  77, -91,   3,  75, -47, -39,  39,},
{ -58,  59,  42, -85,  -2,  90, -44, -70,  83,  26, -95,  19,  78, -52, -54,  78,  24,-100,  32,  86, -94,  -7,  87, -75, -10,  82, -68, -20,  82, -36, -52,  45,},
{  70, -97, -12, 123, -69, -81, 110,  -4, -84,  55,  25, -49,   6,  41, -32, -18,  58, -34, -57,  83,   3, -82,  86, -13, -79,  92, -19, -65,  85, -11, -67,  43,},
{ -46,  76, -30, -38,  54, -11, -13,  -9,  18,   8, -46,  52,   8, -83,  85,  -8, -78, 106, -47, -65, 115, -66,  -8,  77,-104,  50,  43,-103,  94,   7,-107,  67,},
{  52, -81,  13,  80, -84, -10,  89, -82,  -5, 105,-106,  -9, 112,-107,   4, 105,-102,  -1,  87, -77,   4,  59, -68,  22,  31, -44,  22,   0, -15,  17,  -6,   0,},
{ -33,  70, -41, -40,  97, -75, -16, 107,-107,  13,  80,-110,  70,  32,-110,  88,  -4, -70,  94, -46, -30,  60, -42,  23,  -4, -19,  39, -59,  37,  39, -82,  41,},
{ -70, 131, -99,   9,  78,-111,  77,   5, -80,  91, -42, -19,  58, -60,  24,   7, -19,  39, -48,  23,  14, -40,  52, -52,  26,  26, -85,  97, -30, -66, 111, -56,},
{  46, -90,  92, -50, -17,  67, -86,  75, -32, -25,  68, -82,  67, -15, -44,  74, -76,  48,  13, -70,  90, -73,  36,  13, -57,  84, -83,  41,  27, -84,  98, -46,},
{ -37,  76, -87,  75, -38, -10,  55, -84,  85, -55,   8,  41, -77,  89, -76,  37,  24, -76,  98, -90,  49,   7, -51,  75, -78,  63, -29, -13,  56, -90,  87, -37,},
{ -29,  60, -78,  80, -63,  29,  14, -52,  82, -96,  87, -55,  17,  23, -66,  92, -89,  63, -31,  -9,  49, -74,  84, -79,  55, -20, -17,  50, -79,  99, -90,  40,},
{  22, -51,  71, -82,  79, -63,  37,  -4, -29,  60, -88, 103, -96,  73, -48,  23,  11, -47,  74, -88,  83, -70,  53, -27,  -5,  36, -61,  79, -92,  90, -71,  29,},
{ -22,  48, -66,  80, -87,  89, -79,  63, -46,  23,   3, -30,  55, -75,  93,-101,  97, -89,  74, -54,  31,  -7, -16,  35, -52,  67, -74,  76, -76,  70, -56,  24,},
{ -15,  34, -50,  61, -73,  83, -87,  87, -83,  78, -72,  65, -52,  38, -27,  14,   2, -17,  33, -50,  62, -72,  84, -92,  95, -94,  88, -78,  67, -51,  35, -15,},
{  12, -26,  37, -46,  53, -60,  68, -70,  69, -69,  70, -70,  70, -75,  82, -84,  83, -84,  86, -83,  77, -74,  74, -75,  70, -60,  52, -46,  38, -30,  21,  -9,},
};

extern TMatrixCoeff g_aiKLT32HP[32][32] =
{
{  61,  79,  97, 116, 134, 149, 165, 181, 198, 210, 223, 237, 248, 259, 270, 280, 287, 293, 299, 308, 311, 314, 317, 320, 322, 319, 313, 310, 305, 299, 290, 281,},
{-166,-208,-242,-277,-300,-319,-333,-341,-344,-340,-328,-310,-283,-249,-212,-174,-131, -85, -37,   9,  56, 104, 151, 193, 232, 267, 295, 312, 322, 328, 322, 305,},
{-259,-309,-338,-352,-347,-327,-291,-236,-170, -98, -20,  56, 137, 209, 267, 311, 340, 350, 339, 312, 268, 211, 144,  68, -16, -96,-165,-227,-282,-314,-321,-308,},
{-295,-342,-344,-318,-259,-180, -79,  35, 141, 235, 311, 353, 356, 319, 247, 141,  24, -92,-194,-274,-327,-356,-344,-296,-219,-116,  -3, 111, 212, 278, 314, 324,},
{-312,-347,-316,-233,-106,  37, 173, 276, 336, 351, 299, 190,  43,-121,-242,-318,-351,-332,-248,-116,  31, 181, 287, 348, 351, 295, 191,  50,-103,-229,-308,-340,},
{-339,-354,-265,-109,  75, 232, 327, 341, 274, 149, -31,-205,-324,-359,-300,-158,  25, 185, 296, 341, 297, 166,  -1,-165,-304,-359,-327,-212, -25, 157, 299, 365,},
{-348,-325,-166,  36, 227, 343, 332, 199,   6,-170,-288,-328,-238, -64, 127, 284, 341, 285, 118, -98,-285,-380,-314,-133,  79, 286, 389, 335, 154, -70,-269,-362,},
{-379,-311, -72, 194, 361, 346, 186, -46,-269,-355,-281, -89, 157, 322, 330, 183, -47,-241,-347,-281, -92, 147, 316, 327, 196, -41,-278,-366,-256, -48, 197, 343,},
{-340,-239,  38, 276, 358, 214, -53,-269,-336,-217,  48, 282, 326, 191, -45,-267,-368,-238,  73, 325, 386, 218,-105,-329,-337,-171, 129, 353, 311, 102,-149,-330,},
{-343,-181, 151, 365, 306,   3,-300,-394,-196, 163, 374, 306,  38,-255,-351,-187,  73, 292, 302,  88,-185,-341,-240,  54, 303, 321,  78,-236,-338,-200, 102, 326,},
{-199,-125, 111, 257, 154, -56,-233,-225,   2, 200, 253,  93,-196,-314,-129, 171, 356, 199,-139,-369,-290, 109, 430, 316,-111,-430,-340, 101, 441, 365, -65,-423,},
{-317, -88, 264, 312, 111,-257,-399, -73, 293, 407,  99,-375,-384, -45, 291, 380,  66,-312,-330, -46, 293, 316, -52,-308,-196,  84, 292, 142,-182,-260, -23, 224,},
{ 375,  -3,-311,-257,  53, 362, 217,-209,-352,-107, 255, 319,  -2,-335,-233, 177, 355,  72,-248,-280,   1, 344, 209,-200,-353, -52, 355, 257,-210,-362, -41, 318,},
{-344,  19, 345, 210,-204,-331,   9, 318, 207,-197,-320,  10, 324, 190,-209,-338,   3, 328, 186,-212,-312,  48, 378, 103,-346,-259, 254, 357, -98,-381, -98, 310,},
{-377,  87, 426,  79,-347,-245, 226, 364, -84,-424, -53, 409, 160,-340,-250, 192, 348, -38,-337,-101, 273, 228,-217,-267, 114, 278, -12,-285, -62, 266, 124,-212,},
{ 223,-110,-282,  77, 265,  12,-247,-119, 275, 178,-292,-216, 276, 283,-217,-338, 129, 375, -56,-357, -50, 385, 131,-450,-140, 446, 111,-366,-123, 328, 166,-273,},
{-378, 263, 361,-245,-340, 226, 351,-219,-349, 210, 307,-154,-264,  98, 276, -54,-305,  62, 282, -56,-282,  65, 330,-100,-356, 181, 300,-250,-234, 262, 205,-233,},
{-222, 145, 263,-193,-259, 236, 237,-274,-176, 279,  99,-282, -24, 347,-103,-340, 153, 322,-181,-309, 221, 290,-334,-162, 414, -21,-404, 152, 363,-221,-298, 260,},
{ 144,-135,-128, 198,  51,-227, -14, 278, -53,-304, 193, 284,-364,-170, 428,   0,-399, 118, 367,-195,-354, 407, 124,-480, 179, 306,-363,  10, 298,-186,-157, 158,},
{-233, 235, 170,-339,  -8, 361,-174,-278, 330, 103,-378,  78, 311,-209,-216, 311,  97,-399, 129, 344,-374, -28, 350,-301, -38, 328,-274, -80, 330,-143,-210, 178,},
{ 280,-388, -49, 493,-274,-322, 441, -17,-337, 219,  99,-194,  23, 163,-130, -71, 234,-134,-230, 333,  11,-328, 343, -51,-314, 370, -76,-259, 338, -44,-268, 173,},
{-184, 303,-119,-152, 216, -43, -53, -36,  72,  30,-185, 208,  33,-333, 339, -33,-312, 426,-188,-261, 460,-266, -31, 307,-417, 198, 171,-413, 377,  29,-428, 267,},
{ 208,-324,  50, 319,-335, -38, 358,-328, -20, 418,-422, -36, 446,-428,  17, 421,-409,  -6, 346,-307,  16, 236,-270,  88, 124,-176,  88,   0, -60,  67, -23,   1,},
{-133, 280,-165,-161, 387,-301, -62, 428,-428,  51, 318,-442, 280, 128,-440, 351, -15,-280, 375,-184,-120, 238,-169,  91, -14, -75, 157,-236, 147, 156,-327, 162,},
{-280, 523,-395,  34, 313,-445, 309,  18,-319, 364,-167, -75, 232,-239,  96,  29, -75, 156,-192,  92,  56,-158, 208,-207, 105, 103,-339, 386,-120,-265, 444,-226,},
{ 185,-361, 369,-202, -69, 268,-343, 301,-129,-101, 271,-328, 267, -61,-177, 296,-305, 191,  51,-281, 361,-293, 145,  52,-229, 336,-333, 163, 109,-337, 393,-186,},
{-148, 305,-350, 299,-153, -41, 221,-337, 342,-221,  32, 162,-308, 358,-306, 150,  95,-306, 391,-358, 194,  29,-203, 299,-313, 250,-117, -51, 225,-358, 347,-148,},
{-115, 242,-311, 321,-254, 115,  54,-209, 327,-385, 350,-220,  66,  93,-264, 367,-355, 254,-126, -36, 197,-296, 337,-316, 221, -79, -70, 199,-317, 395,-361, 159,},
{  88,-205, 283,-327, 317,-252, 147, -16,-116, 240,-351, 412,-385, 292,-194,  91,  45,-187, 297,-351, 332,-278, 212,-106, -20, 146,-244, 316,-368, 358,-284, 117,},
{ -88, 191,-262, 318,-347, 354,-317, 253,-183,  93,  11,-120, 220,-301, 370,-403, 390,-355, 296,-217, 125, -27, -64, 139,-208, 269,-297, 304,-304, 278,-222,  96,},
{ -59, 138,-198, 246,-292, 330,-348, 347,-333, 311,-289, 258,-207, 153,-109,  57,  10, -68, 132,-200, 246,-287, 335,-369, 381,-375, 354,-313, 266,-204, 140, -58,},
{  49,-106, 146,-185, 213,-239, 272,-281, 276,-277, 280,-280, 281,-299, 326,-337, 333,-337, 346,-332, 308,-295, 298,-300, 282,-239, 206,-183, 151,-119,  86, -35,},
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
