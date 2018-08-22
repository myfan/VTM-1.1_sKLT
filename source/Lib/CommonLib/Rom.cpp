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
};

extern TMatrixCoeff g_aiKLT64HP[64][64] =
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
