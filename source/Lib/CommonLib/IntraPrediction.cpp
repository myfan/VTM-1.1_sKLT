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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"
#include "Buffer.h"

#include "dtrace_next.h"
#include "Rom.h"

#include <memory.h>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar IntraPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    20, //   1xn
    20, //   2xn
    20, //   4xn
    14, //   8xn
    2,  //  16xn
    0,  //  32xn
#if HM_MDIS_AS_IN_JEM
    20, //  64xn
#else
    0,  //  64xn
#endif
    0,  // 128xn
  },
  { // Chroma
    40, //   1xn
    40, //   2xn
    40, //   4xn
    28, //   8xn
    4,  //  16xn
    0,  //  32xn
#if HM_MDIS_AS_IN_JEM
    40, //  64xn
#else
    0,  //  64xn
#endif
    0,  // 128xn
  }
};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

IntraPrediction::IntraPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
{
  for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (UInt buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      m_piYuvExt[ch][buf] = nullptr;
    }
  }

  m_piTemp = nullptr;
}

IntraPrediction::~IntraPrediction()
{
  destroy();
}

Void IntraPrediction::destroy()
{
  for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (UInt buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      delete[] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = nullptr;
    }
  }

  delete[] m_piTemp;
  m_piTemp = nullptr;
}

Void IntraPrediction::init(ChromaFormat chromaFormatIDC, const unsigned bitDepthY)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
    m_iYuvExtSize = (MAX_CU_SIZE * 2 + 1) * (MAX_CU_SIZE * 2 + 1);

    for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      for (UInt buf = 0; buf < NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[m_iYuvExtSize];
      }
    }
  }

  if (m_piTemp == nullptr)
  {
    m_piTemp = new Pel[ MAX_CU_SIZE * MAX_CU_SIZE ];
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel IntraPrediction::xGetPredValDc( const CPelBuf &pSrc, const Size &dstSize )
{
  CHECK( dstSize.width == 0 || dstSize.height == 0, "Empty area provided" );

  Int iInd, iSum = 0;
  Pel pDcVal;
  const int width  = dstSize.width;
  const int height = dstSize.height;

  for( iInd = 0; iInd < width; iInd++ )
  {
    iSum += pSrc.at( 1 + iInd, 0 );
  }
  for( iInd = 0; iInd < height; iInd++ )
  {
    iSum += pSrc.at( 0, 1 + iInd );
  }

  pDcVal = ( iSum + ( ( width + height ) >> 1 ) ) / ( width + height );
  return pDcVal;
}

void IntraPrediction::predIntraAng( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool useFilteredPredSamples )
{
  const ComponentID    compID       = MAP_CHROMA( compId );
  const ChannelType    channelType  = toChannelType( compID );
  const Int            iWidth       = piPred.width;
  const Int            iHeight      = piPred.height;
  const UInt           uiDirMode    = PU::getFinalIntraMode( pu, channelType );


  CHECK( g_aucLog2[iWidth] < 2 && pu.cs->pcv->noChroma2x2, "Size not allowed" );
  CHECK( g_aucLog2[iWidth] > 7, "Size not allowed" );
  CHECK( iWidth != iHeight && !pu.cs->pcv->rectCUs, "Rectangular block are only allowed with QTBT" );

  const Int  srcStride = ( iWidth + iHeight + 1 );

#if HEVC_USE_HOR_VER_PREDFILTERING
  const Bool enableEdgeFilters = !(CU::isRDPCMEnabled( *pu.cu ) && pu.cu->transQuantBypass);
#endif
  Pel *ptrSrc = getPredictorPtr( compID, useFilteredPredSamples );

  {
    switch( uiDirMode )
    {
    case( DC_IDX ):     xPredIntraDc    ( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, channelType );            break; // including DCPredFiltering
    case( PLANAR_IDX ): xPredIntraPlanar( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, *pu.cs->sps );            break;
#if HEVC_USE_HOR_VER_PREDFILTERING
    default:            xPredIntraAng   ( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, channelType, uiDirMode,
                                          pu.cs->slice->clpRng( compID ), enableEdgeFilters, *pu.cs->sps );          break;
#else
    default:            xPredIntraAng   ( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, channelType, uiDirMode,
                                          pu.cs->slice->clpRng( compID ), *pu.cs->sps );          break;
#endif
    }
  }
}


Void IntraPrediction::xFilterGroup(Pel* pMulDst[], Int i, Pel const * const piSrc, Int iRecStride, Bool bAboveAvaillable, Bool bLeftAvaillable)
{
  pMulDst[0][i] = (piSrc[1] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[1][i] = (piSrc[iRecStride] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[3][i] = (piSrc[0] + piSrc[1] + 1) >> 1;

  pMulDst[2][i] = (piSrc[0] + piSrc[1] + piSrc[iRecStride] + piSrc[iRecStride + 1] + 2) >> 2;

}



/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra coding).
 */

//NOTE: Bit-Limit - 24-bit source
Void IntraPrediction::xPredIntraPlanar( const CPelBuf &pSrc, PelBuf &pDst, const SPS& sps )
{
  const UInt width  = pDst.width;
  const UInt height = pDst.height;
  const UInt log2W  = g_aucLog2[ width ];
  const UInt log2H  = g_aucLog2[ height ];

  Int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  const UInt offset = width * height;

  // Get left and above reference column and row
  for( Int k = 0; k < width + 1; k++ )
  {
    topRow[k] = pSrc.at( k + 1, 0 );
  }

  for( Int k = 0; k < height + 1; k++ )
  {
    leftColumn[k] = pSrc.at( 0, k + 1 );
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];
  Int topRight = topRow[width];

  for( Int k = 0; k < width; k++ )
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] << log2H;
  }

  for( Int k = 0; k < height; k++ )
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] << log2W;
  }

  const UInt finalShift = 1 + log2W + log2H;
  const UInt stride     = pDst.stride;
  Pel*       pred       = pDst.buf;
  for( Int y = 0; y < height; y++, pred += stride )
  {
    Int horPred = leftColumn[y];

    for( Int x = 0; x < width; x++ )
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      Int vertPred = topRow[x];
      pred[x]      = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
    }
  }
}




Void IntraPrediction::xPredIntraDc( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const bool enableBoundaryFilter )
{
  const Pel dcval = xGetPredValDc( pSrc, pDst );
  pDst.fill( dcval );

#if HEVC_USE_DC_PREDFILTERING
  if( enableBoundaryFilter )
  {
    xDCPredFiltering( pSrc, pDst, channelType );
  }
#endif
}

#if HEVC_USE_DC_PREDFILTERING
/** Function for filtering intra DC predictor. This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void IntraPrediction::xDCPredFiltering(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType &channelType)
{
  UInt iWidth = pDst.width;
  UInt iHeight = pDst.height;
  Int x, y;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))
  {
    //top-left
    pDst.at(0, 0) = (Pel)((pSrc.at(1, 0) + pSrc.at(0, 1) + 2 * pDst.at(0, 0) + 2) >> 2);

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )
    {
      pDst.at(x, 0) = (Pel)((pSrc.at(x + 1, 0)  +  3 * pDst.at(x, 0) + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1; y < iHeight; y++ )
    {
      pDst.at(0, y) = (Pel)((pSrc.at(0, y + 1) + 3 * pDst.at(0, y) + 2) >> 2);
    }
  }

  return;
}
#endif

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
*
* This function derives the prediction samples for the angular mode based on the prediction direction indicated by
* the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
* the reference row above the block in the case of vertical prediction or displacement of the rightmost column
* of the block and reference column left from the block in the case of the horizontal prediction. The displacement
* is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
* the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
* from the extended main reference.
*/
//NOTE: Bit-Limit - 25-bit source
#if HEVC_USE_HOR_VER_PREDFILTERING
Void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const UInt dirMode, const ClpRng& clpRng, const Bool bEnableEdgeFilters, const SPS& sps, const bool enableBoundaryFilter )
#else
Void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const UInt dirMode, const ClpRng& clpRng, const SPS& sps, const bool enableBoundaryFilter )
#endif
{
  Int width =Int(pDst.width);
  Int height=Int(pDst.height);

  CHECK( !( dirMode > DC_IDX && dirMode < NUM_LUMA_MODE ), "Invalid intra dir" );

  const Bool       bIsModeVer         = (dirMode >= DIA_IDX);
  const Int        intraPredAngleMode = (bIsModeVer) ? (Int)dirMode - VER_IDX :  -((Int)dirMode - HOR_IDX);
  const Int        absAngMode         = abs(intraPredAngleMode);
  const Int        signAng            = intraPredAngleMode < 0 ? -1 : 1;
#if HEVC_USE_HOR_VER_PREDFILTERING
  const Bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT);
#endif

  // Set bitshifts and scale the angle parameter to block size

  static const Int angTable[17]    = { 0,    1,    2,    3,    5,    7,    9,   11,   13,   15,   17,   19,   21,   23,   26,   29,   32 };
  static const Int invAngTable[17] = { 0, 8192, 4096, 2731, 1638, 1170,  910,  745,  630,  546,  482,  431,  390,  356,  315,  282,  256 }; // (256 * 32) / Angle

  Int invAngle                    = invAngTable[absAngMode];
  Int absAng                      = angTable   [absAngMode];
  Int intraPredAngle              = signAng * absAng;

  Pel* refMain;
  Pel* refSide;

  Pel  refAbove[2 * MAX_CU_SIZE + 1];
  Pel  refLeft [2 * MAX_CU_SIZE + 1];

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
    for( Int x = 0; x < width + 1; x++ )
    {
      refAbove[x + height - 1] = pSrc.at( x, 0 );
    }
    for( Int y = 0; y < height + 1; y++ )
    {
      refLeft[y + width - 1] = pSrc.at( 0, y );
    }
    refMain = (bIsModeVer ? refAbove + height : refLeft  + width ) - 1;
    refSide = (bIsModeVer ? refLeft  + width  : refAbove + height) - 1;

    // Extend the Main reference to the left.
    Int invAngleSum    = 128;       // rounding for (shift by 8)
    const Int refMainOffsetPreScale = bIsModeVer ? height : width;
    for( Int k = -1; k > (refMainOffsetPreScale * intraPredAngle) >> 5; k-- )
    {
      invAngleSum += invAngle;
      refMain[k] = refSide[invAngleSum>>8];
    }
  }
  else
  {
    for( Int x = 0; x < width + height + 1; x++ )
    {
      refAbove[x] = pSrc.at(x, 0);
      refLeft[x]  = pSrc.at(0, x);
    }
    refMain = bIsModeVer ? refAbove : refLeft ;
    refSide = bIsModeVer ? refLeft  : refAbove;
  }

  // swap width/height if we are doing a horizontal mode:
  Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
  const Int dstStride = bIsModeVer ? pDst.stride : MAX_CU_SIZE;
  Pel *pDstBuf = bIsModeVer ? pDst.buf : tempArray;
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }


  if( intraPredAngle == 0 )  // pure vertical or pure horizontal
  {
    for( Int y = 0; y < height; y++ )
    {
      for( Int x = 0; x < width; x++ )
      {
        pDstBuf[y*dstStride + x] = refMain[x + 1];
      }
    }
#if HEVC_USE_HOR_VER_PREDFILTERING
    if (edgeFilter)
    {
      for( Int y = 0; y < height; y++ )
      {
        pDstBuf[y*dstStride] = ClipPel( pDstBuf[y*dstStride] + ( ( refSide[y + 1] - refSide[0] ) >> 1 ), clpRng );
      }
    }
#endif
  }
  else
  {
    Pel *pDsty=pDstBuf;

    for (Int y=0, deltaPos=intraPredAngle; y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)
    {
      const Int deltaInt   = deltaPos >> 5;
      const Int deltaFract = deltaPos & (32 - 1);

      if( absAng < 32 )
      {
        {
          // Do linear filtering
          const Pel *pRM = refMain + deltaInt + 1;
          Int lastRefMainPel = *pRM++;
          for( Int x = 0; x < width; pRM++, x++ )
          {
            Int thisRefMainPel = *pRM;
            pDsty[x + 0] = ( Pel ) ( ( ( 32 - deltaFract )*lastRefMainPel + deltaFract*thisRefMainPel + 16 ) >> 5 );
            lastRefMainPel = thisRefMainPel;
          }
        }
      }
      else
      {
        // Just copy the integer samples
        for( Int x = 0; x < width; x++ )
        {
          pDsty[x] = refMain[x + deltaInt + 1];
        }
      }
    }
#if HEVC_USE_HOR_VER_PREDFILTERING
    if( edgeFilter && absAng <= 1 )
    {
      for( Int y = 0; y < height; y++ )
      {
        pDstBuf[y*dstStride] = ClipPel( pDstBuf[y*dstStride] + ((refSide[y + 1] - refSide[0]) >> 2), clpRng );
      }
    }
#endif
  }

  // Flip the block if this is the horizontal mode
  if( !bIsModeVer )
  {
    for( Int y = 0; y < height; y++ )
    {
      for( Int x = 0; x < width; x++ )
      {
        pDst.at( y, x ) = pDstBuf[x];
      }
      pDstBuf += dstStride;
    }
  }
}

void IntraPrediction::xReferenceFilter( const int doubleSize, const int origWeight, const int filterOrder, Pel *piRefVector, Pel *piLowPassRef )
{
  const int imCoeff[3][4] =
  {
    { 20, 15, 6, 1 },
    { 16, 14, 7, 3 },
    { 14, 12, 9, 4 }
  };

  const int * piFc;

  int binBuff[4 * MAX_CU_SIZE + 9];
  int * piTmp = &binBuff[2 * MAX_CU_SIZE + 4];   // to  use negative indexes
  Pel * piDat = piRefVector;
  Pel * piRes = piLowPassRef;

  for( int k = -doubleSize; k <= doubleSize; k++ )
    piTmp[k] = piDat[k];

  for( int n = 1; n <= 3; n++ )
  {
    piTmp[-doubleSize - n] = piTmp[-doubleSize - 1 + n];
    piTmp[ doubleSize + n] = piTmp[ doubleSize + 1 - n];
  }

  switch( filterOrder )
  {
  case 0:
    break;
  case 1:
    for( int k = -doubleSize; k <= doubleSize; k++ )
      piRes[k] = (Pel)(((piTmp[k] << 1) + piTmp[k - 1] + piTmp[k + 1] + 2) >> 2);
    break;
  case 2:
    for( int k = -doubleSize; k <= doubleSize; k++ )
      piRes[k] = (Pel)(((piTmp[k] << 1) + ((piTmp[k] + piTmp[k - 1] + piTmp[k + 1]) << 2) + piTmp[k - 2] + piTmp[k + 2] + 8) >> 4);
    break;
  case 3:
  case 5:
  case 7:
    piFc = imCoeff[(filterOrder - 3) >> 1];
    for( int k = -doubleSize; k <= doubleSize; k++ )
    {
      int s = 32 + piFc[0] * piTmp[k];
      for( int n = 1; n < 4; n++ )
        s += piFc[n] * (piTmp[k - n] + piTmp[k + n]);

      piRes[k] = (Pel)(s >> 6);
    }
    break;
  default:
    EXIT( "Invalid intra prediction reference filter order" );
  }

  int ParShift = 6; //normalization factor
  int ParScale = 1 << ParShift;
  int ParOffset = 1 << (ParShift - 1);

  if( origWeight != 0 )
  {
    int iCmptWeight = ParScale - origWeight;
    for( int k = -doubleSize; k <= doubleSize; k++ )
      piLowPassRef[k] = (origWeight * piRefVector[k] + iCmptWeight * piLowPassRef[k] + ParOffset) >> ParShift;
  }
}

Bool IntraPrediction::useDPCMForFirstPassIntraEstimation(const PredictionUnit &pu, const UInt &uiDirMode)
{
  return CU::isRDPCMEnabled(*pu.cu) && pu.cu->transQuantBypass && (uiDirMode == HOR_IDX || uiDirMode == VER_IDX);
}

inline Bool isAboveLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT );
inline Int  isAboveAvailable      ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *validFlags );
inline Int  isLeftAvailable       ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *validFlags );
inline Int  isAboveRightAvailable ( const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *validFlags );
inline Int  isBelowLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *validFlags );

Void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const Bool bFilterRefSamples)
{
  const CodingStructure& cs   = *cu.cs;

  Pel *refBufUnfiltered   = m_piYuvExt[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered     = m_piYuvExt[area.compID][PRED_BUF_FILTERED];

  // ----- Step 1: unfiltered reference samples -----
  xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, cu );
  // ----- Step 2: filtered reference samples -----
  if( bFilterRefSamples )
  {
    xFilterReferenceSamples( refBufUnfiltered, refBufFiltered, area, *cs.sps );
  }
}

void IntraPrediction::xFillReferenceSamples( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu )
{
  const ChannelType      chType = toChannelType( area.compID );
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int  tuWidth            = area.width;
  const int  tuHeight           = area.height;
  const int  predSize           = tuWidth + tuHeight;
  const int  predStride         = predSize + 1;

  const bool noShift            = pcv.noChroma2x2 && area.width == 4; // don't shift on the lowest level (chroma not-split)
  const int  unitWidth          = pcv.minCUWidth  >> (noShift ? 0 : getComponentScaleX( area.compID, sps.getChromaFormatIdc() ));
  const int  unitHeight         = pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY( area.compID, sps.getChromaFormatIdc() ));

  const int  totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits     = (predSize + (unitHeight - 1)) / unitHeight;
  const int  totalUnits         = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits      = std::max<int>( tuWidth / unitWidth, 1 );
  const int  numLeftUnits       = std::max<int>( tuHeight / unitHeight, 1 );
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK( numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported" );

  // ----- Step 1: analyze neighborhood -----
  const Position posLT          = area;
  const Position posRT          = area.topRight();
  const Position posLB          = area.bottomLeft();

  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  int   numIntraNeighbor = 0;

  memset( neighborFlags, 0, totalUnits );

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable( cu, chType, posLT );
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor += isAboveAvailable     ( cu, chType, posLT, numAboveUnits,      unitWidth,  (neighborFlags + totalLeftUnits + 1) );
  numIntraNeighbor += isAboveRightAvailable( cu, chType, posRT, numAboveRightUnits, unitWidth,  (neighborFlags + totalLeftUnits + 1 + numAboveUnits) );
  numIntraNeighbor += isLeftAvailable      ( cu, chType, posLT, numLeftUnits,       unitHeight, (neighborFlags + totalLeftUnits - 1) );
  numIntraNeighbor += isBelowLeftAvailable ( cu, chType, posLB, numLeftBelowUnits,  unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits) );

  // ----- Step 2: fill reference samples (depending on neighborhood) -----
  CHECK( predStride * predStride > m_iYuvExtSize, "Reference sample area not supported" );

  const Pel*  srcBuf    = recoBuf.buf;
  const int   srcStride = recoBuf.stride;
        Pel*  ptrDst    = refBufUnfiltered;
  const Pel*  ptrSrc;
  const Pel   valueDC   = 1 << (sps.getBitDepth( chType ) - 1);


  if( numIntraNeighbor == 0 )
  {
    // Fill border with DC value
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j]            = valueDC; }
    for( int i = 1; i <= predSize; i++ ) { ptrDst[i*predStride] = valueDC; }
  }
  else if( numIntraNeighbor == totalUnits )
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - srcStride - 1;
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j] = ptrSrc[j]; }
    // Fill left and below left border with rec. samples
    ptrSrc = srcBuf - 1;
    for( int i = 1; i <= predSize; i++ ) { ptrDst[i*predStride] = *(ptrSrc); ptrSrc += srcStride; }
  }
  else // reference samples are partially available
  {
    // BB: old implementation using tmpLineBuf
    // ---------------------------------------
    Pel  tmpLineBuf[5 * MAX_CU_SIZE];
    Pel* ptrTmp;
    int  unitIdx;

    // Initialize
    const int totalSamples = (totalLeftUnits * unitHeight) + ((totalAboveUnits + 1) * unitWidth); // all above units have "unitWidth" samples each, all left/below-left units have "unitHeight" samples each
    for( int k = 0; k < totalSamples; k++ ) { tmpLineBuf[k] = valueDC; }

    // Fill top-left sample
    ptrSrc = srcBuf - srcStride - 1;
    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight);
    unitIdx = totalLeftUnits;
    if( neighborFlags[unitIdx] )
    {
      Pel topLeftVal = ptrSrc[0];
      for( int j = 0; j < unitWidth; j++ ) { ptrTmp[j] = topLeftVal; }
    }

    // Fill left & below-left samples (downwards)
    ptrSrc += srcStride;
    ptrTmp--;
    unitIdx--;

    for( int k = 0; k < totalLeftUnits; k++ )
    {
      if( neighborFlags[unitIdx] )
      {
        for( int i = 0; i < unitHeight; i++ ) { ptrTmp[-i] = ptrSrc[i*srcStride]; }
      }
      ptrSrc += unitHeight*srcStride;
      ptrTmp -= unitHeight;
      unitIdx--;
    }

    // Fill above & above-right samples (left-to-right) (each unit has "unitWidth" samples)
    ptrSrc = srcBuf - srcStride;
    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight) + unitWidth; // offset line buffer by totalLeftUnits*unitHeight (for left/below-left) + unitWidth (for above-left)
    unitIdx = totalLeftUnits + 1;
    for( int k = 0; k < totalAboveUnits; k++ )
    {
      if( neighborFlags[unitIdx] )
      {
        for( int j = 0; j < unitWidth; j++ ) { ptrTmp[j] = ptrSrc[j]; }
      }
      ptrSrc += unitWidth;
      ptrTmp += unitWidth;
      unitIdx++;
    }

    // Pad reference samples when necessary
    int  currUnit       = 0;
    Pel* ptrTmpCurrUnit = tmpLineBuf;

    if( !neighborFlags[0] )
    {
      int nextUnit = 1;
      while( nextUnit < totalUnits && !neighborFlags[nextUnit] )
      {
        nextUnit++;
      }
      Pel* ptrTmpRef = tmpLineBuf + ((nextUnit < totalLeftUnits) ? (nextUnit * unitHeight) : ((totalLeftUnits * (unitHeight - unitWidth)) + (nextUnit * unitWidth)));
      const Pel refSample = *ptrTmpRef;
      // Pad unavailable samples with new value
      // fill left column
      while( currUnit < std::min<int>( nextUnit, totalLeftUnits ) )
      {
        for( int i = 0; i < unitHeight; i++ ) { ptrTmpCurrUnit[i] = refSample; }
        ptrTmpCurrUnit += unitHeight;
        currUnit++;
      }
      // fill top row
      while( currUnit < nextUnit )
      {
        for( int j = 0; j < unitWidth; j++ ) { ptrTmpCurrUnit[j] = refSample; }
        ptrTmpCurrUnit += unitWidth;
        currUnit++;
      }
    }

    // pad all other reference samples.
    while( currUnit < totalUnits )
    {
      const int numSamplesInCurrUnit = (currUnit >= totalLeftUnits) ? unitWidth : unitHeight;
      if( !neighborFlags[currUnit] ) // samples not available
      {
        const Pel refSample = *(ptrTmpCurrUnit - 1);
        for( int k = 0; k < numSamplesInCurrUnit; k++ ) { ptrTmpCurrUnit[k] = refSample; }

      }
      ptrTmpCurrUnit += numSamplesInCurrUnit;
      currUnit++;
    }

    // Copy processed samples
    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight) + (unitWidth - 1);
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j] = ptrTmp[j]; } // top left, top and top right samples

    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight);
    for( int i = 1; i <= predSize; i++ ) { ptrDst[i*predStride] = ptrTmp[-i]; }
  }
}

void IntraPrediction::xFilterReferenceSamples( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps )
{
  const int  tuWidth    = area.width;
  const int  tuHeight   = area.height;
  const int  predSize   = tuWidth + tuHeight;
  const int  predStride = predSize + 1;


#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  // Strong intra smoothing
  ChannelType chType = toChannelType( area.compID );
  if( sps.getUseStrongIntraSmoothing() && isLuma( chType ) )
  {
    const Pel bottomLeft = refBufUnfiltered[predStride * predSize];
    const Pel topLeft    = refBufUnfiltered[0];
    const Pel topRight   = refBufUnfiltered[predSize];

    const int  threshold     = 1 << (sps.getBitDepth( chType ) - 5);
    const bool bilinearLeft  = abs( (bottomLeft + topLeft)  - (2 * refBufUnfiltered[predStride * tuHeight]) ) < threshold; //difference between the
    const bool bilinearAbove = abs( (topLeft    + topRight) - (2 * refBufUnfiltered[             tuWidth ]) ) < threshold; //ends and the middle

    if( tuWidth >= 32 && tuHeight >= 32 && bilinearLeft && bilinearAbove )
#if !HEVC_USE_INTRA_SMOOTHING_T32
    if( tuWidth > 32 && tuHeight > 32 )
#endif
#if !HEVC_USE_INTRA_SMOOTHING_T64
    if( tuWidth < 64 && tuHeight < 64 )
#endif
    {
      Pel *piDestPtr = refBufFiltered + (predStride * predSize); // bottom left

      // apply strong intra smoothing
      for( UInt i = 0; i < predSize; i++, piDestPtr -= predStride ) //left column (bottom to top)
      {
        *piDestPtr = (((predSize - i) * bottomLeft) + (i * topLeft) + predSize / 2) / predSize;
      }
      for( UInt i = 0; i <= predSize; i++, piDestPtr++ )            //full top row (left-to-right)
      {
        *piDestPtr = (((predSize - i) * topLeft) + (i * topRight) + predSize / 2) / predSize;
      }

      return;
    }
  }
#endif

  // Regular reference sample filter
  const Pel *piSrcPtr  = refBufUnfiltered + (predStride * predSize); // bottom left
        Pel *piDestPtr = refBufFiltered   + (predStride * predSize); // bottom left

  // bottom left (not filtered)
  *piDestPtr = *piSrcPtr;
  piDestPtr -= predStride;
  piSrcPtr  -= predStride;
  //left column (bottom to top)
  for( UInt i=1; i < predSize; i++, piDestPtr-=predStride, piSrcPtr-=predStride )
  {
    *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[-predStride] + 2) >> 2;
  }
  //top-left
  *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[1] + 2) >> 2;
  piDestPtr++;
  piSrcPtr++;
  //top row (left-to-right)
  for( UInt i=1; i < predSize; i++, piDestPtr++, piSrcPtr++ )
  {
    *piDestPtr = (piSrcPtr[1] + 2 * piSrcPtr[0] + piSrcPtr[-1] + 2) >> 2;
  }
  // top right (not filtered)
  *piDestPtr=*piSrcPtr;
}

bool IntraPrediction::useFilteredIntraRefSamples( const ComponentID &compID, const PredictionUnit &pu, bool modeSpecific, const UnitArea &tuArea )
{
  const SPS         &sps    = *pu.cs->sps;
  const ChannelType  chType = toChannelType( compID );

  // high level conditions
  if( sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag() )                                       { return false; }
  if( !isLuma( chType ) && pu.chromaFormat != CHROMA_444 )                                               { return false; }


  if( !modeSpecific )                                                                                    { return true; }

  // pred. mode related conditions
  const int dirMode = PU::getFinalIntraMode( pu, chType );
  if( dirMode == DC_IDX )                                                                                { return false; }

  int diff = std::min<int>( abs( dirMode - HOR_IDX ), abs( dirMode - VER_IDX ) );
  int log2Size = ((g_aucLog2[tuArea.blocks[compID].width] + g_aucLog2[tuArea.blocks[compID].height]) >> 1);
  CHECK( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
  return (diff > m_aucIntraFilter[chType][log2Size]);
}


Bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT)
{
  const CodingStructure& cs = *cu.cs;
  const Position refPos = posLT.offset(-1, -1);
  const CodingUnit* pcCUAboveLeft = cs.isDecomp( refPos, chType ) ? cs.getCURestricted( refPos, cu, chType ) : nullptr;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool bAboveLeftFlag;

  if (isConstrained)
  {
    bAboveLeftFlag = pcCUAboveLeft && CU::isIntra(*pcCUAboveLeft);
  }
  else
  {
    bAboveLeftFlag = (pcCUAboveLeft ? true : false);
  }

  return bAboveLeftFlag;
}

Int isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;
  Int maxDx = uiNumUnitsInPU * unitWidth;

  for (UInt dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posLT.offset(dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }
  return iNumIntra;
}

Int isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;
  Int maxDy = uiNumUnitsInPU * unitHeight;

  for (UInt dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLT.offset(-1, dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

Int isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  UInt maxDx = uiNumUnitsInPU * unitWidth;

  for (UInt dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posRT.offset(unitWidth + dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }

  return iNumIntra;
}

Int isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;
  Int maxDy = uiNumUnitsInPU * unitHeight;

  for (UInt dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLB.offset(-1, unitHeight + dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if ( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}


//! \}
