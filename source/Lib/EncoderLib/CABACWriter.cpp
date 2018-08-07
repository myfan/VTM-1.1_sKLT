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

/** \file     CABACWriter.cpp
 *  \brief    Writer for low level syntax
 */

#include "CommonLib/Contexts.h"
#include "CABACWriter.h"

#include "EncLib.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/BinaryDecisionTree.h"

#include <map>
#include <algorithm>
#include <limits>


//! \ingroup EncoderLib
//! \{

void CABACWriter::initCtxModels( const Slice& slice )
{
  Int       qp                = slice.getSliceQp();
  SliceType sliceType         = slice.getSliceType();
  SliceType encCABACTableIdx  = slice.getEncCABACTableIdx();
  if( !slice.isIntra() && (encCABACTableIdx==B_SLICE || encCABACTableIdx==P_SLICE) && slice.getPPS()->getCabacInitPresentFlag() )
  {
    sliceType = encCABACTableIdx;
  }
  m_BinEncoder.reset( qp, (int)sliceType );
}



#if HM_REPRODUCE_CONTEXT_IDX_CALCULATION
template <class BinProbModel>
SliceType xGetCtxInitId( const Slice& slice, const BinEncIf& binEncoder, Ctx& ctxTest )
{
  const CtxStore<BinProbModel>& ctxStoreTest = static_cast<const CtxStore<BinProbModel>&>( ctxTest );
  const CtxStore<BinProbModel>& ctxStoreRef  = static_cast<const CtxStore<BinProbModel>&>( binEncoder.getCtx() );
  int qp = slice.getSliceQp();
  if( !slice.isIntra() )
  {
    SliceType aSliceTypeChoices[] = { B_SLICE, P_SLICE };
    UInt bestCost             = MAX_UINT;
    SliceType bestSliceType   = aSliceTypeChoices[0];
    for (UInt idx=0; idx<2; idx++)
    {
      UInt curCost            = 0;
      SliceType curSliceType  = aSliceTypeChoices[idx];
      ctxTest.init( qp, (int)curSliceType );
      for( int k = 0; k < Ctx::NumberOfContexts; k++ )
      {
        if( binEncoder.getNumBins(k) > 0 )
        {
          const BinProbModel& rcProbModel = ctxStoreRef[k];
          double              prob0       = rcProbModel.getProb0();
          double              prob1       = rcProbModel.getProb1();
          curCost += (UInt)(prob0 * ctxStoreTest.estFracBits(0,k) + prob1 * ctxStoreTest.estFracBits(1,k) );
        }
      }
      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}
#else
template <class BinProbModel>
SliceType xGetCtxInitId( const Slice& slice, const BinEncIf& binEncoder, Ctx& ctxTest )
{
  const CtxStore<BinProbModel>& ctxStoreTest = static_cast<const CtxStore<BinProbModel>&>( ctxTest );
  const CtxStore<BinProbModel>& ctxStoreRef  = static_cast<const CtxStore<BinProbModel>&>( binEncoder.getCtx() );
  int qp = slice.getSliceQp();
  if( !slice.isIntra() )
  {
    SliceType aSliceTypeChoices[] = { B_SLICE, P_SLICE };
    uint64_t  bestCost            = std::numeric_limits<uint64_t>::max();
    SliceType bestSliceType       = aSliceTypeChoices[0];
    for (UInt idx=0; idx<2; idx++)
    {
      uint64_t  curCost           = 0;
      SliceType curSliceType      = aSliceTypeChoices[idx];
      ctxTest.init( qp, (int)curSliceType );
      for( int k = 0; k < Ctx::NumberOfContexts; k++ )
      {
        if( binEncoder.getNumBins(k) > 0 )
        {
          curCost += uint64_t( binEncoder.getNumBins(k) ) * ctxStoreRef[k].estFracExcessBits( ctxStoreTest[k] );
        }
      }
      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}
#endif


SliceType CABACWriter::getCtxInitId( const Slice& slice )
{
  switch( m_TestCtx.getBPMType() )
  {
  case BPM_Std:   return  xGetCtxInitId<BinProbModel_Std>   ( slice, m_BinEncoder, m_TestCtx );
  default:        return  NUMBER_OF_SLICE_TYPES;
  }
}



unsigned estBits( BinEncIf& binEnc, const std::vector<bool>& bins, const Ctx& ctx, const int ctxId, const uint8_t winSize )
{
  binEnc.initCtxAndWinSize( ctxId, ctx, winSize );
  binEnc.start();
  const std::size_t numBins   = bins.size();
  unsigned          startBits = binEnc.getNumWrittenBits();
  for( std::size_t binId = 0; binId < numBins; binId++ )
  {
    unsigned  bin = ( bins[binId] ? 1 : 0 );
    binEnc.encodeBin( bin, ctxId );
  }
  unsigned endBits    = binEnc.getNumWrittenBits();
  unsigned codedBits  = endBits - startBits;
  return   codedBits;
}





//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    void  end_of_slice()
//================================================================================

void CABACWriter::end_of_slice()
{
  m_BinEncoder.encodeBinTrm ( 1 );
  m_BinEncoder.finish       ();
}




//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qp, ctuRsAddr, skipSao )
//================================================================================

void CABACWriter::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr, bool skipSao /* = false */ )
{
  CUCtx cuCtx( qps[CH_L] );
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );

  partitioner->initCtu( area, CH_L, *cs.slice );

  if( !skipSao )
  {
    sao( *cs.slice, ctuRsAddr );
  }
  coding_tree( cs, *partitioner, cuCtx );
  qps[CH_L] = cuCtx.qp;
  if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
  {
    CUCtx cuCtxChroma( qps[CH_C] );
    partitioner->initCtu( area, CH_C, *cs.slice );
    coding_tree( cs, *partitioner, cuCtxChroma );
    qps[CH_C] = cuCtxChroma.qp;
  }

  delete partitioner;
}





//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao             ( slice, ctuRsAddr )
//    void  sao_block_pars  ( saoPars, bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, onlyEstMergeInfo )
//    void  sao_offset_pars ( ctbPars, compID, sliceEnabled, bitDepth )
//================================================================================

void CABACWriter::sao( const Slice& slice, unsigned ctuRsAddr )
{
  const SPS& sps = *slice.getSPS();
  if( !sps.getUseSAO() )
  {
    return;
  }

  CodingStructure&     cs                     = *slice.getPic()->cs;
  const PreCalcValues& pcv                    = *cs.pcv;
  const SAOBlkParam&  sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool                slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool                slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  bool                sliceEnabled[3]         = { slice_sao_luma_flag, slice_sao_chroma_flag, slice_sao_chroma_flag };
  int                 frame_width_in_ctus     = pcv.widthInCtus;
  int                 ry                      = ctuRsAddr      / frame_width_in_ctus;
  int                 rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  const Position      pos                     ( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned      curSliceIdx             = slice.getIndependentSliceIdx();
#if HEVC_TILES_WPP
  const unsigned      curTileIdx              = cs.picture->tileMap->getTileIdxMap( pos );
  bool                leftMergeAvail          = cs.getCURestricted( pos.offset( -(Int)pcv.maxCUWidth, 0  ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool                aboveMergeAvail         = cs.getCURestricted( pos.offset( 0, -(Int)pcv.maxCUHeight ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
#else
  bool                leftMergeAvail          = cs.getCURestricted( pos.offset( -(Int)pcv.maxCUWidth, 0  ), curSliceIdx, CH_L ) ? true : false;
  bool                aboveMergeAvail         = cs.getCURestricted( pos.offset( 0, -(Int)pcv.maxCUHeight ), curSliceIdx, CH_L ) ? true : false;
#endif
  sao_block_pars( sao_ctu_pars, sps.getBitDepths(), sliceEnabled, leftMergeAvail, aboveMergeAvail, false );
}


void CABACWriter::sao_block_pars( const SAOBlkParam& saoPars, const BitDepths& bitDepths, bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo )
{
  bool isLeftMerge  = false;
  bool isAboveMerge = false;
  if( leftMergeAvail )
  {
    // sao_merge_left_flag
    isLeftMerge   = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_LEFT );
    m_BinEncoder.encodeBin( (isLeftMerge), Ctx::SaoMergeFlag() );
  }
  if( aboveMergeAvail && !isLeftMerge )
  {
    // sao_merge_above_flag
    isAboveMerge  = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_ABOVE );
    m_BinEncoder.encodeBin( (isAboveMerge), Ctx::SaoMergeFlag() );
  }
  if( onlyEstMergeInfo )
  {
    return; //only for RDO
  }
  if( !isLeftMerge && !isAboveMerge )
  {
    // explicit parameters
    for( int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      sao_offset_pars( saoPars[compIdx], ComponentID(compIdx), sliceEnabled[compIdx], bitDepths.recon[ toChannelType(ComponentID(compIdx)) ] );
    }
  }
}


void CABACWriter::sao_offset_pars( const SAOOffset& ctbPars, ComponentID compID, bool sliceEnabled, int bitDepth )
{
  if( !sliceEnabled )
  {
    CHECK( ctbPars.modeIdc != SAO_MODE_OFF, "Sao must be off, if it is disabled on slice level" );
    return;
  }
  const bool isFirstCompOfChType = ( getFirstComponentOfChannel( toChannelType(compID) ) == compID );

  if( isFirstCompOfChType )
  {
    // sao_type_idx_luma / sao_type_idx_chroma
    if( ctbPars.modeIdc == SAO_MODE_OFF )
    {
      m_BinEncoder.encodeBin  ( 0, Ctx::SaoTypeIdx() );
    }
    else if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 0 );
    }
    else
    {
      CHECK(!( ctbPars.typeIdc < SAO_TYPE_START_BO ), "Unspecified error");
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 1 );
    }
  }

  if( ctbPars.modeIdc == SAO_MODE_NEW )
  {
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( bitDepth );
    int       numClasses    = ( ctbPars.typeIdc == SAO_TYPE_BO ? 4 : NUM_SAO_EO_CLASSES );
    int       k             = 0;
    int       offset[4];
    for( int i = 0; i < numClasses; i++ )
    {
      if( ctbPars.typeIdc != SAO_TYPE_BO && i == SAO_CLASS_EO_PLAIN )
      {
        continue;
      }
      int classIdx = ( ctbPars.typeIdc == SAO_TYPE_BO ? ( ctbPars.typeAuxInfo + i ) % NUM_SAO_BO_CLASSES : i );
      offset[k++]  = ctbPars.offset[classIdx];
    }

    // sao_offset_abs
    for( int i = 0; i < 4; i++ )
    {
      unsigned absOffset = ( offset[i] < 0 ? -offset[i] : offset[i] );
      unary_max_eqprob( absOffset, maxOffsetQVal );
    }

    // band offset mode
    if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      // sao_offset_sign
      for( int i = 0; i < 4; i++ )
      {
        if( offset[i] )
        {
          m_BinEncoder.encodeBinEP( (offset[i] < 0) );
        }
      }
      // sao_band_position
      m_BinEncoder.encodeBinsEP( ctbPars.typeAuxInfo, NUM_SAO_BO_CLASSES_LOG2 );
    }
    // edge offset mode
    else
    {
      if( isFirstCompOfChType )
      {
        // sao_eo_class_luma / sao_eo_class_chroma
        CHECK( ctbPars.typeIdc - SAO_TYPE_START_EO < 0, "sao edge offset class is outside valid range" );
        m_BinEncoder.encodeBinsEP( ctbPars.typeIdc - SAO_TYPE_START_EO, NUM_SAO_EO_TYPES_LOG2 );
      }
    }
  }
}



//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    void  split_cu_flag     ( split, cs, partitioner )
//    void  split_cu_mode_mt  ( split, cs, partitioner )
//================================================================================

void CABACWriter::coding_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx )
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();
  const CodingUnit &cu        = *cs.getCU( currArea.blocks[partitioner.chType], partitioner.chType );

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if( pps.getUseDQP() && partitioner.currDepth <= pps.getMaxCuDQPDepth() )
  {
    cuCtx.isDQPCoded          = false;
  }
  if( cs.slice->getUseChromaQpAdj() && partitioner.currDepth <= pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
  }

  const PartSplit implicitSplit = partitioner.getImplicitSplit( cs );

  // QT
  bool canQtSplit = partitioner.canSplit( CU_QUAD_SPLIT, cs );

  if( canQtSplit )
  {
    // split_cu_flag
    bool qtSplit = implicitSplit == CU_QUAD_SPLIT;

    if( !qtSplit && implicitSplit != CU_QUAD_SPLIT )
    {
      qtSplit = ( cu.qtDepth > partitioner.currQtDepth );
      split_cu_flag( qtSplit, cs, partitioner );
    }

    // quad-tree split
    if( qtSplit )
    {
      partitioner.splitCurrArea( CU_QUAD_SPLIT, cs );

      do
      {
        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
      return;
    }
  }

  {
    //// MT
    //bool mtSplit = partitioner.canSplit( CU_MT_SPLIT, cs );

    //if( mtSplit )
    // MT
    bool mtSplit = partitioner.canSplit( CU_MT_SPLIT, cs );

    if( mtSplit )
    {
      const PartSplit splitMode = CU::getSplitAtDepth( cu, partitioner.currDepth );

      CHECK( implicitSplit != CU_DONT_SPLIT && implicitSplit != splitMode, "Different split found than the implicit split" );

      if( implicitSplit == CU_DONT_SPLIT )
      {
        split_cu_mode_mt( splitMode, cs, partitioner );
      }

      if( splitMode != CU_DONT_SPLIT )
      {
        partitioner.splitCurrArea( splitMode, cs );
        do
        {
          if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
          {
            coding_tree( cs, partitioner, cuCtx );
          }
        } while( partitioner.nextPart( cs ) );

        partitioner.exitCurrSplit();
        return;
      }
    }
  }

  // Predict QP on start of quantization group
  if( pps.getUseDQP() && !cuCtx.isDQPCoded && CU::isQGStart( cu ) )
  {
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }

  // coding unit
  coding_unit( cu, partitioner, cuCtx );

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
  DTRACE_BLOCK_REC_COND( ( !isEncoding() ), cs.picture->getRecoBuf( cu ), cu, cu.predMode );
}


void CABACWriter::split_cu_flag( bool split, const CodingStructure& cs, Partitioner& partitioner )
{
  unsigned maxQTDepth = ( cs.sps->getSpsNext().getUseQTBT()
    ? g_aucLog2[cs.sps->getSpsNext().getCTUSize()] - g_aucLog2[cs.sps->getSpsNext().getMinQTSize( cs.slice->getSliceType(), partitioner.chType )]
    : cs.sps->getLog2DiffMaxMinCodingBlockSize() );
//#else
//  unsigned maxQTDepth = ( cs.sps->getSpsNext().getUseQTBT()
//    ? g_aucLog2[cs.sps->getSpsNext().getCTUSize()] - g_aucLog2[cs.sps->getSpsNext().getMinQTSize( cs.slice->getSliceType(), partitioner.chType )]
//    : cs.sps->getLog2DiffMaxMinCodingBlockSize() );
//#endif
  if( partitioner.currDepth == maxQTDepth )
  {
    return;
  }
  unsigned  ctxId = DeriveCtx::CtxCUsplit( cs, partitioner );
  m_BinEncoder.encodeBin( (split), Ctx::SplitFlag(ctxId) );

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_flag() ctx=%d split=%d\n", ctxId, split ? 1 : 0 );
}

void CABACWriter::split_cu_mode_mt(const PartSplit split, const CodingStructure& cs, Partitioner& partitioner)
{
  unsigned ctxIdBT = DeriveCtx::CtxBTsplit( cs, partitioner );

  unsigned width   = partitioner.currArea().lumaSize().width;
  unsigned height  = partitioner.currArea().lumaSize().height;

  DecisionTree dt( g_mtSplitDTT );

#if HM_QTBT_AS_IN_JEM_SYNTAX
  unsigned minBTSize = cs.slice->isIntra() ? ( partitioner.chType == 0 ? MIN_BT_SIZE : MIN_BT_SIZE_C ) : MIN_BT_SIZE_INTER;

  dt.setAvail( DTT_SPLIT_BT_HORZ, height > minBTSize && ( partitioner.canSplit( CU_HORZ_SPLIT, cs ) || width  == minBTSize ) );
  dt.setAvail( DTT_SPLIT_BT_VERT, width  > minBTSize && ( partitioner.canSplit( CU_VERT_SPLIT, cs ) || height == minBTSize ) );
#else
  dt.setAvail( DTT_SPLIT_BT_HORZ, partitioner.canSplit( CU_HORZ_SPLIT, cs ) );
  dt.setAvail( DTT_SPLIT_BT_VERT, partitioner.canSplit( CU_VERT_SPLIT, cs ) );
#endif

  dt.setAvail( DTT_SPLIT_TT_HORZ, partitioner.canSplit( CU_TRIH_SPLIT,    cs ) );
  dt.setAvail( DTT_SPLIT_TT_VERT, partitioner.canSplit( CU_TRIV_SPLIT,    cs ) );

  unsigned btSCtxId = width == height ? 0 : ( width > height ? 1 : 2 );
  dt.setCtxId( DTT_SPLIT_DO_SPLIT_DECISION,   Ctx::BTSplitFlag( ctxIdBT ) );
  dt.setCtxId( DTT_SPLIT_HV_DECISION,         Ctx::BTSplitFlag( 3 + btSCtxId ) );

  dt.setCtxId( DTT_SPLIT_H_IS_BT_12_DECISION, Ctx::BTSplitFlag( 6 + btSCtxId ) );
  dt.setCtxId( DTT_SPLIT_V_IS_BT_12_DECISION, Ctx::BTSplitFlag( 9 + btSCtxId ) );


  encode_sparse_dt( dt, split == CU_DONT_SPLIT ? ( unsigned ) DTT_SPLIT_NO_SPLIT : ( unsigned ) split );

  DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode_mt() ctx=%d split=%d\n", ctxIdBT, split);
}


//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_transquant_bypass_flag ( cu )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  pcm_flag                  ( cu )
//    void  pcm_samples               ( tu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================

void CABACWriter::coding_unit( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  CodingStructure& cs = *cu.cs;

  // transquant bypass flag
  if( cs.pps->getTransquantBypassEnabledFlag() )
  {
    cu_transquant_bypass_flag( cu );
  }

  // skip flag
  if( !cs.slice->isIntra() )
  {
    cu_skip_flag( cu );
  }


  // skip data
  if( cu.skip )
  {
    CHECK( !cu.firstPU->mergeFlag, "Merge flag has to be on!" );
    PredictionUnit&   pu = *cu.firstPU;
    prediction_unit ( pu );
    end_of_ctu      ( cu, cuCtx );
    return;
  }

  // prediction mode and partitioning data
  pred_mode ( cu );
#if HEVC_USE_PART_SIZE
  part_mode ( cu );
#endif

  // pcm samples
  if( CU::isIntra(cu) && cu.partSize == SIZE_2Nx2N )
  {
    pcm_data( cu );
    if( cu.ipcm )
    {
      end_of_ctu( cu, cuCtx );
      return;
    }
  }

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // end of cu
  end_of_ctu( cu, cuCtx );
}


void CABACWriter::cu_transquant_bypass_flag( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( (cu.transQuantBypass), Ctx::TransquantBypassFlag() );
}


void CABACWriter::cu_skip_flag( const CodingUnit& cu )
{
  unsigned ctxId = DeriveCtx::CtxSkipFlag( cu );
  m_BinEncoder.encodeBin( ( cu.skip ), Ctx::SkipFlag( ctxId ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0 );
}


void CABACWriter::pred_mode( const CodingUnit& cu )
{
  if( cu.cs->slice->isIntra() )
  {
    return;
  }
  m_BinEncoder.encodeBin( ( CU::isIntra( cu ) ), Ctx::PredMode() );
}

#if HEVC_USE_PART_SIZE
void CABACWriter::part_mode( const CodingUnit& cu )
{
  if( cu.cs->pcv->only2Nx2N )
  {
    CHECK( cu.partSize != SIZE_2Nx2N, "No CU sub-partitionining allowed with QTBT" );
    return;
  }

  const SPS&      sps       = *cu.cs->sps;
  const unsigned  cuWidth   = cu.lumaSize().width;
  const unsigned  cuHeight  = cu.lumaSize().height;
  const int       log2DiffMaxMinCodingBlockSize = sps.getLog2DiffMaxMinCodingBlockSize();
  const PartSize  partSize  = cu.partSize;

  DecisionTree dt( g_partSizeDTT );

  dt.setCtxId( DTT_PS_IS_2Nx2N, Ctx::PartSize() );

  if( CU::isIntra( cu ) )
  {
    dt.setAvail( DTT_PS_nLx2N, false );
    dt.setAvail( DTT_PS_2NxN,  false );
    dt.setAvail( DTT_PS_Nx2N,  false );
    dt.setAvail( DTT_PS_nRx2N, false );
    dt.setAvail( DTT_PS_2NxnU, false );
    dt.setAvail( DTT_PS_2NxnD, false );
    dt.setAvail( DTT_PS_NxN,   cu.qtDepth == log2DiffMaxMinCodingBlockSize );
  }
  else
  {
    const bool isAmpAvail = sps.getUseAMP() && cu.qtDepth < log2DiffMaxMinCodingBlockSize;

    dt.setAvail( DTT_PS_2NxN,  true );
    dt.setAvail( DTT_PS_Nx2N,  true );
    dt.setAvail( DTT_PS_nLx2N, isAmpAvail );
    dt.setAvail( DTT_PS_nRx2N, isAmpAvail );
    dt.setAvail( DTT_PS_2NxnU, isAmpAvail );
    dt.setAvail( DTT_PS_2NxnD, isAmpAvail );
    dt.setAvail( DTT_PS_NxN,   cu.qtDepth == log2DiffMaxMinCodingBlockSize && !( cuWidth == 8 && cuHeight == 8 ) );

    dt.setCtxId( DTT_PS_IS_2Nx,     Ctx::PartSize( 1 ) );
    dt.setCtxId( DTT_PS_IS_2NxN,    Ctx::PartSize( 3 ) );
    dt.setCtxId( DTT_PS_IS_NOT_NxN, Ctx::PartSize( 2 ) );
    dt.setCtxId( DTT_PS_IS_Nx2N,    Ctx::PartSize( 3 ) );
  }

  encode_sparse_dt( dt, partSize );
}

#endif
void CABACWriter::pcm_data( const CodingUnit& cu )
{
  pcm_flag( cu );
  if( cu.ipcm )
  {
    m_BinEncoder.pcmAlignBits();
    pcm_samples( *cu.firstTU );
  }
}


void CABACWriter::pcm_flag( const CodingUnit& cu )
{
  const SPS& sps = *cu.cs->sps;
  if( !sps.getUsePCM() || cu.lumaSize().width > (1 << sps.getPCMLog2MaxSize()) || cu.lumaSize().width < (1 << sps.getPCMLog2MinSize()) )
  {
    return;
  }
  m_BinEncoder.encodeBinTrm( cu.ipcm );
}


void CABACWriter::cu_pred_data( const CodingUnit& cu )
{
  if( CU::isIntra( cu ) )
  {
    intra_luma_pred_modes  ( cu );
    intra_chroma_pred_modes( cu );
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    prediction_unit( pu );
  }

}



void CABACWriter::intra_luma_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  unsigned numMPMs   = cu.cs->pcv->numMPMs;
  int      numBlocks = CU::getNumPUs( cu );
  unsigned *mpm_preds  [4];
  unsigned mpm_idxs    [4];
  unsigned ipred_modes [4];

  const PredictionUnit* pu = cu.firstPU;

  // prev_intra_luma_pred_flag
  for( int k = 0; k < numBlocks; k++ )
  {
    unsigned*& mpm_pred   = mpm_preds[k];
    unsigned&  mpm_idx    = mpm_idxs[k];
    unsigned&  ipred_mode = ipred_modes[k];

    mpm_pred = ( unsigned* ) alloca( numMPMs * sizeof( unsigned ) );
    PU::getIntraMPMs( *pu, mpm_pred );

    ipred_mode = pu->intraDir[0];
    mpm_idx    = numMPMs;
    for( unsigned idx = 0; idx < numMPMs; idx++ )
    {
      if( ipred_mode == mpm_pred[idx] )
      {
        mpm_idx = idx;
        break;
      }
    }
    m_BinEncoder.encodeBin( mpm_idx < numMPMs, Ctx::IPredMode[0]() );

    pu = pu->next;
  }

  pu = cu.firstPU;

  // mpm_idx / rem_intra_luma_pred_mode
  for( int k = 0; k < numBlocks; k++ )
  {
    const unsigned& mpm_idx = mpm_idxs[k];
    if( mpm_idx < numMPMs )
    {
      {
        m_BinEncoder.encodeBinEP( mpm_idx > 0 );
        if( mpm_idx )
        {
          m_BinEncoder.encodeBinEP( mpm_idx > 1 );
        }
      }
    }
    else
    {
      unsigned* mpm_pred   = mpm_preds[k];
      unsigned  ipred_mode = ipred_modes[k];

      // sorting of MPMs
      std::sort( mpm_pred, mpm_pred + numMPMs );

      {
        CHECK( g_intraMode33to65AngMapping[g_intraMode65to33AngMapping[ipred_mode]] != ipred_mode, "Using an extended intra mode, although not enabled" );

        ipred_mode = g_intraMode65to33AngMapping[ipred_mode];
        for( int idx = int( numMPMs ) - 1; idx >= 0; idx-- )
        {
          if( ipred_mode > g_intraMode65to33AngMapping[mpm_pred[idx]] )
          {
            ipred_mode--;
          }
        }

        CHECK( ipred_mode >= 32, "Incorrect mode" );

        m_BinEncoder.encodeBinsEP( ipred_mode, 5 );
      }
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
    pu = pu->next;
  }
}


void CABACWriter::intra_luma_pred_mode( const PredictionUnit& pu )
{

  // prev_intra_luma_pred_flag
  unsigned  numMPMs  = pu.cs->pcv->numMPMs;
  unsigned *mpm_pred = ( unsigned* ) alloca( numMPMs * sizeof( unsigned ) );

  PU::getIntraMPMs( pu, mpm_pred );

  unsigned ipred_mode = pu.intraDir[0];
  unsigned mpm_idx = numMPMs;

  for( unsigned idx = 0; idx < numMPMs; idx++ )
  {
    if( ipred_mode == mpm_pred[idx] )
    {
      mpm_idx = idx;
      break;
    }
  }
  m_BinEncoder.encodeBin( mpm_idx < numMPMs, Ctx::IPredMode[0]() );

  // mpm_idx / rem_intra_luma_pred_mode
  if( mpm_idx < numMPMs )
  {
    {
      m_BinEncoder.encodeBinEP( mpm_idx > 0 );
      if( mpm_idx )
      {
        m_BinEncoder.encodeBinEP( mpm_idx > 1 );
      }
    }
  }
  else
  {
    std::sort( mpm_pred, mpm_pred + numMPMs );
    {
      CHECK( g_intraMode33to65AngMapping[g_intraMode65to33AngMapping[ipred_mode]] != ipred_mode, "Using an extended intra mode, although not enabled" );

      ipred_mode = g_intraMode65to33AngMapping[ipred_mode];
      for( int idx = int( numMPMs ) - 1; idx >= 0; idx-- )
      {
        if( ipred_mode > g_intraMode65to33AngMapping[mpm_pred[idx]] )
        {
          ipred_mode--;
        }
      }

      m_BinEncoder.encodeBinsEP( ipred_mode, 5 );
    }
  }
}


void CABACWriter::intra_chroma_pred_modes( const CodingUnit& cu )
{
  if( cu.chromaFormat == CHROMA_400 || ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_LUMA ) )
  {
    return;
  }

#if HEVC_USE_PART_SIZE
  int numBlocks = enable4ChromaPUsInIntraNxNCU( cu.chromaFormat ) ? CU::getNumPUs( cu ) : 1;

#endif
  const PredictionUnit* pu = cu.firstPU;

#if HEVC_USE_PART_SIZE
  for( int k = 0; k < numBlocks; k++ )
  {
    intra_chroma_pred_mode( *pu );
    pu = pu->next;
  }

  assert( numBlocks == 1 || pu == nullptr );
#else
  intra_chroma_pred_mode( *pu );
#endif
}


void CABACWriter::intra_chroma_pred_mode( const PredictionUnit& pu )
{
  const unsigned intraDir = pu.intraDir[1];
  {
    if( intraDir == DM_CHROMA_IDX )
    {
      m_BinEncoder.encodeBin( 0, Ctx::IPredMode[1]( 1 ) );
      return;
    }
    m_BinEncoder.encodeBin( 1, Ctx::IPredMode[1]( 1 ) );
  }

  // chroma candidate index
  unsigned chromaCandModes[ NUM_CHROMA_MODE ];
  PU::getIntraChromaCandModes( pu, chromaCandModes );

  int candId = 0;
  for ( ; candId < NUM_CHROMA_MODE; candId++ )
  {
    if( intraDir == chromaCandModes[ candId ] )
    {
      break;
    }
  }

  CHECK( candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds" );
  CHECK( chromaCandModes[ candId ] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path" );
  {
    m_BinEncoder.encodeBinsEP( candId, 2 );
  }
}


void CABACWriter::cu_residual( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  if( CU::isInter( cu ) )
  {
    PredictionUnit& pu = *cu.firstPU;
    if( !( ( cu.cs->pcv->noRQT || cu.partSize == SIZE_2Nx2N ) && pu.mergeFlag ) )
    {
      rqt_root_cbf( cu );
    }

    if( !cu.rootCbf )
    {
      return;
    }
  }


#if HEVC_USE_RQT
  cuCtx.quadtreeTULog2MinSizeInCU = CU::getQuadtreeTULog2MinSizeInCU( cu );
#endif
  ChromaCbfs chromaCbfs;
  transform_tree( *cu.cs, partitioner, cuCtx, chromaCbfs );
}

void CABACWriter::rqt_root_cbf( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( cu.rootCbf, Ctx::QtRootCbf() );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}


void CABACWriter::end_of_ctu( const CodingUnit& cu, CUCtx& cuCtx )
{
  const Slice*  slice             = cu.cs->slice;
#if HEVC_TILES_WPP
  const TileMap& tileMap          = *cu.cs->picture->tileMap;
  const int     currentCTUTsAddr  = tileMap.getCtuRsToTsAddrMap( CU::getCtuAddr( cu ) );
#else
  const int     currentCTUTsAddr  = CU::getCtuAddr( cu );
#endif
  const bool    isLastSubCUOfCtu  = CU::isLastSubCUOfCtu( cu );

  if ( isLastSubCUOfCtu
    && ( !CS::isDualITree( *cu.cs ) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );

    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    // i.e. when the slice segment CurEnd CTU address is the current CTU address+1.
#if HEVC_DEPENDENT_SLICES
    if( slice->getSliceSegmentCurEndCtuTsAddr() != currentCTUTsAddr + 1 )
#else
    if(slice->getSliceCurEndCtuTsAddr() != currentCTUTsAddr + 1)
#endif
    {
      m_BinEncoder.encodeBinTrm( 0 );
    }
  }
}





//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu );
//    void  merge_flag      ( pu );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACWriter::prediction_unit( const PredictionUnit& pu )
{
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  CHECK( pu.cacheUsed, "Processing a PU that should be in cache!" );
  CHECK( pu.cu->cacheUsed, "Processing a CU that should be in cache!" );

#endif
  if( pu.cu->skip )
  {
    CHECK( !pu.mergeFlag, "merge_flag must be true for skipped CUs" );
  }
  else
  {
    merge_flag( pu );
  }
  if( pu.mergeFlag )
  {
    merge_idx    ( pu );
  }
  else
  {
    inter_pred_idc( pu );
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      ref_idx     ( pu, REF_PIC_LIST_0 );
      {
        mvd_coding( pu.mvd[REF_PIC_LIST_0] );
      }
      mvp_flag    ( pu, REF_PIC_LIST_0 );
    }
    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      ref_idx     ( pu, REF_PIC_LIST_1 );
      if( !pu.cs->slice->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
      {
        {
          mvd_coding( pu.mvd[REF_PIC_LIST_1] );
        }
      }
      mvp_flag    ( pu, REF_PIC_LIST_1 );
    }
  }
}


void CABACWriter::merge_flag( const PredictionUnit& pu )
{
  m_BinEncoder.encodeBin( pu.mergeFlag, Ctx::MergeFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}


void CABACWriter::merge_idx( const PredictionUnit& pu )
{

  int numCandminus1 = int( pu.cs->slice->getMaxNumMergeCand() ) - 1;
  if( numCandminus1 > 0 )
  {
    if( pu.mergeIdx == 0 )
    {
      m_BinEncoder.encodeBin( 0, Ctx::MergeIdx() );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 1, Ctx::MergeIdx() );
      for( unsigned idx = 1; idx < numCandminus1; idx++ )
      {
        {
          m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
        }
        if( pu.mergeIdx == idx )
        {
          break;
        }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
}

void CABACWriter::inter_pred_idc( const PredictionUnit& pu )
{
  if( !pu.cs->slice->isInterB() )
  {
    return;
  }
  if( pu.cu->partSize == SIZE_2Nx2N || pu.cu->lumaSize().width != 8 )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( pu.interDir == 3 )
    {
      m_BinEncoder.encodeBin( 1, Ctx::InterDir(ctxId) );
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 0, Ctx::InterDir(ctxId) );
    }
  }
  m_BinEncoder.encodeBin( ( pu.interDir == 2 ), Ctx::InterDir( 4 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
}


void CABACWriter::ref_idx( const PredictionUnit& pu, RefPicList eRefList )
{
  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);
  if( numRef <= 1 )
  {
    return;
  }
  int refIdx  = pu.refIdx[eRefList];
  m_BinEncoder.encodeBin( (refIdx > 0), Ctx::RefPic() );
  if( numRef <= 2 || refIdx == 0 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  m_BinEncoder.encodeBin( (refIdx > 1), Ctx::RefPic(1) );
  if( numRef <= 3 || refIdx == 1 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  for( int idx = 3; idx < numRef; idx++ )
  {
    if( refIdx > idx - 1 )
    {
      m_BinEncoder.encodeBinEP( 1 );
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d ctxId=%d pos=(%d,%d)\n", 1, 0, pu.lumaPos().x, pu.lumaPos().y );
    }
    else
    {
      m_BinEncoder.encodeBinEP( 0 );
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d ctxId=%d pos=(%d,%d)\n", 0, 0, pu.lumaPos().x, pu.lumaPos().y );
      break;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
}

void CABACWriter::mvp_flag( const PredictionUnit& pu, RefPicList eRefList )
{
  m_BinEncoder.encodeBin( pu.mvpIdx[eRefList], Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", pu.mvpIdx[eRefList], pu.lumaPos().x, pu.lumaPos().y );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, pu.mvpIdx[eRefList] );
}



//================================================================================
//  clause 7.3.8.7
//--------------------------------------------------------------------------------
//    void  pcm_samples( tu )
//================================================================================

void CABACWriter::pcm_samples( const TransformUnit& tu )
{
  CHECK( !tu.cu->ipcm, "pcm mode expected" );

  const SPS&        sps       = *tu.cu->cs->sps;
  const ComponentID maxCompId = ( tu.chromaFormat == CHROMA_400 ? COMPONENT_Y : COMPONENT_Cr );
  for( ComponentID compID = COMPONENT_Y; compID <= maxCompId; compID = ComponentID(compID+1) )
  {
    const CPelBuf   samples     = tu.getPcmbuf( compID );
    const unsigned  sampleBits  = sps.getPCMBitDepth( toChannelType(compID) );
    for( unsigned y = 0; y < samples.height; y++ )
    {
      for( unsigned x = 0; x < samples.width; x++ )
      {
        m_BinEncoder.encodeBinsPCM( samples.at(x, y), sampleBits );
      }
    }
#if ENABLE_CHROMA_422
    if( tu.cs->pcv->multiBlock422 && compID != COMPONENT_Y )
    {
      const CPelBuf samples2 = tu.getPcmbuf( ComponentID( compID + SCND_TBLOCK_OFFSET ) );
      for( unsigned y = 0; y < samples2.height; y++ )
      {
        for( unsigned x = 0; x < samples2.width; x++ )
        {
          m_BinEncoder.encodeBinsPCM( samples2.at(x, y), sampleBits );
        }
      }
    }
#endif
  }
  m_BinEncoder.restart();
}



//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( split, depth )
//    bool  cbf_comp            ( cbf, area, depth )
//================================================================================

void CABACWriter::transform_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
{
  const UnitArea&       area          = partitioner.currArea();

#if HM_QTBT_AS_IN_JEM_SYNTAX
  if( cs.pcv->noRQT )
  {
    const TransformUnit &tu = *cs.getTU( area.blocks[partitioner.chType].pos(), partitioner.chType );

    transform_unit_qtbt( tu, cuCtx, chromaCbfs );

    return;
  }
#endif

  const TransformUnit&  tu            = *cs.getTU( area.blocks[partitioner.chType].pos(), partitioner.chType );
  const CodingUnit&     cu            = *tu.cu;
#if HEVC_USE_RQT || ENABLE_BMS
#if HEVC_USE_RQT
  const SPS&            sps           = *cs.sps;
  const unsigned        log2TrafoSize = g_aucLog2[area.lumaSize().width];
#endif
  const unsigned        trDepth       = partitioner.currTrDepth;
  const bool            split         = ( tu.depth > trDepth );

  // split_transform_flag
  if( cs.pcv->noRQT )
  {
#if ENABLE_BMS
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      CHECK( !split, "transform split implied" );
    }
    else
#endif
    CHECK( split, "transform split not allowed with QTBT" );
  }
#if HEVC_USE_RQT
#if HEVC_USE_PART_SIZE
  else if( CU::isIntra(cu) && cu.partSize == SIZE_NxN && trDepth == 0 )
  {
    CHECK( !split, "transform split must be true for Intra_NxN" );
  }
#endif
  else if( sps.getQuadtreeTUMaxDepthInter() == 1 && CU::isInter(cu) && cu.partSize != SIZE_2Nx2N && trDepth == 0 )
  {
    if( log2TrafoSize > cuCtx.quadtreeTULog2MinSizeInCU )
    {
      CHECK( !split, "transform split must be true for inferred split (for units greater than the minimum transform size)" );
    }
    else
    {
      CHECK( split,  "transform split must be false for inferred split (for units smaller than or equal to minimum transform size)" );
    }
  }
  else if( log2TrafoSize > sps.getQuadtreeTULog2MaxSize() )
  {
    CHECK( !split, "transform split must be true for units greater than the maximum transform size" );
  }
  else if( log2TrafoSize == sps.getQuadtreeTULog2MinSize() )
  {
    CHECK( split,  "transform split must be false for units equal to the minimum transform size" );
  }
  else if( log2TrafoSize == cuCtx.quadtreeTULog2MinSizeInCU )
  {
    CHECK( split,  "transform split must be false for maximum split depth" );
  }
  else
  {
    CHECK( log2TrafoSize <= cuCtx.quadtreeTULog2MinSizeInCU, "block cannot be split in multiple TUs" );

    if( sps.getSpsNext().nextToolsEnabled() )
    {
      split_transform_flag( split, sps.getQuadtreeTULog2MaxSize() - log2TrafoSize );
    }
    else
    {
      split_transform_flag( split, 5 - log2TrafoSize );
    }
  }
#endif
#endif

  // cbf_cb & cbf_cr
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !CS::isDualITree( cs ) || partitioner.chType == CHANNEL_TYPE_CHROMA ) )
  {
#if HEVC_USE_RQT
    const bool firstCbfOfCU   = ( trDepth == 0 );
    const bool allQuadrants = TU::isProcessingAllQuadrants( area );
#endif
#if ENABLE_CHROMA_422
    const bool twoChromaCbfs  = ( cs.pcv->multiBlock422 && ( !split || log2TrafoSize == 3 ) );
    if( twoChromaCbfs )
    {
      if( firstCbfOfCU || ( allQuadrants && chromaCbfs.Cb ) )
      {
        chromaCbfs.Cb   = TU::getCbfAtDepth( tu, COMPONENT_Cb,   trDepth );
        chromaCbfs.Cb2  = TU::getCbfAtDepth( tu, COMPONENT_Cb2,  trDepth );
        cbf_comp( cs, chromaCbfs.Cb,  area.blocks[   COMPONENT_Cb ], trDepth );
        cbf_comp( cs, chromaCbfs.Cb2, area.blocks[   COMPONENT_Cb ], trDepth );
      }
      else
      {
        bool   cbfCb  = ( TU::getCbfAtDepth( tu, COMPONENT_Cb,  trDepth ) ||
                          TU::getCbfAtDepth( tu, COMPONENT_Cb2, trDepth )    );
        CHECK( cbfCb != chromaCbfs.Cb, "incorrect Cb cbf" );
      }
      if( firstCbfOfCU || ( allQuadrants && chromaCbfs.Cr ) )
      {
        chromaCbfs.Cr   = TU::getCbfAtDepth( tu, COMPONENT_Cr,   trDepth );
        chromaCbfs.Cr2  = TU::getCbfAtDepth( tu, COMPONENT_Cr2,  trDepth );
        cbf_comp( cs, chromaCbfs.Cr,  area.blocks[   COMPONENT_Cr ], trDepth );
        cbf_comp( cs, chromaCbfs.Cr2, area.blocks[   COMPONENT_Cr ], trDepth );
      }
      else
      {
        bool   cbfCr  = ( TU::getCbfAtDepth( tu, COMPONENT_Cr,  trDepth ) ||
                          TU::getCbfAtDepth( tu, COMPONENT_Cr2, trDepth )    );
        CHECK( cbfCr != chromaCbfs.Cr, "incorrect Cr cbf" );
      }
    }
    else
#endif
    {
#if HEVC_USE_RQT
      if( firstCbfOfCU || ( allQuadrants && chromaCbfs.Cb ) )
#elif ENABLE_BMS
      if( trDepth == 0 || chromaCbfs.Cb )
#endif
      {
#if HEVC_USE_RQT || ENABLE_BMS
        chromaCbfs.Cb = TU::getCbfAtDepth( tu, COMPONENT_Cb, trDepth );
        cbf_comp( cs, chromaCbfs.Cb, area.blocks[COMPONENT_Cb], trDepth );
#else
        chromaCbfs.Cb = TU::getCbf( tu, COMPONENT_Cb );
        cbf_comp( cs, chromaCbfs.Cb, area.blocks[COMPONENT_Cb] );
#endif
      }
#if HEVC_USE_RQT || ENABLE_BMS
      else
      {
        CHECK( TU::getCbfAtDepth( tu, COMPONENT_Cb, trDepth ) != chromaCbfs.Cb, "incorrect Cb cbf" );
      }

#if HEVC_USE_RQT
      if( firstCbfOfCU || ( allQuadrants && chromaCbfs.Cr ) )
#else
      if( trDepth == 0 || chromaCbfs.Cr )
#endif
#endif
      {
#if HEVC_USE_RQT || ENABLE_BMS
        chromaCbfs.Cr = TU::getCbfAtDepth( tu, COMPONENT_Cr,   trDepth );
        cbf_comp( cs, chromaCbfs.Cr, area.blocks[COMPONENT_Cr], trDepth );
#else
        chromaCbfs.Cr = TU::getCbf( tu, COMPONENT_Cr );
        cbf_comp( cs, chromaCbfs.Cr, area.blocks[COMPONENT_Cr] );
#endif
      }
#if HEVC_USE_RQT || ENABLE_BMS
      else
      {
        CHECK( TU::getCbfAtDepth( tu, COMPONENT_Cr, trDepth ) != chromaCbfs.Cr, "incorrect Cr cbf" );
      }
#endif
    }
  }
  else if( CS::isDualITree( cs ) )
  {
    chromaCbfs = ChromaCbfs( false );
  }

#if HEVC_USE_RQT || ENABLE_BMS
  if( split )
  {
    if( area.chromaFormat != CHROMA_400 )
    {
      chromaCbfs.Cb        = TU::getCbfAtDepth( tu, COMPONENT_Cb,  trDepth );
      chromaCbfs.Cr        = TU::getCbfAtDepth( tu, COMPONENT_Cr,  trDepth );
#if ENABLE_CHROMA_422
      if( cs.pcv->multiBlock422 )
      {
        chromaCbfs.Cb     |= TU::getCbfAtDepth( tu, COMPONENT_Cb2, trDepth );
        chromaCbfs.Cr     |= TU::getCbfAtDepth( tu, COMPONENT_Cr2, trDepth );
      }
#endif
    }
#if INTRA_KLT_MATRIX & 0
    if (trDepth == 0) klt_cu_flag(cu);
#endif

#if ENABLE_BMS
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
#if ENABLE_TRACING
      const CompArea &tuArea = partitioner.currArea().blocks[partitioner.chType];
      DTRACE( g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n", partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height );

#endif
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else
#endif
#if HEVC_USE_RQT
    partitioner.splitCurrArea( TU_QUAD_SPLIT, cs );
#else
      THROW( "Implicit TU split not available" );
#endif

    do
    {
      ChromaCbfs subChromaCbfs = chromaCbfs;
      transform_tree( cs, partitioner, cuCtx, subChromaCbfs );
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
#endif
  {
#if HEVC_USE_RQT || ENABLE_BMS
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );
#else
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth );
#endif

    if( !isChroma( partitioner.chType ) )
    {
#if HEVC_USE_RQT || ENABLE_BMS
      if( !CU::isIntra( cu ) && trDepth == 0 && !chromaCbfs.sigChroma( area.chromaFormat ) )
      {
        CHECK( !TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), "Luma cbf must be true for inter units with no chroma coeffs" );
      }
#else
      if( !CU::isIntra( cu ) && !chromaCbfs.sigChroma( area.chromaFormat ) )
      {
        CHECK( !TU::getCbf( tu, COMPONENT_Y ), "Luma cbf must be true for inter units with no chroma coeffs" );
      }
#endif
      else
      {
#if HEVC_USE_RQT || ENABLE_BMS
        cbf_comp( cs, TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), tu.Y(), trDepth );
#else
        cbf_comp( cs, TU::getCbf( tu, COMPONENT_Y ), tu.Y() );
#endif
      }
    }

#if INTRA_KLT_MATRIX & 0
#if HEVC_USE_RQT || ENABLE_BMS
    if( trDepth == 0 ) klt_cu_flag( cu );
#else
    klt_cu_flag( cu );
#endif
#endif

    transform_unit( tu, cuCtx, chromaCbfs );
  }
}

#if HEVC_USE_RQT
void CABACWriter::split_transform_flag( bool split, unsigned depth )
{
  m_BinEncoder.encodeBin( split, Ctx::TransSubdivFlag( depth ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "split_transform_flag() ctx=%d split=%d\n", depth, split );
}

#endif
#if HEVC_USE_RQT || ENABLE_BMS
void CABACWriter::cbf_comp( const CodingStructure& cs, bool cbf, const CompArea& area, unsigned depth )
#else
void CABACWriter::cbf_comp( const CodingStructure& cs, bool cbf, const CompArea& area )
#endif
{
#if HEVC_USE_RQT || ENABLE_BMS
  const unsigned  ctxId   = DeriveCtx::CtxQtCbf( area.compID, depth );
#else
  const unsigned  ctxId   = DeriveCtx::CtxQtCbf( area.compID );
#endif
  const CtxSet&   ctxSet  = Ctx::QtCbf[ toChannelType(area.compID) ];
  m_BinEncoder.encodeBin( cbf, ctxSet( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
}





//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================

void CABACWriter::mvd_coding( const Mv &rMvd )
{
  int       horMvd = rMvd.getHor();
  int       verMvd = rMvd.getVer();
  unsigned  horAbs  = unsigned( horMvd < 0 ? -horMvd : horMvd );
  unsigned  verAbs  = unsigned( verMvd < 0 ? -verMvd : verMvd );


  // abs_mvd_greater0_flag[ 0 | 1 ]
  m_BinEncoder.encodeBin( (horAbs > 0), Ctx::Mvd() );
  m_BinEncoder.encodeBin( (verAbs > 0), Ctx::Mvd() );

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    m_BinEncoder.encodeBin( (horAbs > 1), Ctx::Mvd(1) );
  }
  if( verAbs > 0 )
  {
    m_BinEncoder.encodeBin( (verAbs > 1), Ctx::Mvd(1) );
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    if( horAbs > 1 )
    {
      exp_golomb_eqprob( horAbs - 2, 1 );
    }
    m_BinEncoder.encodeBinEP( (horMvd < 0) );
  }
  if( verAbs > 0 )
  {
    if( verAbs > 1 )
    {
      exp_golomb_eqprob( verAbs - 2, 1 );
    }
    m_BinEncoder.encodeBinEP( (verMvd < 0) );
  }
}




//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================

void CABACWriter::transform_unit( const TransformUnit& tu, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
{
  CodingUnit& cu        = *tu.cu;
#if HEVC_USE_RQT
  int         currDepth = tu.depth;
#endif
  bool        lumaOnly  = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
#if ENABLE_CHROMA_422
  bool        cbf[5]    = { TU::getCbfAtDepth( tu, COMPONENT_Y,  currDepth ), chromaCbfs.Cb, chromaCbfs.Cr, chromaCbfs.Cb2, chromaCbfs.Cr2 };
#else
#if HEVC_USE_RQT
  bool        cbf[3]    = { TU::getCbfAtDepth( tu, COMPONENT_Y,  currDepth ), chromaCbfs.Cb, chromaCbfs.Cr };
#else
  bool        cbf[3]    = { TU::getCbf( tu, COMPONENT_Y ), chromaCbfs.Cb, chromaCbfs.Cr };
#endif
#endif
  bool        cbfLuma   = ( cbf[ COMPONENT_Y ] != 0 );
  bool        cbfChroma = false;


  if( cu.chromaFormat != CHROMA_400 )
  {
    if( tu.blocks[COMPONENT_Cb].valid() )
    {
#if HEVC_USE_RQT
      cbf   [ COMPONENT_Cb  ] = TU::getCbfAtDepth( tu, COMPONENT_Cb,  currDepth );
      cbf   [ COMPONENT_Cr  ] = TU::getCbfAtDepth( tu, COMPONENT_Cr,  currDepth );
#if ENABLE_CHROMA_422
      if( cu.cs->pcv->multiBlock422 )
      {
        cbf [ COMPONENT_Cb2 ] = TU::getCbfAtDepth( tu, COMPONENT_Cb2, currDepth );
        cbf [ COMPONENT_Cr2 ] = TU::getCbfAtDepth( tu, COMPONENT_Cr2, currDepth );
      }
#endif
#else
      cbf   [ COMPONENT_Cb  ] = TU::getCbf( tu, COMPONENT_Cb );
      cbf   [ COMPONENT_Cr  ] = TU::getCbf( tu, COMPONENT_Cr );
#endif
    }
#if ENABLE_CHROMA_422
    cbfChroma = ( cbf[ COMPONENT_Cb ] || cbf[ COMPONENT_Cr ] || ( cu.cs->pcv->multiBlock422 && ( cbf[ COMPONENT_Cb2 ] || cbf[ COMPONENT_Cr2 ] ) ) );
#else
    cbfChroma = ( cbf[ COMPONENT_Cb ] || cbf[ COMPONENT_Cr ] );
#endif
  }
  if( cbfLuma || cbfChroma )
  {
    if( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      cu_qp_delta( cu, cuCtx.qp, cu.qp );
      cuCtx.qp         = cu.qp;
      cuCtx.isDQPCoded = true;
    }
    if( cu.cs->slice->getUseChromaQpAdj() && cbfChroma && !cu.transQuantBypass && !cuCtx.isChromaQpAdjCoded )
    {
      cu_chroma_qp_offset( cu );
      cuCtx.isChromaQpAdjCoded = true;
    }
    if( cbfLuma )
    {
      residual_coding( tu, COMPONENT_Y );
    }
    if( !lumaOnly )
    {
      for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
      {
        if( TU::hasCrossCompPredInfo( tu, compID ) )
        {
          cross_comp_pred( tu, compID );
        }
        if( cbf[ compID ] )
        {
          residual_coding( tu, compID );
        }
#if ENABLE_CHROMA_422
        if( cu.cs->pcv->multiBlock422 )
        {
          if( cbf[ compID + SCND_TBLOCK_OFFSET ] )
          {
            residual_coding( tu, ComponentID(compID+SCND_TBLOCK_OFFSET) );
          }
        }
#endif
      }
    }
  }
}

#if HM_QTBT_AS_IN_JEM_SYNTAX
void CABACWriter::transform_unit_qtbt( const TransformUnit& tu, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
{
  CodingUnit& cu  = *tu.cu;
  bool cbfLuma    = false;
  bool cbfChroma  = false;

  bool lumaOnly   = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  bool chromaOnly =                                    !tu.blocks[COMPONENT_Y ].valid();

  if( !lumaOnly )
  {
    for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
    {
#if HEVC_USE_RQT || ENABLE_BMS
      cbf_comp( *tu.cs, tu.cbf[compID] != 0, tu.blocks[compID], tu.depth );
#else
      cbf_comp( *tu.cs, tu.cbf[compID] != 0, tu.blocks[compID] );
#endif
      chromaCbfs.cbf( compID ) = tu.cbf[compID] != 0;

      if( TU::hasCrossCompPredInfo( tu, compID ) )
      {
        cross_comp_pred( tu, compID );
      }
      if( tu.cbf[compID] )
      {
        residual_coding( tu, compID );
        cbfChroma = true;
      }
    }
  }

  if( !chromaOnly )
  {
    if( !CU::isIntra( cu ) && !chromaCbfs.sigChroma( tu.chromaFormat ) )
    {
#if HEVC_USE_RQT || ENABLE_BMS
      CHECK( !TU::getCbfAtDepth( tu, COMPONENT_Y, 0 ), "The luma CBF is implicitely '1', but '0' found" );
#else
      CHECK( !TU::getCbf( tu, COMPONENT_Y ), "The luma CBF is implicitely '1', but '0' found" );
#endif
    }
    else
    {
#if HEVC_USE_RQT || ENABLE_BMS
      cbf_comp( *tu.cs, TU::getCbf( tu, COMPONENT_Y ), tu.Y(), tu.depth );
#else
      cbf_comp( *tu.cs, TU::getCbf( tu, COMPONENT_Y ), tu.Y() );
#endif
    }

    if( tu.cbf[0] )
    {
      residual_coding( tu, COMPONENT_Y );
      cbfLuma = true;
    }
  }

  if( cbfLuma || cbfChroma )
  {
    if( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      cu_qp_delta( cu, cuCtx.qp, cu.qp );
      cuCtx.qp         = cu.qp;
      cuCtx.isDQPCoded = true;
    }
    if( cu.cs->slice->getUseChromaQpAdj() && cbfChroma && !cu.transQuantBypass && !cuCtx.isChromaQpAdjCoded )
    {
      cu_chroma_qp_offset( cu );
      cuCtx.isChromaQpAdjCoded = true;
    }
  }
}
#endif

void CABACWriter::cu_qp_delta( const CodingUnit& cu, int predQP, const SChar qp )
{
  CHECK(!( predQP != std::numeric_limits<int>::max()), "Unspecified error");
  int       DQp         = qp - predQP;
  int       qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
  DQp                   = ( DQp + 78 + qpBdOffsetY + ( qpBdOffsetY / 2 ) ) % ( 52 + qpBdOffsetY ) - 26 - ( qpBdOffsetY / 2 );
  unsigned  absDQP      = unsigned( DQp < 0 ? -DQp : DQp );
  unsigned  unaryDQP    = std::min<unsigned>( absDQP, CU_DQP_TU_CMAX );

  unary_max_symbol( unaryDQP, Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( absDQP >= CU_DQP_TU_CMAX )
  {
    exp_golomb_eqprob( absDQP - CU_DQP_TU_CMAX, CU_DQP_EG_k );
  }
  if( absDQP > 0 )
  {
    m_BinEncoder.encodeBinEP( DQp < 0 );
  }

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACWriter::cu_chroma_qp_offset( const CodingUnit& cu )
{
  // cu_chroma_qp_offset_flag
  unsigned qpAdj = cu.chromaQpAdj;
  if( qpAdj == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ChromaQpAdjFlag() );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ChromaQpAdjFlag() );
    int length = cu.cs->pps->getPpsRangeExtension().getChromaQpOffsetListLen();
    if( length > 1 )
    {
      unary_max_symbol( qpAdj-1, Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
    }
  }
}





//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    void        transform_skip_flag     ( tu, compID )
//    void        explicit_rdpcm_mode     ( tu, compID )
//    void        last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACWriter::residual_coding( const TransformUnit& tu, ComponentID compID )
{
#if ENABLE_TRACING || HEVC_USE_SIGN_HIDING || INTRA_KLT_MATRIX
  const CodingUnit& cu = *tu.cu;
#endif
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );

  // code transform skip and explicit rdpcm mode
  transform_skip_flag( tu, compID );
  explicit_rdpcm_mode( tu, compID );

#if HEVC_USE_SIGN_HIDING
  // determine sign hiding
  bool signHiding  = ( cu.cs->pps->getSignDataHidingEnabledFlag() && !cu.transQuantBypass && tu.rdpcm[compID] == RDPCM_OFF );
  if(  signHiding && CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && tu.transformSkip[compID] )
  {
    const ChannelType chType    = toChannelType( compID );
    const unsigned    intraMode = PU::getFinalIntraMode( *cu.cs->getPU( tu.blocks[compID].pos(), chType ), chType );
    if( intraMode == HOR_IDX || intraMode == VER_IDX )
    {
      signHiding = false;
    }
  }
#endif

  // init coeff coding context
#if HEVC_USE_SIGN_HIDING
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
#else
  CoeffCodingContext  cctx    ( tu, compID );
#endif
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;
  unsigned&           GRStats = m_BinEncoder.getCtx().getGRAdaptStats( TU::getGolombRiceStatisticsIndex( tu, compID ) );

  // determine and set last coeff position and sig group flags
  int                      scanPosLast = -1;
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      scanPosLast = scanPos;
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }
  CHECK( scanPosLast < 0, "Coefficient coding called for empty TU" );
  cctx.setScanPosLast(scanPosLast);

  // code last coeff position
  last_sig_coeff( cctx );

  // code subblocks
  cctx.setGoRiceStats( GRStats );
#if INTRA_KLT_MATRIX
  bool useEmt = ( cu.cs->sps->getSpsNext().getUseIntraKLT() && cu.predMode == MODE_INTRA ) || ( cu.cs->sps->getSpsNext().getUseInterKLT() && cu.predMode != MODE_INTRA );
  useEmt = useEmt && isLuma(compID);
#endif

  for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock       ( subSetId, sigGroupFlags[subSetId] );
    residual_coding_subblock( cctx, coeff );

  }
  GRStats = cctx.currGoRiceStats();
#if INTRA_KLT_MATRIX
  if (useEmt)
  {
    if (!tu.transformSkip[compID])
    {
      klt_tu_index(tu);
    }
    else
    {
      CHECK(tu.kltIdx != 0, "Error KLT Index");
      CHECK((*tu.cu).kltFlag != 0, "Error KLT Flag");
    }
  }
#endif
}


void CABACWriter::transform_skip_flag( const TransformUnit& tu, ComponentID compID )
{
  if( !tu.cu->cs->pps->getUseTransformSkip() || tu.cu->transQuantBypass || !TU::hasTransformSkipFlag( *tu.cs, tu.blocks[compID] ) )
  {
    return;
  }
  m_BinEncoder.encodeBin( tu.transformSkip[compID], Ctx::TransformSkipFlag(toChannelType(compID)) );

  DTRACE( g_trace_ctx, D_SYNTAX, "transform_skip_flag() etype=%d pos=(%d,%d) trSkip=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, (int)tu.transformSkip[compID] );
}




void CABACWriter::explicit_rdpcm_mode( const TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu = *tu.cu;
  if( !CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && ( tu.transformSkip[compID] || cu.transQuantBypass ) )
  {
    ChannelType chType = toChannelType( compID );
    switch( tu.rdpcm[compID] )
    {
    case RDPCM_VER:
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmFlag(chType) );
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmDir (chType) );
      break;
    case RDPCM_HOR:
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmFlag(chType) );
      m_BinEncoder.encodeBin( 0, Ctx::RdpcmDir (chType) );
      break;
    default: // RDPCM_OFF
      m_BinEncoder.encodeBin( 0, Ctx::RdpcmFlag(chType) );
    }
  }
}


void CABACWriter::last_sig_coeff( CoeffCodingContext& cctx )
{
  unsigned blkPos = cctx.blockPos( cctx.scanPosLast() );
  unsigned posX, posY;
#if HEVC_USE_MDCS
  if( cctx.scanType() == SCAN_VER )
  {
    posX  = blkPos / cctx.width();
    posY  = blkPos - ( posX * cctx.width() );
  }
  else
#endif
  {
    posY  = blkPos / cctx.width();
    posX  = blkPos - ( posY * cctx.width() );
  }

  unsigned CtxLast;
  unsigned GroupIdxX = g_uiGroupIdx[ posX ];
  unsigned GroupIdxY = g_uiGroupIdx[ posY ];

  for( CtxLast = 0; CtxLast < GroupIdxX; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastXCtxId( CtxLast ) );
  }
  if( GroupIdxX < cctx.maxLastPosX() )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastXCtxId( CtxLast ) );
  }
  for( CtxLast = 0; CtxLast < GroupIdxY; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxY < cctx.maxLastPosY() )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxX > 3 )
  {
    posX -= g_uiMinInGroup[ GroupIdxX ];
    for (Int i = ( ( GroupIdxX - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posX >> i ) & 1 );
    }
  }
  if( GroupIdxY > 3 )
  {
    posY -= g_uiMinInGroup[ GroupIdxY ];
    for ( Int i = ( ( GroupIdxY - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posY >> i ) & 1 );
    }
  }
}




void CABACWriter::residual_coding_subblock( CoeffCodingContext& cctx, const TCoeff* coeff )
{
  //===== init =====
  const int   maxSbbSize  = 1 << cctx.log2CGSize();
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         nextSigPos  = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );

  //===== encode significant_coeffgroup_flag =====
  if( !isLast && cctx.isNotFirst() )
  {
    if( cctx.isSigGroup() )
    {
      m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId() );
    }
    else
    {
      m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  {
    //===== encode significant_coeff_flag's =====
    const int inferSigPos = ( cctx.isNotFirst() ? minSubPos : -1 );
    unsigned  numNonZero  = 0;
#if HEVC_USE_SIGN_HIDING
    int       firstNZPos  = maxSbbSize;
    int       lastNZPos   = -1;
#endif
    int       absCoeff    [ 1 << MLS_CG_SIZE ];
    unsigned  signPattern = 0;
    if( isLast )
    {
#if HEVC_USE_SIGN_HIDING
      firstNZPos                    = nextSigPos;
      lastNZPos                     = std::max<int>( lastNZPos, nextSigPos );
#endif
      TCoeff    Coeff               = coeff[ cctx.blockPos( nextSigPos-- ) ];
      absCoeff[ numNonZero++ ]      = ( Coeff > 0 ? Coeff : ( signPattern++, -Coeff ) );
    }
    for( ; nextSigPos >= minSubPos; nextSigPos-- )
    {
      TCoeff    Coeff               = coeff[ cctx.blockPos( nextSigPos ) ];
      unsigned  sigFlag             = ( Coeff != 0 );
      if( numNonZero || nextSigPos != inferSigPos )
      {
        m_BinEncoder.encodeBin( sigFlag, cctx.sigCtxId( nextSigPos ) );
      }
      if( sigFlag )
      {
#if HEVC_USE_SIGN_HIDING
        firstNZPos                = nextSigPos;
        lastNZPos                 = std::max<int>( lastNZPos, nextSigPos );
#endif
        signPattern             <<= 1;
        absCoeff[ numNonZero++ ]  = ( Coeff > 0 ? Coeff : ( signPattern++, -Coeff ) );
      }
    }


    //===== decode abs_greater1_flag's =====
    const unsigned  numGt1Flags = std::min<unsigned>( numNonZero, C1FLAG_NUMBER );
    int             gt2FlagIdx  = maxSbbSize;
    bool            escapeData  = false;
    uint16_t        ctxGt1Id    = 1;
    for( unsigned k = 0; k < numGt1Flags; k++ )
    {
      if( absCoeff[ k ] > 1 )
      {
        m_BinEncoder.encodeBin( 1, cctx.greater1CtxId( ctxGt1Id ) );
        ctxGt1Id      = 0;
        if( gt2FlagIdx < maxSbbSize )
        {
          escapeData  = true;
        }
        else
        {
          gt2FlagIdx  = k;
        }
      }
      else
      {
        m_BinEncoder.encodeBin( 0, cctx.greater1CtxId( ctxGt1Id ) );
        if( ctxGt1Id && ctxGt1Id < 3 )
        {
          ctxGt1Id++;
        }
      }
    }
    escapeData = escapeData || ( numGt1Flags < numNonZero );
    cctx.setGt2Flag( ctxGt1Id == 0 );


    //===== decode abs_greater2_flag =====
    if( gt2FlagIdx < maxSbbSize )
    {
      if( absCoeff[ gt2FlagIdx ] > 2 )
      {
        m_BinEncoder.encodeBin( 1, cctx.greater2CtxId() );
        escapeData  = true;
      }
      else
      {
        m_BinEncoder.encodeBin( 0, cctx.greater2CtxId() );
      }
    }


    //===== align data =====
    if( escapeData && cctx.alignFlag() )
    {
      m_BinEncoder.align();
    }


    //===== decode sign's =====
#if HEVC_USE_SIGN_HIDING
    unsigned numSigns = numNonZero;
    if( cctx.hideSign( firstNZPos, lastNZPos ) )
    {
      numSigns    --;
      signPattern >>= 1;
    }
    m_BinEncoder.encodeBinsEP( signPattern, numSigns );
#else
    m_BinEncoder.encodeBinsEP( signPattern, numNonZero );
#endif

    //===== decode remaining absolute values =====
    if( escapeData )
    {
      bool      updateGoRiceStats = cctx.updGoRiceStats();
      unsigned  GoRicePar         = cctx.currGoRiceStats() >> 2;
      unsigned  MaxGoRicePar      = ( updateGoRiceStats ? std::numeric_limits<unsigned>::max() : 4 );
      int       baseLevel         = 3;
      for( int k = 0; k < numNonZero; k++ )
      {
        if( absCoeff[ k ] >= baseLevel )
        {
          int remAbs    = absCoeff[ k ] - baseLevel;
          m_BinEncoder.encodeRemAbsEP( remAbs, GoRicePar, cctx.extPrec(), cctx.maxLog2TrDRange() );

          // update rice parameter
          if( absCoeff[ k ] > ( 3 << GoRicePar ) )
          {
            GoRicePar = std::min<unsigned>( MaxGoRicePar, GoRicePar + 1 );
          }
          if( updateGoRiceStats )
          {
            unsigned initGoRicePar = cctx.currGoRiceStats() >> 2;
            if( remAbs >= ( 3 << initGoRicePar) )
            {
              cctx.incGoRiceStats();
            }
            else if( cctx.currGoRiceStats() > 0 && ( remAbs << 1 ) < ( 1 << initGoRicePar ) )
            {
              cctx.decGoRiceStats();
            }
            updateGoRiceStats = false;
          }
        }
        if( k > C1FLAG_NUMBER - 2 )
        {
          baseLevel = 1;
        }
        else if( baseLevel == 3 && absCoeff[ k ] > 1 )
        {
          baseLevel = 2;
        }
      }
    }
  }
}





//================================================================================
//  clause 7.3.8.12
//--------------------------------------------------------------------------------
//    void  cross_comp_pred( tu, compID )
//================================================================================

void CABACWriter::cross_comp_pred( const TransformUnit& tu, ComponentID compID )
{
  CHECK(!( !isLuma( compID ) ), "Unspecified error");
  signed char alpha   = tu.compAlpha[compID];
  unsigned    ctxBase = ( compID == COMPONENT_Cr ? 5 : 0 );
  if( alpha == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::CrossCompPred( ctxBase ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
    return;
  }

  static const unsigned log2AbsAlphaMinus1Table[8] = { 0, 1, 1, 2, 2, 2, 3, 3 };
  unsigned sign = ( alpha < 0 );
  if( sign )
  {
    alpha = -alpha;
  }
  CHECK(!( alpha <= 8 ), "Unspecified error");
  m_BinEncoder.encodeBin( 1, Ctx::CrossCompPred(ctxBase) );
  if( alpha > 1)
  {
     m_BinEncoder.encodeBin( 1, Ctx::CrossCompPred(ctxBase+1) );
     unary_max_symbol( log2AbsAlphaMinus1Table[alpha-1]-1, Ctx::CrossCompPred(ctxBase+2), Ctx::CrossCompPred(ctxBase+3), 2 );
  }
  else
  {
     m_BinEncoder.encodeBin( 0, Ctx::CrossCompPred(ctxBase+1) );
  }
  m_BinEncoder.encodeBin( sign, Ctx::CrossCompPred(ctxBase+4) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
}




//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    void  unary_max_symbol  ( symbol, ctxId0, ctxIdN, maxSymbol )
//    void  unary_max_eqprob  ( symbol,                 maxSymbol )
//    void  exp_golomb_eqprob ( symbol, count )
//================================================================================

void CABACWriter::unary_max_symbol( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol )
{
  CHECK( symbol > maxSymbol, "symbol > maxSymbol" );
  const unsigned totalBinsToWrite = std::min( symbol + 1, maxSymbol );
  for( unsigned binsWritten = 0; binsWritten < totalBinsToWrite; ++binsWritten )
  {
    const unsigned nextBin = symbol > binsWritten;
    m_BinEncoder.encodeBin( nextBin, binsWritten == 0 ? ctxId0 : ctxIdN );
  }
}


void CABACWriter::unary_max_eqprob( unsigned symbol, unsigned maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return;
  }
  bool     codeLast = ( maxSymbol > symbol );
  unsigned bins     = 0;
  unsigned numBins  = 0;
  while( symbol-- )
  {
    bins   <<= 1;
    bins   ++;
    numBins++;
  }
  if( codeLast )
  {
    bins  <<= 1;
    numBins++;
  }
  CHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}


void CABACWriter::exp_golomb_eqprob( unsigned symbol, unsigned count )
{
  unsigned bins    = 0;
  unsigned numBins = 0;
  while( symbol >= (unsigned)(1<<count) )
  {
    bins <<= 1;
    bins++;
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  bins <<= 1;
  numBins++;
  bins = (bins << count) | symbol;
  numBins += count;
  CHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}

void CABACWriter::encode_sparse_dt( DecisionTree& dt, unsigned toCodeId )
{
  // propagate the sparsity information from end-nodes to intermediate nodes
  dt.reduce();

  unsigned depth  = dt.dtt.depth;
  unsigned offset = 0;

  const unsigned encElPos = dt.dtt.mapping[toCodeId];

  while( dt.dtt.hasSub[offset] )
  {
    CHECKD( depth == 0, "Depth is '0' for a decision node in a decision tree" );

    const unsigned posRight = offset + 1;
    const unsigned posLeft  = offset + ( 1u << depth );

    const bool isLeft = encElPos >= posLeft;

    if( dt.isAvail[posRight] && dt.isAvail[posLeft] )
    {
      // encode the decision as both sub-paths are available
      const unsigned ctxId = dt.ctxId[offset];

      if( ctxId > 0 )
      {
        DTRACE( g_trace_ctx, D_DECISIONTREE, "Decision coding using context %d\n", ctxId - 1 );
        m_BinEncoder.encodeBin( isLeft ? 0 : 1, ctxId - 1 );
      }
      else
      {
        DTRACE( g_trace_ctx, D_DECISIONTREE, "Decision coding as an EP bin\n" );
        m_BinEncoder.encodeBinEP( isLeft ? 0 : 1 );
      }
    }

    DTRACE( g_trace_ctx, D_DECISIONTREE, "Following the tree to the %s sub-node\n", isLeft ? "left" : "right" );

    offset = isLeft ? posLeft : posRight;
    depth--;
  }

  CHECKD( offset != encElPos,             "Encoded a different element than assigned" );
  CHECKD( dt.dtt.ids[offset] != toCodeId, "Encoded a different element than assigned" );
  CHECKD( dt.isAvail[offset] == false,    "The encoded element is not available" );
  DTRACE( g_trace_ctx, D_DECISIONTREE,    "Found an end-node of the tree\n" );
  return;
}

#if INTRA_KLT_MATRIX
Void CABACWriter::klt_cu_flag( const CodingUnit& cu )
{
  const CodingStructure& cs = *cu.cs;

  if( !( ( cs.sps->getSpsNext().getUseIntraKLT() && CU::isIntra( cu ) ) || ( cs.sps->getSpsNext().getUseInterKLT() && CU::isInter( cu ) ) ) || isChroma( cu.chType ) )
  {
    return;
  }

  unsigned depth          = cu.qtDepth;
  const unsigned cuWidth  = cu.lwidth();
  const unsigned cuHeight = cu.lheight();

  if (cuWidth <= KLTSPLIT_INTRA_MIN_CU && cuHeight <= KLTSPLIT_INTRA_MIN_CU)
  {
    m_BinEncoder.encodeBin( cu.kltFlag, Ctx::KLTCuFlag( depth ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_cu_flag() etype=%d pos=(%d,%d) emtCuFlag=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.kltFlag );
  }
}

Void CABACWriter::klt_tu_index( const TransformUnit& tu )
{
  int maxSizeEmtIntra, maxSizeEmtInter;
  if( tu.cs->pcv->noRQT )
  {
    maxSizeEmtIntra = EMT_INTRA_MAX_CU_WITH_QTBT;
    maxSizeEmtInter = EMT_INTER_MAX_CU_WITH_QTBT;
  }
  else
  {
    maxSizeEmtIntra = EMT_INTRA_MAX_CU;
    maxSizeEmtInter = EMT_INTER_MAX_CU;
  }

  SizeType lumaWidth = tu.cu->Y().width;
  SizeType lumaHeight = tu.cu->Y().height;
  if( CU::isIntra( *tu.cu ) && (lumaWidth <= maxSizeEmtIntra ) && (lumaHeight <= maxSizeEmtIntra ) )
  {
    if ( (lumaWidth <= KLTSPLIT_INTRA_MIN_CU) && (lumaHeight <= KLTSPLIT_INTRA_MIN_CU) )
    {
      CHECK(tu.kltIdx != 0, "Error KLT Index for small blocks");
      m_BinEncoder.encodeBin(((*tu.cu).kltFlag), Ctx::KLTCuFlag(0));
    }
    else
    {
      UChar trIdx = tu.kltIdx + (UChar)((*tu.cu).kltFlag); //! 0: kltFlag = 0, 1~3: kltFlag = 1, klt index
      if (trIdx == 1)
      {
        CHECK(!((*tu.cu).kltFlag), "Error Klt Signal");
        m_BinEncoder.encodeBin(1, Ctx::KLTTuIndex(0));
      }
      else
      {
        m_BinEncoder.encodeBin(0, Ctx::KLTTuIndex(0));
        if (trIdx == 3)
        {
          m_BinEncoder.encodeBin(1, Ctx::KLTTuIndex(1));
        }
        else
        {
          m_BinEncoder.encodeBin(0, Ctx::KLTTuIndex(1));
          m_BinEncoder.encodeBin((trIdx == 2), Ctx::KLTTuIndex(2));
        }
      }
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_tu_index() etype=%d pos=(%d,%d) emtTrIdx=%d\n", COMPONENT_Y, tu.blocks[COMPONENT_Y].x, tu.blocks[COMPONENT_Y].y, ( int ) tu.kltIdx );
  }
  if( !CU::isIntra( *tu.cu ) && ( tu.cu->Y().width <= maxSizeEmtInter ) && ( tu.cu->Y().height <= maxSizeEmtInter ) )
  {
    UChar trIdx = tu.kltIdx;
    m_BinEncoder.encodeBin( ( trIdx & 1 ) ? 1 : 0, Ctx::KLTTuIndex( 2 ) );
    m_BinEncoder.encodeBin( ( trIdx / 2 ) ? 1 : 0, Ctx::KLTTuIndex( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_tu_index() etype=%d pos=(%d,%d) emtTrIdx=%d\n", COMPONENT_Y, tu.blocks[COMPONENT_Y].x, tu.blocks[COMPONENT_Y].y, ( int ) tu.kltIdx );
  }
}
#endif

//! \}
