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

/** \file     TrQuant_EMT.h
    \brief    transform and quantization class (header)
*/

#ifndef __TRQUANT_EMT__
#define __TRQUANT_EMT__

#include "CommonDef.h"

////DCT-II transforms
void fastForwardDCT2_B2  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar);
void fastInverseDCT2_B2  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar);
void fastForwardDCT2_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar);
void fastInverseDCT2_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar);
void fastForwardDCT2_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar);
void fastInverseDCT2_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar);
void fastForwardDCT2_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar);
void fastInverseDCT2_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar);
void fastForwardDCT2_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar);
void fastInverseDCT2_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar);
void fastForwardDCT2_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar);
void fastInverseDCT2_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar);
void fastForwardDCT2_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar);
void fastInverseDCT2_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar);

//DST-VII transforms
void fastForwardDST7_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);

#if INTRA_KLT_MATRIX
void fastForwardKLT_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar uhKLTIdx);
void fastInverseKLT_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar uhKLTIdx);
void fastForwardKLT_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar uhKLTIdx);
void fastInverseKLT_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar uhKLTIdx);
void fastForwardKLT_B16(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar uhKLTIdx);
void fastInverseKLT_B16(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar uhKLTIdx);
void fastForwardKLT_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar uhKLTIdx);
void fastInverseKLT_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar uhKLTIdx);
void fastForwardKLT_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar uhKLTIdx);
void fastInverseKLT_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar uhKLTIdx);
void fastForwardKLT_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, UChar uhKLTIdx);
void fastInverseKLT_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum, UChar uhKLTIdx);
#endif
#endif // __TRQUANT__
