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

/** \file     TrQuant_EMT.cpp
    \brief    transform and quantization class
*/

#include "TrQuant_EMT.h"

#include "Rom.h"

#include <stdlib.h>
#include <math.h>
#include <limits>
#include <memory.h>

#if x86_SSE_128
#include <emmintrin.h>
#include <smmintrin.h>
#endif

// ********************************** DCT-II **********************************

//Fast DCT-II transforms
void fastForwardDCT2_B2(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use)
{
  Int j;
  Int E, O;
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr2[DCT2][0] : g_aiT2[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const Int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* E and O */
    E = src[0] + src[1];
    O = src[0] - src[1];

    dst[0] = (iT[0] * E + add) >> shift;
    dst[line] = (iT[2] * O + add) >> shift;


    src += 2;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j<2; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

void fastInverseDCT2_B2(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j;
  Int E, O;
  Int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr2[DCT2][0] : g_aiT2[TRANSFORM_INVERSE][0];

  const Int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    E = iT[0] * (src[0] + src[line]);
    O = iT[2] * (src[0] - src[line]);

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3(outputMinimum, outputMaximum, (E + add) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (O + add) >> shift);

    src++;
    dst += 2;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 1) * sizeof(TCoeff));
  }

  /*TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  #define T(a,b)    ( (TCoeff)( g_aiT2[ TRANSFORM_INVERSE ][ a ][ b ] ) * src[ a * line ] )

  for (int j = 0; j < line; j++, src++, dst += 2)
  {
  dst[0] = Clip3(outputMinimum, outputMaximum, (T(0, 0) + T(1, 0) + add) >> shift);
  dst[1] = Clip3(outputMinimum, outputMaximum, (T(0, 1) + T(1, 1) + add) >> shift);
  }

  #undef  T*/
}

/** 4x4 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
void fastForwardDCT2_B4(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use)
{
  Int j;
  TCoeff E[2], O[2];
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr4[DCT2][0] : g_aiT4[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const Int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* E and O */
    E[0] = src[0] + src[3];
    O[0] = src[0] - src[3];
    E[1] = src[1] + src[2];
    O[1] = src[1] - src[2];

    dst[0] = (iT[0] * E[0] + iT[1] * E[1] + add) >> shift;
    dst[2 * line] = (iT[8] * E[0] + iT[9] * E[1] + add) >> shift;
    dst[line] = (iT[4] * O[0] + iT[5] * O[1] + add) >> shift;
    dst[3 * line] = (iT[12] * O[0] + iT[13] * O[1] + add) >> shift;

    src += 4;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j<4; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

/** 4x4 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B4( const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  Int j;
  Int E[2], O[2];
  Int add = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr4[DCT2][0] : g_aiT4[TRANSFORM_INVERSE][0];

  const Int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    O[0] = iT[1 * 4 + 0] * src[line] + iT[3 * 4 + 0] * src[3 * line];
    O[1] = iT[1 * 4 + 1] * src[line] + iT[3 * 4 + 1] * src[3 * line];
    E[0] = iT[0 * 4 + 0] * src[   0] + iT[2 * 4 + 0] * src[2 * line];
    E[1] = iT[0 * 4 + 1] * src[   0] + iT[2 * 4 + 1] * src[2 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3( outputMinimum, outputMaximum, ( E[0] + O[0] + add ) >> shift );
    dst[1] = Clip3( outputMinimum, outputMaximum, ( E[1] + O[1] + add ) >> shift );
    dst[2] = Clip3( outputMinimum, outputMaximum, ( E[1] - O[1] + add ) >> shift );
    dst[3] = Clip3( outputMinimum, outputMaximum, ( E[0] - O[0] + add ) >> shift );

    src++;
    dst += 4;
  }
  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 2 ) * sizeof( TCoeff ) );
  }
}


template< Int uiTrSize >
inline void _fastInverseMM( const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
{
  const int  rnd_factor  = 1 << (shift - 1);
  const int  reducedLine = line - iSkipLine;
  const int  cutoff      = uiTrSize - iSkipLine2;

  for( int i = 0; i<reducedLine; i++ )
  {
    for( int j = 0; j<uiTrSize; j++ )
    {
      int iSum = 0;
      for( int k = 0; k<cutoff; k++)
      {
        iSum += src[k*line + i] * iT[k*uiTrSize + j];
      }
      dst[i*uiTrSize + j] = Clip3(outputMinimum, outputMaximum, (Int)(iSum + rnd_factor) >> shift);
    }
  }

  if (iSkipLine)
  {
    memset(dst + (reducedLine*uiTrSize), 0, (iSkipLine*uiTrSize) * sizeof(TCoeff));
  }
}


template< Int uiTrSize >
inline void _fastForwardMM( const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, const TMatrixCoeff* tc )
{
  const int  rnd_factor  = 1 << (shift - 1);
  const int  reducedLine = line - iSkipLine;
  const int  cutoff      = uiTrSize - iSkipLine2;
  TCoeff *pCoef;

  for( int i = 0; i<reducedLine; i++ )
  {
    pCoef = dst;
    const TMatrixCoeff* iT = tc;
    for( int j = 0; j<cutoff; j++ )
    {
      int iSum = 0;
      for( int k = 0; k<uiTrSize; k++ )
      {
        iSum += src[k] * iT[k];
      }
      pCoef[i] = (iSum + rnd_factor) >> shift;
      pCoef += line;
      iT += uiTrSize;
    }
    src += uiTrSize;
  }

  if( iSkipLine )
  {
    pCoef = dst + reducedLine;
    for( int j = 0; j<cutoff; j++ )
    {
      memset(pCoef, 0, sizeof(TCoeff) * iSkipLine);
      pCoef += line;
    }
  }

  if( iSkipLine2 )
  {
    pCoef = dst + line*cutoff;
    memset(pCoef, 0, sizeof(TCoeff) * line * iSkipLine2);
  }
}


#if x86_SSE_128

template<int HEIGHT>
inline void _fastForwardMM_B4_10bit_sse128(const int *src, int *dst, int shift, const int line, int iSkipLine, int iSkipLine2, const short* tc)
{
	__m128i src_128[HEIGHT], coef_128[4];
	__m128i tmpProduct, sum0, sum1, sum2, sum3, allSum, tmpDst;
	__m128i factor;

	factor = _mm_set1_epi32(1 << (shift - 1));

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < HEIGHT; ++i) {
		src_128[i] = _mm_loadu_si128((__m128i*)(src + i * 4));
	}

	coef_128[0] = _mm_set_epi32(tc[3], tc[2], tc[1], tc[0]);
	coef_128[1] = _mm_set_epi32(tc[7], tc[6], tc[5], tc[4]);
	coef_128[2] = _mm_set_epi32(tc[11], tc[10], tc[9], tc[8]);
	coef_128[3] = _mm_set_epi32(tc[15], tc[14], tc[13], tc[12]);

	int group_num = line / 4;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < group_num; ++j) {
			tmpProduct = _mm_mullo_epi32(src_128[4 * j + 0], coef_128[i]);
			sum0 = _mm_add_epi32(tmpProduct, _mm_srli_si128(tmpProduct, 8));
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

			tmpProduct = _mm_mullo_epi32(src_128[4 * j + 1], coef_128[i]);
			sum1 = _mm_add_epi32(tmpProduct, _mm_srli_si128(tmpProduct, 8));
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

			tmpProduct = _mm_mullo_epi32(src_128[4 * j + 2], coef_128[i]);
			sum2 = _mm_add_epi32(tmpProduct, _mm_srli_si128(tmpProduct, 8));
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

			tmpProduct = _mm_mullo_epi32(src_128[4 * j + 3], coef_128[i]);
			sum3 = _mm_add_epi32(tmpProduct, _mm_srli_si128(tmpProduct, 8));
			sum3 = _mm_add_epi64(sum3, _mm_srli_si128(sum3, 4));

			sum0 = _mm_unpacklo_epi32(sum0, sum1);
			sum2 = _mm_unpacklo_epi32(sum2, sum3);
			allSum = _mm_unpacklo_epi64(sum0, sum2);
			tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

			_mm_storeu_si128((__m128i*)&dst[i * line + j * 4], tmpDst);
		}
	}
}

template<int WIDTH>
inline void _fastInverseMM_B4_10bit_sse128(const int *src, int *dst, int shift, int line, int iSkipLine, int iSkipLine2, const int outputMinimum, const int outputMaximum, const short* iT) {
	const int m128iNUM = WIDTH * 4 / 4;
	const int WIDTH_4 = WIDTH / 4;
	__m128i src_128[m128iNUM], coef_128[4];
	__m128i tmpProduct, sum0, sum1, sum2, sum3, allSum, tmpDst;
	__m128i factor, min_val, max_val;
	__m128i tr[m128iNUM], tr_coef[4];

	factor = _mm_set1_epi32(1 << (shift - 1));
	min_val = _mm_set1_epi32(outputMinimum);
	max_val = _mm_set1_epi32(outputMaximum);

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < WIDTH; ++i) {
		int p_s = i * 4;
		src_128[i] = _mm_loadu_si128((__m128i*)(src + p_s));
	}
	for (int i = 0; i < 4; ++i) {
		int p_s = i * 4;
		coef_128[i] = _mm_set_epi32(iT[p_s + 3], iT[p_s + 2], iT[p_s + 1], iT[p_s + 0]);
	}

	// Transpose source matrix
#define TRANSPOSE_Nx4_32BIT(I) \
	for (int i = 0; i < WIDTH_4; ++i) { \
		tr[i * 4 + 0] = _mm_unpacklo_epi32(I[i + 0], I[i + WIDTH_4 + 0]); \
		tr[i * 4 + 1] = _mm_unpackhi_epi32(I[i + 0], I[i + WIDTH_4 + 0]); \
		tr[i * 4 + 2] = _mm_unpacklo_epi32(I[i + 2 * WIDTH_4 + 0], I[i + 3 * WIDTH_4 + 0]); \
		tr[i * 4 + 3] = _mm_unpackhi_epi32(I[i + 2 * WIDTH_4 + 0], I[i + 3 * WIDTH_4 + 0]); \
	} \
	for (int i = 0; i < WIDTH_4; ++i) { \
		I[i * 4 + 0] = _mm_unpacklo_epi64(tr[i * 4 + 0], tr[i * 4 + 2]); \
		I[i * 4 + 1] = _mm_unpackhi_epi64(tr[i * 4 + 0], tr[i * 4 + 2]); \
		I[i * 4 + 2] = _mm_unpacklo_epi64(tr[i * 4 + 1], tr[i * 4 + 3]); \
		I[i * 4 + 3] = _mm_unpackhi_epi64(tr[i * 4 + 1], tr[i * 4 + 3]); \
	}\

	TRANSPOSE_Nx4_32BIT(src_128);
#undef TRANSPOSE_Nx4_32BIT

	// Transpose coefficient matrix
#define TRANSPOSE_4x4_32BIT(I) \
	for (int i = 0; i < 1; ++i) { \
		tr_coef[i * 4 + 0] = _mm_unpacklo_epi32(I[i + 0], I[i + 1 + 0]); \
		tr_coef[i * 4 + 1] = _mm_unpackhi_epi32(I[i + 0], I[i + 1 + 0]); \
		tr_coef[i * 4 + 2] = _mm_unpacklo_epi32(I[i + 2 * 1 + 0], I[i + 3 * 1 + 0]); \
		tr_coef[i * 4 + 3] = _mm_unpackhi_epi32(I[i + 2 * 1 + 0], I[i + 3 * 1 + 0]); \
	} \
	for (int i = 0; i < 1; ++i) { \
		I[i * 4 + 0] = _mm_unpacklo_epi64(tr_coef[i * 4 + 0], tr_coef[i * 4 + 2]); \
		I[i * 4 + 1] = _mm_unpackhi_epi64(tr_coef[i * 4 + 0], tr_coef[i * 4 + 2]); \
		I[i * 4 + 2] = _mm_unpacklo_epi64(tr_coef[i * 4 + 1], tr_coef[i * 4 + 3]); \
		I[i * 4 + 3] = _mm_unpackhi_epi64(tr_coef[i * 4 + 1], tr_coef[i * 4 + 3]); \
	}\

	TRANSPOSE_4x4_32BIT(coef_128);
#undef TRANSPOSE_4x4_32BIT

	for (int i = 0; i < WIDTH; i++) {
		tmpProduct = _mm_mullo_epi32(coef_128[0], src_128[i]);
		sum0 = _mm_add_epi32(tmpProduct, _mm_srli_si128(tmpProduct, 8));
		sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

		tmpProduct = _mm_mullo_epi32(coef_128[1], src_128[i]);
		sum1 = _mm_add_epi32(tmpProduct, _mm_srli_si128(tmpProduct, 8));
		sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

		tmpProduct = _mm_mullo_epi32(coef_128[2], src_128[i]);
		sum2 = _mm_add_epi32(tmpProduct, _mm_srli_si128(tmpProduct, 8));
		sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

		tmpProduct = _mm_mullo_epi32(coef_128[3], src_128[i]);
		sum3 = _mm_add_epi32(tmpProduct, _mm_srli_si128(tmpProduct, 8));
		sum3 = _mm_add_epi64(sum3, _mm_srli_si128(sum3, 4));

		sum0 = _mm_unpacklo_epi32(sum0, sum1);
		sum2 = _mm_unpacklo_epi32(sum2, sum3);
		allSum = _mm_unpacklo_epi64(sum0, sum2);
		tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

		//clip
		tmpDst = _mm_min_epi32(tmpDst, max_val);
		tmpDst = _mm_max_epi32(tmpDst, min_val);

		_mm_storeu_si128((__m128i*)&dst[i * 4], tmpDst);
	}
}

template<int HEIGHT>
inline void _fastForwardMM_B8_10bit_sse128(const int *src, int *dst, int shift, int line, int iSkipLine, int iSkipLine2, const short* tc)
{
	__m128i src_128_0to3[HEIGHT], src_128_4to7[HEIGHT], coef_128_0to3[8], coef_128_4to7[8];
	__m128i sum_0to3, sum_4to7, sum0, sum1, sum2, sum3, allSum, tmpDst;
	__m128i factor;

	// shift
	factor = _mm_set1_epi32(1 << (shift - 1));

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < HEIGHT; ++i) {
		src_128_0to3[i] = _mm_loadu_si128((__m128i*)(src + i * 2 * 4));
		src_128_4to7[i] = _mm_loadu_si128((__m128i*)(src + (i * 2 + 1) * 4));
	}
	coef_128_0to3[0] = _mm_set_epi32(tc[3], tc[2], tc[1], tc[0]);
	coef_128_4to7[0] = _mm_set_epi32(tc[7], tc[6], tc[5], tc[4]);
	coef_128_0to3[1] = _mm_set_epi32(tc[11], tc[10], tc[9], tc[8]);
	coef_128_4to7[1] = _mm_set_epi32(tc[15], tc[14], tc[13], tc[12]);
	coef_128_0to3[2] = _mm_set_epi32(tc[19], tc[18], tc[17], tc[16]);
	coef_128_4to7[2] = _mm_set_epi32(tc[23], tc[22], tc[21], tc[20]);
	coef_128_0to3[3] = _mm_set_epi32(tc[27], tc[26], tc[25], tc[24]);
	coef_128_4to7[3] = _mm_set_epi32(tc[31], tc[30], tc[29], tc[28]);
	coef_128_0to3[4] = _mm_set_epi32(tc[35], tc[34], tc[33], tc[32]);
	coef_128_4to7[4] = _mm_set_epi32(tc[39], tc[38], tc[37], tc[36]);
	coef_128_0to3[5] = _mm_set_epi32(tc[43], tc[42], tc[41], tc[40]);
	coef_128_4to7[5] = _mm_set_epi32(tc[47], tc[46], tc[45], tc[44]);
	coef_128_0to3[6] = _mm_set_epi32(tc[51], tc[50], tc[49], tc[48]);
	coef_128_4to7[6] = _mm_set_epi32(tc[55], tc[54], tc[53], tc[52]);
	coef_128_0to3[7] = _mm_set_epi32(tc[59], tc[58], tc[57], tc[56]);
	coef_128_4to7[7] = _mm_set_epi32(tc[63], tc[62], tc[61], tc[60]);

	int group_num = line / 4;
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < group_num; j++) {
			// Four results
			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 0], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 0], coef_128_4to7[i]);
			sum0 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 8));
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 1], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 1], coef_128_4to7[i]);
			sum1 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 8));
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 2], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 2], coef_128_4to7[i]);
			sum2 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 8));
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 3], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 3], coef_128_4to7[i]);
			sum3 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 8));
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 4));

			sum2 = _mm_unpacklo_epi32(sum2, sum3);
			sum0 = _mm_unpacklo_epi32(sum0, sum1);
			allSum = _mm_unpacklo_epi64(sum0, sum2);
			tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

			_mm_storeu_si128((__m128i*)&dst[i * line + j * 4], tmpDst);
		}
	}
}

template<int WIDTH>
inline void _fastInverseMM_B8_10bit_sse128(const int *src, int *dst, int shift, int line, int iSkipLine, int iSkipLine2, const int outputMinimum, const int outputMaximum, const short* iT) {
	const int m128iNUM = WIDTH * 8 / 4;
	const int WIDTH_4 = WIDTH / 4;
	__m128i src_128[m128iNUM], coef_128[16];
	__m128i sum_0to3, sum_4to7, sum0, sum1, sum2, sum3, allSum, tmpDst;
	__m128i factor, min_val, max_val;
	__m128i tr[m128iNUM], tr_coef[16];

	// shift
	factor = _mm_set1_epi32(1 << (shift - 1));
	min_val = _mm_set1_epi32(outputMinimum);
	max_val = _mm_set1_epi32(outputMaximum);

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < m128iNUM; ++i) {
		int p_s = i * 4;
		src_128[i] = _mm_loadu_si128((__m128i*)(src + p_s));
	}
	for (int i = 0; i < 16; ++i) {
		int p_s = i * 4;
		coef_128[i] = _mm_set_epi32(iT[p_s + 3], iT[p_s + 2], iT[p_s + 1], iT[p_s + 0]);
	}

	// Transpose source matrix
#define TRANSPOSE_Nx8_32BIT(I) \
	for (int i = 0; i < m128iNUM / 8; ++i) { \
		const int ix8 = i * 8;\
		tr[ix8 + 0] = _mm_unpacklo_epi32(I[i + 0], I[i + WIDTH_4 + 0]); \
		tr[ix8 + 1] = _mm_unpackhi_epi32(I[i + 0], I[i + WIDTH_4 + 0]); \
		tr[ix8 + 2] = _mm_unpacklo_epi32(I[i + 2 * WIDTH_4 + 0], I[i + 3 * WIDTH_4 + 0]); \
		tr[ix8 + 3] = _mm_unpackhi_epi32(I[i + 2 * WIDTH_4 + 0], I[i + 3 * WIDTH_4 + 0]); \
		tr[ix8 + 4] = _mm_unpacklo_epi32(I[i + 4 * WIDTH_4 + 0], I[i + 5 * WIDTH_4 + 0]); \
		tr[ix8 + 5] = _mm_unpackhi_epi32(I[i + 4 * WIDTH_4 + 0], I[i + 5 * WIDTH_4 + 0]); \
		tr[ix8 + 6] = _mm_unpacklo_epi32(I[i + 6 * WIDTH_4 + 0], I[i + 7 * WIDTH_4 + 0]); \
		tr[ix8 + 7] = _mm_unpackhi_epi32(I[i + 6 * WIDTH_4 + 0], I[i + 7 * WIDTH_4 + 0]); \
	} \
	for (int i = 0; i < m128iNUM / 8; ++i) { \
		const int ix8 = i * 8;\
		I[ix8 + 0] = _mm_unpacklo_epi64(tr[ix8 + 0], tr[ix8 + 2]); \
		I[ix8 + 1] = _mm_unpacklo_epi64(tr[ix8 + 4], tr[ix8 + 6]); \
		I[ix8 + 2] = _mm_unpackhi_epi64(tr[ix8 + 0], tr[ix8 + 2]); \
		I[ix8 + 3] = _mm_unpackhi_epi64(tr[ix8 + 4], tr[ix8 + 6]); \
		I[ix8 + 4] = _mm_unpacklo_epi64(tr[ix8 + 1], tr[ix8 + 3]); \
		I[ix8 + 5] = _mm_unpacklo_epi64(tr[ix8 + 5], tr[ix8 + 7]); \
		I[ix8 + 6] = _mm_unpackhi_epi64(tr[ix8 + 1], tr[ix8 + 3]); \
		I[ix8 + 7] = _mm_unpackhi_epi64(tr[ix8 + 5], tr[ix8 + 7]); \
	}\

	TRANSPOSE_Nx8_32BIT(src_128);
#undef TRANSPOSE_Nx8_32BIT

	// Transpose coefficient matrix
#define TRANSPOSE_8x8_32BIT(I) \
	for (int i = 0; i < 2; ++i) { \
		const int ix8 = i * 8; \
		tr_coef[ix8 + 0] = _mm_unpacklo_epi32(I[i + 0], I[i + 2 + 0]); \
		tr_coef[ix8 + 1] = _mm_unpackhi_epi32(I[i + 0], I[i + 2 + 0]); \
		tr_coef[ix8 + 2] = _mm_unpacklo_epi32(I[i + 2 * 2 + 0], I[i + 3 * 2 + 0]); \
		tr_coef[ix8 + 3] = _mm_unpackhi_epi32(I[i + 2 * 2 + 0], I[i + 3 * 2 + 0]); \
		tr_coef[ix8 + 4] = _mm_unpacklo_epi32(I[i + 4 * 2 + 0], I[i + 5 * 2 + 0]); \
		tr_coef[ix8 + 5] = _mm_unpackhi_epi32(I[i + 4 * 2 + 0], I[i + 5 * 2 + 0]); \
		tr_coef[ix8 + 6] = _mm_unpacklo_epi32(I[i + 6 * 2 + 0], I[i + 7 * 2 + 0]); \
		tr_coef[ix8 + 7] = _mm_unpackhi_epi32(I[i + 6 * 2 + 0], I[i + 7 * 2 + 0]); \
	} \
	for (int i = 0; i < 2; ++i) { \
		const int ix8 = i * 8;\
		I[ix8 + 0] = _mm_unpacklo_epi64(tr_coef[ix8 + 0], tr_coef[ix8 + 2]); \
		I[ix8 + 1] = _mm_unpacklo_epi64(tr_coef[ix8 + 4], tr_coef[ix8 + 6]); \
		I[ix8 + 2] = _mm_unpackhi_epi64(tr_coef[ix8 + 0], tr_coef[ix8 + 2]); \
		I[ix8 + 3] = _mm_unpackhi_epi64(tr_coef[ix8 + 4], tr_coef[ix8 + 6]); \
		I[ix8 + 4] = _mm_unpacklo_epi64(tr_coef[ix8 + 1], tr_coef[ix8 + 3]); \
		I[ix8 + 5] = _mm_unpacklo_epi64(tr_coef[ix8 + 5], tr_coef[ix8 + 7]); \
		I[ix8 + 6] = _mm_unpackhi_epi64(tr_coef[ix8 + 1], tr_coef[ix8 + 3]); \
		I[ix8 + 7] = _mm_unpackhi_epi64(tr_coef[ix8 + 5], tr_coef[ix8 + 7]); \
	}\

	TRANSPOSE_8x8_32BIT(coef_128);
#undef TRANSPOSE_8x8_32BIT

	for (int i = 0; i < WIDTH; i++) {
		for (int j = 0; j < 2; j++) {
			// Four results
			sum_0to3 = _mm_mullo_epi32(coef_128[j * 8 + 0], src_128[i * 2]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 8 + 1], src_128[i * 2 + 1]);
			sum0 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 8));
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

			sum_0to3 = _mm_mullo_epi32(coef_128[j * 8 + 2], src_128[i * 2]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 8 + 3], src_128[i * 2 + 1]);
			sum1 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 8));
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

			sum_0to3 = _mm_mullo_epi32(coef_128[j * 8 + 4], src_128[i * 2]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 8 + 5], src_128[i * 2 + 1]);
			sum2 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 8));
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

			sum_0to3 = _mm_mullo_epi32(coef_128[j * 8 + 6], src_128[i * 2]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 8 + 7], src_128[i * 2 + 1]);
			sum3 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 8));
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 4));

			sum2 = _mm_unpacklo_epi32(sum2, sum3);
			sum0 = _mm_unpacklo_epi32(sum0, sum1);
			allSum = _mm_unpacklo_epi64(sum0, sum2);
			tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

			//clip
			tmpDst = _mm_min_epi32(tmpDst, max_val);
			tmpDst = _mm_max_epi32(tmpDst, min_val);

			_mm_storeu_si128((__m128i*)&dst[i * 8 + j * 4], tmpDst);
		}
	}
}

template<int HEIGHT>
inline void _fastForwardMM_B16_10bit_sse128(const int *src, int *dst, int shift, int line, int iSkipLine, int iSkipLine2, const short* tc)
{
	__m128i src_128_0to3[HEIGHT], src_128_4to7[HEIGHT], src_128_8to11[HEIGHT], src_128_12to15[HEIGHT], coef_128_0to3[16],
		coef_128_4to7[16], coef_128_8to11[16], coef_128_12to15[16];
	__m128i allSum, tmpDst, sum_0to3, sum_4to7, sum_8to11, sum_12to15, sum0, sum1, sum2, sum3;
	__m128i factor;

	// shift
	factor = _mm_set1_epi32(1 << (shift - 1));

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < HEIGHT; ++i) {
		int p_s = i * 16;
		src_128_0to3[i] = _mm_loadu_si128((__m128i*)(src + p_s + 0));
		src_128_4to7[i] = _mm_loadu_si128((__m128i*)(src + p_s + 4));
		src_128_8to11[i] = _mm_loadu_si128((__m128i*)(src + p_s + 8));
		src_128_12to15[i] = _mm_loadu_si128((__m128i*)(src + p_s + 12));
	}
	for (int i = 0; i < 16; ++i) {
		int p_s = i * 16;
		coef_128_0to3[i] = _mm_set_epi32(tc[p_s + 3], tc[p_s + 2], tc[p_s + 1], tc[p_s + 0]);
		coef_128_4to7[i] = _mm_set_epi32(tc[p_s + 7], tc[p_s + 6], tc[p_s + 5], tc[p_s + 4]);
		coef_128_8to11[i] = _mm_set_epi32(tc[p_s + 11], tc[p_s + 10], tc[p_s + 9], tc[p_s + 8]);
		coef_128_12to15[i] = _mm_set_epi32(tc[p_s + 15], tc[p_s + 14], tc[p_s + 13], tc[p_s + 12]);
	}

	int group_num = line / 4;
	for (int i = 0; i < 16; i++) {
		for (int j = 0; j < group_num; j++) {
			// Four results
			// 0
			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 0], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 0], coef_128_4to7[i]);
			sum_8to11 = _mm_mullo_epi32(src_128_8to11[j * 4 + 0], coef_128_8to11[i]);
			sum_12to15 = _mm_mullo_epi32(src_128_12to15[j * 4 + 0], coef_128_12to15[i]);

			sum0 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum0 = _mm_add_epi32(sum0, sum_8to11);
			sum0 = _mm_add_epi32(sum0, sum_12to15);
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 8));
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

			// 1
			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 1], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 1], coef_128_4to7[i]);
			sum_8to11 = _mm_mullo_epi32(src_128_8to11[j * 4 + 1], coef_128_8to11[i]);
			sum_12to15 = _mm_mullo_epi32(src_128_12to15[j * 4 + 1], coef_128_12to15[i]);

			sum1 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum1 = _mm_add_epi32(sum1, sum_8to11);
			sum1 = _mm_add_epi32(sum1, sum_12to15);
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 8));
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

			// 2
			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 2], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 2], coef_128_4to7[i]);
			sum_8to11 = _mm_mullo_epi32(src_128_8to11[j * 4 + 2], coef_128_8to11[i]);
			sum_12to15 = _mm_mullo_epi32(src_128_12to15[j * 4 + 2], coef_128_12to15[i]);

			sum2 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum2 = _mm_add_epi32(sum2, sum_8to11);
			sum2 = _mm_add_epi32(sum2, sum_12to15);
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 8));
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

			// 3
			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 3], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 3], coef_128_4to7[i]);
			sum_8to11 = _mm_mullo_epi32(src_128_8to11[j * 4 + 3], coef_128_8to11[i]);
			sum_12to15 = _mm_mullo_epi32(src_128_12to15[j * 4 + 3], coef_128_12to15[i]);

			sum3 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum3 = _mm_add_epi32(sum3, sum_8to11);
			sum3 = _mm_add_epi32(sum3, sum_12to15);
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 8));
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 4));

			// Sum mul of result 0 1 2 3
			sum0 = _mm_unpacklo_epi32(sum0, sum1);
			sum2 = _mm_unpacklo_epi32(sum2, sum3);
			allSum = _mm_unpacklo_epi64(sum0, sum2);
			tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

			_mm_storeu_si128((__m128i*)&dst[i * line + j * 4], tmpDst);
		}
	}
}

template<int WIDTH>
inline void _fastInverseMM_B16_10bit_sse128(const int *src, int *dst, int shift, int line, int iSkipLine, int iSkipLine2, const int outputMinimum, const int outputMaximum, const short* iT) {
	const int m128iNUM = WIDTH * 16 / 4;
	const int WIDTH_4 = WIDTH / 4;
	__m128i src_128[m128iNUM], coef_128[64];
	__m128i allSum, tmpDst, sum_0to3, sum_4to7, sum_8to11, sum_12to15, sum0, sum1, sum2, sum3;
	__m128i factor, min_val, max_val;
	__m128i tr[m128iNUM], tr_coef[64];

	// shift
	factor = _mm_set1_epi32(1 << (shift - 1));
	min_val = _mm_set1_epi32(outputMinimum);
	max_val = _mm_set1_epi32(outputMaximum);

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < m128iNUM; ++i) {
		int p_s = i * 4;
		src_128[i] = _mm_loadu_si128((__m128i*)(src + p_s));
	}
	for (int i = 0; i < 64; ++i) {
		int p_s = i * 4;
		coef_128[i] = _mm_set_epi32(iT[p_s + 3], iT[p_s + 2], iT[p_s + 1], iT[p_s + 0]);
	}
	int a = 0;
	// Transpose source matrix
#define TRANSPOSE_Nx16_32BIT(I) \
	for (int i = 0; i < m128iNUM / 16; ++i) { \
		const int ix16 = i * 16;\
		tr[ix16 + 0] = _mm_unpacklo_epi32(I[i + 0], I[i + WIDTH_4]); \
		tr[ix16 + 1] = _mm_unpackhi_epi32(I[i + 0], I[i + WIDTH_4]); \
		tr[ix16 + 2] = _mm_unpacklo_epi32(I[i + 2 * WIDTH_4], I[i + 3 * WIDTH_4]); \
		tr[ix16 + 3] = _mm_unpackhi_epi32(I[i + 2 * WIDTH_4], I[i + 3 * WIDTH_4]); \
		tr[ix16 + 4] = _mm_unpacklo_epi32(I[i + 4 * WIDTH_4], I[i + 5 * WIDTH_4]); \
		tr[ix16 + 5] = _mm_unpackhi_epi32(I[i + 4 * WIDTH_4], I[i + 5 * WIDTH_4]); \
		tr[ix16 + 6] = _mm_unpacklo_epi32(I[i + 6 * WIDTH_4], I[i + 7 * WIDTH_4]); \
		tr[ix16 + 7] = _mm_unpackhi_epi32(I[i + 6 * WIDTH_4], I[i + 7 * WIDTH_4]); \
		tr[ix16 + 8] = _mm_unpacklo_epi32(I[i + 8 * WIDTH_4], I[i + 9 * WIDTH_4]); \
		tr[ix16 + 9] = _mm_unpackhi_epi32(I[i + 8 * WIDTH_4], I[i + 9 * WIDTH_4]); \
		tr[ix16 + 10] = _mm_unpacklo_epi32(I[i + 10 * WIDTH_4], I[i + 11 * WIDTH_4]); \
		tr[ix16 + 11] = _mm_unpackhi_epi32(I[i + 10 * WIDTH_4], I[i + 11 * WIDTH_4]); \
		tr[ix16 + 12] = _mm_unpacklo_epi32(I[i + 12 * WIDTH_4], I[i + 13 * WIDTH_4]); \
		tr[ix16 + 13] = _mm_unpackhi_epi32(I[i + 12 * WIDTH_4], I[i + 13 * WIDTH_4]); \
		tr[ix16 + 14] = _mm_unpacklo_epi32(I[i + 14 * WIDTH_4], I[i + 15 * WIDTH_4]); \
		tr[ix16 + 15] = _mm_unpackhi_epi32(I[i + 14 * WIDTH_4], I[i + 15 * WIDTH_4]); \
	} \
	for (int i = 0; i < m128iNUM / 16; ++i) { \
		const int ix16 = i * 16;\
		I[ix16 + 0] = _mm_unpacklo_epi64(tr[ix16 + 0], tr[ix16 + 2]); \
		I[ix16 + 1] = _mm_unpacklo_epi64(tr[ix16 + 4], tr[ix16 + 6]); \
		I[ix16 + 2] = _mm_unpacklo_epi64(tr[ix16 + 8], tr[ix16 + 10]); \
		I[ix16 + 3] = _mm_unpacklo_epi64(tr[ix16 + 12], tr[ix16 + 14]); \
		I[ix16 + 4] = _mm_unpackhi_epi64(tr[ix16 + 0], tr[ix16 + 2]); \
		I[ix16 + 5] = _mm_unpackhi_epi64(tr[ix16 + 4], tr[ix16 + 6]); \
		I[ix16 + 6] = _mm_unpackhi_epi64(tr[ix16 + 8], tr[ix16 + 10]); \
		I[ix16 + 7] = _mm_unpackhi_epi64(tr[ix16 + 12], tr[ix16 + 14]); \
		I[ix16 + 8] = _mm_unpacklo_epi64(tr[ix16 + 1], tr[ix16 + 3]); \
		I[ix16 + 9] = _mm_unpacklo_epi64(tr[ix16 + 5], tr[ix16 + 7]); \
		I[ix16 + 10] = _mm_unpacklo_epi64(tr[ix16 + 9], tr[ix16 + 11]); \
		I[ix16 + 11] = _mm_unpacklo_epi64(tr[ix16 + 13], tr[ix16 + 15]); \
		I[ix16 + 12] = _mm_unpackhi_epi64(tr[ix16 + 1], tr[ix16 + 3]); \
		I[ix16 + 13] = _mm_unpackhi_epi64(tr[ix16 + 5], tr[ix16 + 7]); \
		I[ix16 + 14] = _mm_unpackhi_epi64(tr[ix16 + 9], tr[ix16 + 11]); \
		I[ix16 + 15] = _mm_unpackhi_epi64(tr[ix16 + 13], tr[ix16 + 15]); \
	}\

	TRANSPOSE_Nx16_32BIT(src_128);
#undef TRANSPOSE_Nx16_32BIT

	// Transpose coefficient matrix
#define TRANSPOSE_16x16_32BIT(I) \
	for (int i = 0; i < 4; ++i) { \
		const int ix8 = i * 16;\
		tr_coef[ix8 + 0] = _mm_unpacklo_epi32(I[i + 0], I[i + 4]); \
		tr_coef[ix8 + 1] = _mm_unpackhi_epi32(I[i + 0], I[i + 4]); \
		tr_coef[ix8 + 2] = _mm_unpacklo_epi32(I[i + 2 * 4], I[i + 3 * 4]); \
		tr_coef[ix8 + 3] = _mm_unpackhi_epi32(I[i + 2 * 4], I[i + 3 * 4]); \
		tr_coef[ix8 + 4] = _mm_unpacklo_epi32(I[i + 4 * 4], I[i + 5 * 4]); \
		tr_coef[ix8 + 5] = _mm_unpackhi_epi32(I[i + 4 * 4], I[i + 5 * 4]); \
		tr_coef[ix8 + 6] = _mm_unpacklo_epi32(I[i + 6 * 4], I[i + 7 * 4]); \
		tr_coef[ix8 + 7] = _mm_unpackhi_epi32(I[i + 6 * 4], I[i + 7 * 4]); \
		tr_coef[ix8 + 8] = _mm_unpacklo_epi32(I[i + 8 * 4], I[i + 9 * 4]); \
		tr_coef[ix8 + 9] = _mm_unpackhi_epi32(I[i + 8 * 4], I[i + 9 * 4]); \
		tr_coef[ix8 + 10] = _mm_unpacklo_epi32(I[i + 10 * 4], I[i + 11 * 4]); \
		tr_coef[ix8 + 11] = _mm_unpackhi_epi32(I[i + 10 * 4], I[i + 11 * 4]); \
		tr_coef[ix8 + 12] = _mm_unpacklo_epi32(I[i + 12 * 4], I[i + 13 * 4]); \
		tr_coef[ix8 + 13] = _mm_unpackhi_epi32(I[i + 12 * 4], I[i + 13 * 4]); \
		tr_coef[ix8 + 14] = _mm_unpacklo_epi32(I[i + 14 * 4], I[i + 15 * 4]); \
		tr_coef[ix8 + 15] = _mm_unpackhi_epi32(I[i + 14 * 4], I[i + 15 * 4]); \
	} \
	for (int i = 0; i < 4; ++i) { \
		const int ix8 = i * 16;\
		I[ix8 + 0] = _mm_unpacklo_epi64(tr_coef[ix8 + 0], tr_coef[ix8 + 2]); \
		I[ix8 + 1] = _mm_unpacklo_epi64(tr_coef[ix8 + 4], tr_coef[ix8 + 6]); \
		I[ix8 + 2] = _mm_unpacklo_epi64(tr_coef[ix8 + 8], tr_coef[ix8 + 10]); \
		I[ix8 + 3] = _mm_unpacklo_epi64(tr_coef[ix8 + 12], tr_coef[ix8 + 14]); \
		I[ix8 + 4] = _mm_unpackhi_epi64(tr_coef[ix8 + 0], tr_coef[ix8 + 2]); \
		I[ix8 + 5] = _mm_unpackhi_epi64(tr_coef[ix8 + 4], tr_coef[ix8 + 6]); \
		I[ix8 + 6] = _mm_unpackhi_epi64(tr_coef[ix8 + 8], tr_coef[ix8 + 10]); \
		I[ix8 + 7] = _mm_unpackhi_epi64(tr_coef[ix8 + 12], tr_coef[ix8 + 14]); \
		I[ix8 + 8] = _mm_unpacklo_epi64(tr_coef[ix8 + 1], tr_coef[ix8 + 3]); \
		I[ix8 + 9] = _mm_unpacklo_epi64(tr_coef[ix8 + 5], tr_coef[ix8 + 7]); \
		I[ix8 + 10] = _mm_unpacklo_epi64(tr_coef[ix8 + 9], tr_coef[ix8 + 11]); \
		I[ix8 + 11] = _mm_unpacklo_epi64(tr_coef[ix8 + 13], tr_coef[ix8 + 15]); \
		I[ix8 + 12] = _mm_unpackhi_epi64(tr_coef[ix8 + 1], tr_coef[ix8 + 3]); \
		I[ix8 + 13] = _mm_unpackhi_epi64(tr_coef[ix8 + 5], tr_coef[ix8 + 7]); \
		I[ix8 + 14] = _mm_unpackhi_epi64(tr_coef[ix8 + 9], tr_coef[ix8 + 11]); \
		I[ix8 + 15] = _mm_unpackhi_epi64(tr_coef[ix8 + 13], tr_coef[ix8 + 15]); \
	}\

	TRANSPOSE_16x16_32BIT(coef_128);
#undef TRANSPOSE_16x16_32BIT

	for (int i = 0; i < WIDTH; i++) {
		for (int j = 0; j < 4; j++) {
			// Four results
			// 0
			sum_0to3 = _mm_mullo_epi32(coef_128[j * 16 + 0], src_128[i * 4]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 16 + 1], src_128[i * 4 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[j * 16 + 2], src_128[i * 4 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[j * 16 + 3], src_128[i * 4 + 3]);

			sum0 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum0 = _mm_add_epi32(sum0, sum_8to11);
			sum0 = _mm_add_epi32(sum0, sum_12to15);
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 8));
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

			// 1
			sum_0to3 = _mm_mullo_epi32(coef_128[j * 16 + 4], src_128[i * 4]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 16 + 5], src_128[i * 4 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[j * 16 + 6], src_128[i * 4 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[j * 16 + 7], src_128[i * 4 + 3]);

			sum1 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum1 = _mm_add_epi32(sum1, sum_8to11);
			sum1 = _mm_add_epi32(sum1, sum_12to15);
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 8));
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

			// 2
			sum_0to3 = _mm_mullo_epi32(coef_128[j * 16 + 8], src_128[i * 4]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 16 + 9], src_128[i * 4 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[j * 16 + 10], src_128[i * 4 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[j * 16 + 11], src_128[i * 4 + 3]);

			sum2 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum2 = _mm_add_epi32(sum2, sum_8to11);
			sum2 = _mm_add_epi32(sum2, sum_12to15);
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 8));
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

			// 3
			sum_0to3 = _mm_mullo_epi32(coef_128[j * 16 + 12], src_128[i * 4]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 16 + 13], src_128[i * 4 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[j * 16 + 14], src_128[i * 4 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[j * 16 + 15], src_128[i * 4 + 3]);

			sum3 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum3 = _mm_add_epi32(sum3, sum_8to11);
			sum3 = _mm_add_epi32(sum3, sum_12to15);
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 8));
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 4));

			// Sum mul of result 0 1 2 3
			sum0 = _mm_unpacklo_epi32(sum0, sum1);
			sum2 = _mm_unpacklo_epi32(sum2, sum3);
			allSum = _mm_unpacklo_epi64(sum0, sum2);
			tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

			//clip
			tmpDst = _mm_min_epi32(tmpDst, max_val);
			tmpDst = _mm_max_epi32(tmpDst, min_val);

			_mm_storeu_si128((__m128i*)&dst[i * 16 + j * 4], tmpDst);
		}
	}
}

template<int HEIGHT>
inline void _fastForwardMM_B32_10bit_sse128(const int *src, int *dst, int shift, int line, int iSkipLine, int iSkipLine2, const short* tc)
{
	__m128i src_128_0to3[HEIGHT], src_128_4to7[HEIGHT], src_128_8to11[HEIGHT], src_128_12to15[HEIGHT], src_128_16to19[HEIGHT],
		src_128_20to23[HEIGHT], src_128_24to27[HEIGHT], src_128_28to31[HEIGHT], coef_128_0to3[32], coef_128_4to7[32],
		coef_128_8to11[32], coef_128_12to15[32], coef_128_16to19[32], coef_128_20to23[32], coef_128_24to27[32], coef_128_28to31[32];
	__m128i allSum, tmpDst, sum_0to3, sum_4to7, sum_8to11, sum_12to15, sum_16to19, sum_20to23, sum_24to27, sum_28to31,
		sum0, sum1, sum2, sum3;
	__m128i factor;

	// shift
	factor = _mm_set1_epi32(1 << (shift - 1));

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < HEIGHT; ++i) {
		int p_s = i * 32;
		src_128_0to3[i] = _mm_loadu_si128((__m128i*)(src + p_s + 0));
		src_128_4to7[i] = _mm_loadu_si128((__m128i*)(src + p_s + 4));
		src_128_8to11[i] = _mm_loadu_si128((__m128i*)(src + p_s + 8));
		src_128_12to15[i] = _mm_loadu_si128((__m128i*)(src + p_s + 12));
		src_128_16to19[i] = _mm_loadu_si128((__m128i*)(src + p_s + 16));
		src_128_20to23[i] = _mm_loadu_si128((__m128i*)(src + p_s + 20));
		src_128_24to27[i] = _mm_loadu_si128((__m128i*)(src + p_s + 24));
		src_128_28to31[i] = _mm_loadu_si128((__m128i*)(src + p_s + 28));
	}
	for (int i = 0; i < 32; ++i) {
		int p_s = i * 32;
		coef_128_0to3[i] = _mm_set_epi32(tc[p_s + 3], tc[p_s + 2], tc[p_s + 1], tc[p_s + 0]);
		coef_128_4to7[i] = _mm_set_epi32(tc[p_s + 7], tc[p_s + 6], tc[p_s + 5], tc[p_s + 4]);
		coef_128_8to11[i] = _mm_set_epi32(tc[p_s + 11], tc[p_s + 10], tc[p_s + 9], tc[p_s + 8]);
		coef_128_12to15[i] = _mm_set_epi32(tc[p_s + 15], tc[p_s + 14], tc[p_s + 13], tc[p_s + 12]);
		coef_128_16to19[i] = _mm_set_epi32(tc[p_s + 19], tc[p_s + 18], tc[p_s + 17], tc[p_s + 16]);
		coef_128_20to23[i] = _mm_set_epi32(tc[p_s + 23], tc[p_s + 22], tc[p_s + 21], tc[p_s + 20]);
		coef_128_24to27[i] = _mm_set_epi32(tc[p_s + 27], tc[p_s + 26], tc[p_s + 25], tc[p_s + 24]);
		coef_128_28to31[i] = _mm_set_epi32(tc[p_s + 31], tc[p_s + 30], tc[p_s + 29], tc[p_s + 28]);
	}

	int group_num = line / 4;
	for (int i = 0; i < 32; i++) {
		for (int j = 0; j < group_num; j++) {
			// Four results
			// 0
			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 0], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 0], coef_128_4to7[i]);
			sum_8to11 = _mm_mullo_epi32(src_128_8to11[j * 4 + 0], coef_128_8to11[i]);
			sum_12to15 = _mm_mullo_epi32(src_128_12to15[j * 4 + 0], coef_128_12to15[i]);
			sum_16to19 = _mm_mullo_epi32(src_128_16to19[j * 4 + 0], coef_128_16to19[i]);
			sum_20to23 = _mm_mullo_epi32(src_128_20to23[j * 4 + 0], coef_128_20to23[i]);
			sum_24to27 = _mm_mullo_epi32(src_128_24to27[j * 4 + 0], coef_128_24to27[i]);
			sum_28to31 = _mm_mullo_epi32(src_128_28to31[j * 4 + 0], coef_128_28to31[i]);

			sum0 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum0 = _mm_add_epi32(sum0, sum_8to11);
			sum0 = _mm_add_epi32(sum0, sum_12to15);
			sum0 = _mm_add_epi32(sum0, sum_16to19);
			sum0 = _mm_add_epi32(sum0, sum_20to23);
			sum0 = _mm_add_epi32(sum0, sum_24to27);
			sum0 = _mm_add_epi32(sum0, sum_28to31);
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 8));
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

			// 1
			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 1], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 1], coef_128_4to7[i]);
			sum_8to11 = _mm_mullo_epi32(src_128_8to11[j * 4 + 1], coef_128_8to11[i]);
			sum_12to15 = _mm_mullo_epi32(src_128_12to15[j * 4 + 1], coef_128_12to15[i]);
			sum_16to19 = _mm_mullo_epi32(src_128_16to19[j * 4 + 1], coef_128_16to19[i]);
			sum_20to23 = _mm_mullo_epi32(src_128_20to23[j * 4 + 1], coef_128_20to23[i]);
			sum_24to27 = _mm_mullo_epi32(src_128_24to27[j * 4 + 1], coef_128_24to27[i]);
			sum_28to31 = _mm_mullo_epi32(src_128_28to31[j * 4 + 1], coef_128_28to31[i]);

			sum1 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum1 = _mm_add_epi32(sum1, sum_8to11);
			sum1 = _mm_add_epi32(sum1, sum_12to15);
			sum1 = _mm_add_epi32(sum1, sum_16to19);
			sum1 = _mm_add_epi32(sum1, sum_20to23);
			sum1 = _mm_add_epi32(sum1, sum_24to27);
			sum1 = _mm_add_epi32(sum1, sum_28to31);
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 8));
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

			// 2
			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 2], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 2], coef_128_4to7[i]);
			sum_8to11 = _mm_mullo_epi32(src_128_8to11[j * 4 + 2], coef_128_8to11[i]);
			sum_12to15 = _mm_mullo_epi32(src_128_12to15[j * 4 + 2], coef_128_12to15[i]);
			sum_16to19 = _mm_mullo_epi32(src_128_16to19[j * 4 + 2], coef_128_16to19[i]);
			sum_20to23 = _mm_mullo_epi32(src_128_20to23[j * 4 + 2], coef_128_20to23[i]);
			sum_24to27 = _mm_mullo_epi32(src_128_24to27[j * 4 + 2], coef_128_24to27[i]);
			sum_28to31 = _mm_mullo_epi32(src_128_28to31[j * 4 + 2], coef_128_28to31[i]);

			sum2 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum2 = _mm_add_epi32(sum2, sum_8to11);
			sum2 = _mm_add_epi32(sum2, sum_12to15);
			sum2 = _mm_add_epi32(sum2, sum_16to19);
			sum2 = _mm_add_epi32(sum2, sum_20to23);
			sum2 = _mm_add_epi32(sum2, sum_24to27);
			sum2 = _mm_add_epi32(sum2, sum_28to31);
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 8));
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

			// 3
			sum_0to3 = _mm_mullo_epi32(src_128_0to3[j * 4 + 3], coef_128_0to3[i]);
			sum_4to7 = _mm_mullo_epi32(src_128_4to7[j * 4 + 3], coef_128_4to7[i]);
			sum_8to11 = _mm_mullo_epi32(src_128_8to11[j * 4 + 3], coef_128_8to11[i]);
			sum_12to15 = _mm_mullo_epi32(src_128_12to15[j * 4 + 3], coef_128_12to15[i]);
			sum_16to19 = _mm_mullo_epi32(src_128_16to19[j * 4 + 3], coef_128_16to19[i]);
			sum_20to23 = _mm_mullo_epi32(src_128_20to23[j * 4 + 3], coef_128_20to23[i]);
			sum_24to27 = _mm_mullo_epi32(src_128_24to27[j * 4 + 3], coef_128_24to27[i]);
			sum_28to31 = _mm_mullo_epi32(src_128_28to31[j * 4 + 3], coef_128_28to31[i]);

			sum3 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum3 = _mm_add_epi32(sum3, sum_8to11);
			sum3 = _mm_add_epi32(sum3, sum_12to15);
			sum3 = _mm_add_epi32(sum3, sum_16to19);
			sum3 = _mm_add_epi32(sum3, sum_20to23);
			sum3 = _mm_add_epi32(sum3, sum_24to27);
			sum3 = _mm_add_epi32(sum3, sum_28to31);
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 8));
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 4));

			// Sum mul of result 0 1 2 3
			sum0 = _mm_unpacklo_epi32(sum0, sum1);
			sum2 = _mm_unpacklo_epi32(sum2, sum3);
			allSum = _mm_unpacklo_epi64(sum0, sum2);
			tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

			_mm_storeu_si128((__m128i*)&dst[i * line + j * 4], tmpDst);
		}
	}
}

template<int WIDTH>
inline void _fastInverseMM_B32_10bit_sse128(const int *src, int *dst, int shift, int line, int iSkipLine, int iSkipLine2, const int outputMinimum, const int outputMaximum, const short* iT) {
	const int m128iNUM = WIDTH * 32 / 4;
	const int WIDTH_4 = WIDTH / 4;
	__m128i src_128[m128iNUM], coef_128[256];
	__m128i allSum, tmpDst, sum_0to3, sum_4to7, sum_8to11, sum_12to15, sum_16to19, sum_20to23, sum_24to27, sum_28to31,
		sum0, sum1, sum2, sum3;
	__m128i factor, min_val, max_val;
	__m128i tr[m128iNUM], tr_coef[256];

	// shift
	factor = _mm_set1_epi32(1 << (shift - 1));
	min_val = _mm_set1_epi32(outputMinimum);
	max_val = _mm_set1_epi32(outputMaximum);

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < m128iNUM; ++i) {
		int p_s = i * 4;
		src_128[i] = _mm_loadu_si128((__m128i*)(src + p_s));
	}
	for (int i = 0; i < 256; ++i) {
		int p_s = i * 4;
		coef_128[i] = _mm_set_epi32(iT[p_s + 3], iT[p_s + 2], iT[p_s + 1], iT[p_s + 0]);
	}

	// Transpose coefficient matrix
#define TRANSPOSE_Nx32_32BIT(I) \
	for (int i = 0; i < m128iNUM / 32; ++i) { \
		const int ix32 = i * 32;\
		tr[ix32 + 0] = _mm_unpacklo_epi32(I[i + 0], I[i + WIDTH_4]); \
		tr[ix32 + 1] = _mm_unpackhi_epi32(I[i + 0], I[i + WIDTH_4]); \
		tr[ix32 + 2] = _mm_unpacklo_epi32(I[i + 2 * WIDTH_4], I[i + 3 * WIDTH_4]); \
		tr[ix32 + 3] = _mm_unpackhi_epi32(I[i + 2 * WIDTH_4], I[i + 3 * WIDTH_4]); \
		tr[ix32 + 4] = _mm_unpacklo_epi32(I[i + 4 * WIDTH_4], I[i + 5 * WIDTH_4]); \
		tr[ix32 + 5] = _mm_unpackhi_epi32(I[i + 4 * WIDTH_4], I[i + 5 * WIDTH_4]); \
		tr[ix32 + 6] = _mm_unpacklo_epi32(I[i + 6 * WIDTH_4], I[i + 7 * WIDTH_4]); \
		tr[ix32 + 7] = _mm_unpackhi_epi32(I[i + 6 * WIDTH_4], I[i + 7 * WIDTH_4]); \
		tr[ix32 + 8] = _mm_unpacklo_epi32(I[i + 8 * WIDTH_4], I[i + 9 * WIDTH_4]); \
		tr[ix32 + 9] = _mm_unpackhi_epi32(I[i + 8 * WIDTH_4], I[i + 9 * WIDTH_4]); \
		tr[ix32 + 10] = _mm_unpacklo_epi32(I[i + 10 * WIDTH_4], I[i + 11 * WIDTH_4]); \
		tr[ix32 + 11] = _mm_unpackhi_epi32(I[i + 10 * WIDTH_4], I[i + 11 * WIDTH_4]); \
		tr[ix32 + 12] = _mm_unpacklo_epi32(I[i + 12 * WIDTH_4], I[i + 13 * WIDTH_4]); \
		tr[ix32 + 13] = _mm_unpackhi_epi32(I[i + 12 * WIDTH_4], I[i + 13 * WIDTH_4]); \
		tr[ix32 + 14] = _mm_unpacklo_epi32(I[i + 14 * WIDTH_4], I[i + 15 * WIDTH_4]); \
		tr[ix32 + 15] = _mm_unpackhi_epi32(I[i + 14 * WIDTH_4], I[i + 15 * WIDTH_4]); \
		tr[ix32 + 16] = _mm_unpacklo_epi32(I[i + 16 * WIDTH_4], I[i + 17 * WIDTH_4]); \
		tr[ix32 + 17] = _mm_unpackhi_epi32(I[i + 16 * WIDTH_4], I[i + 17 * WIDTH_4]); \
		tr[ix32 + 18] = _mm_unpacklo_epi32(I[i + 18 * WIDTH_4], I[i + 19 * WIDTH_4]); \
		tr[ix32 + 19] = _mm_unpackhi_epi32(I[i + 18 * WIDTH_4], I[i + 19 * WIDTH_4]); \
		tr[ix32 + 20] = _mm_unpacklo_epi32(I[i + 20 * WIDTH_4], I[i + 21 * WIDTH_4]); \
		tr[ix32 + 21] = _mm_unpackhi_epi32(I[i + 20 * WIDTH_4], I[i + 21 * WIDTH_4]); \
		tr[ix32 + 22] = _mm_unpacklo_epi32(I[i + 22 * WIDTH_4], I[i + 23 * WIDTH_4]); \
		tr[ix32 + 23] = _mm_unpackhi_epi32(I[i + 22 * WIDTH_4], I[i + 23 * WIDTH_4]); \
		tr[ix32 + 24] = _mm_unpacklo_epi32(I[i + 24 * WIDTH_4], I[i + 25 * WIDTH_4]); \
		tr[ix32 + 25] = _mm_unpackhi_epi32(I[i + 24 * WIDTH_4], I[i + 25 * WIDTH_4]); \
		tr[ix32 + 26] = _mm_unpacklo_epi32(I[i + 26 * WIDTH_4], I[i + 27 * WIDTH_4]); \
		tr[ix32 + 27] = _mm_unpackhi_epi32(I[i + 26 * WIDTH_4], I[i + 27 * WIDTH_4]); \
		tr[ix32 + 28] = _mm_unpacklo_epi32(I[i + 28 * WIDTH_4], I[i + 29 * WIDTH_4]); \
		tr[ix32 + 29] = _mm_unpackhi_epi32(I[i + 28 * WIDTH_4], I[i + 29 * WIDTH_4]); \
		tr[ix32 + 30] = _mm_unpacklo_epi32(I[i + 30 * WIDTH_4], I[i + 31 * WIDTH_4]); \
		tr[ix32 + 31] = _mm_unpackhi_epi32(I[i + 30 * WIDTH_4], I[i + 31 * WIDTH_4]); \
	} \
	for (int i = 0; i < m128iNUM / 32; ++i) { \
		const int ix32 = i * 32;\
		I[ix32 + 0] = _mm_unpacklo_epi64(tr[ix32 + 0], tr[ix32 + 2]); \
		I[ix32 + 1] = _mm_unpacklo_epi64(tr[ix32 + 4], tr[ix32 + 6]); \
		I[ix32 + 2] = _mm_unpacklo_epi64(tr[ix32 + 8], tr[ix32 + 10]); \
		I[ix32 + 3] = _mm_unpacklo_epi64(tr[ix32 + 12], tr[ix32 + 14]); \
		I[ix32 + 4] = _mm_unpacklo_epi64(tr[ix32 + 16], tr[ix32 + 18]); \
		I[ix32 + 5] = _mm_unpacklo_epi64(tr[ix32 + 20], tr[ix32 + 22]); \
		I[ix32 + 6] = _mm_unpacklo_epi64(tr[ix32 + 24], tr[ix32 + 26]); \
		I[ix32 + 7] = _mm_unpacklo_epi64(tr[ix32 + 28], tr[ix32 + 30]); \
		I[ix32 + 8] = _mm_unpackhi_epi64(tr[ix32 + 0], tr[ix32 + 2]); \
		I[ix32 + 9] = _mm_unpackhi_epi64(tr[ix32 + 4], tr[ix32 + 6]); \
		I[ix32 + 10] = _mm_unpackhi_epi64(tr[ix32 + 8], tr[ix32 + 10]); \
		I[ix32 + 11] = _mm_unpackhi_epi64(tr[ix32 + 12], tr[ix32 + 14]); \
		I[ix32 + 12] = _mm_unpackhi_epi64(tr[ix32 + 16], tr[ix32 + 18]); \
		I[ix32 + 13] = _mm_unpackhi_epi64(tr[ix32 + 20], tr[ix32 + 22]); \
		I[ix32 + 14] = _mm_unpackhi_epi64(tr[ix32 + 24], tr[ix32 + 26]); \
		I[ix32 + 15] = _mm_unpackhi_epi64(tr[ix32 + 28], tr[ix32 + 30]); \
		I[ix32 + 16] = _mm_unpacklo_epi64(tr[ix32 + 1], tr[ix32 + 3]); \
		I[ix32 + 17] = _mm_unpacklo_epi64(tr[ix32 + 5], tr[ix32 + 7]); \
		I[ix32 + 18] = _mm_unpacklo_epi64(tr[ix32 + 9], tr[ix32 + 11]); \
		I[ix32 + 19] = _mm_unpacklo_epi64(tr[ix32 + 13], tr[ix32 + 15]); \
		I[ix32 + 20] = _mm_unpacklo_epi64(tr[ix32 + 17], tr[ix32 + 19]); \
		I[ix32 + 21] = _mm_unpacklo_epi64(tr[ix32 + 21], tr[ix32 + 23]); \
		I[ix32 + 22] = _mm_unpacklo_epi64(tr[ix32 + 25], tr[ix32 + 27]); \
		I[ix32 + 23] = _mm_unpacklo_epi64(tr[ix32 + 29], tr[ix32 + 31]); \
		I[ix32 + 24] = _mm_unpackhi_epi64(tr[ix32 + 1], tr[ix32 + 3]); \
		I[ix32 + 25] = _mm_unpackhi_epi64(tr[ix32 + 5], tr[ix32 + 7]); \
		I[ix32 + 26] = _mm_unpackhi_epi64(tr[ix32 + 9], tr[ix32 + 11]); \
		I[ix32 + 27] = _mm_unpackhi_epi64(tr[ix32 + 13], tr[ix32 + 15]); \
		I[ix32 + 28] = _mm_unpackhi_epi64(tr[ix32 + 17], tr[ix32 + 19]); \
		I[ix32 + 29] = _mm_unpackhi_epi64(tr[ix32 + 21], tr[ix32 + 23]); \
		I[ix32 + 30] = _mm_unpackhi_epi64(tr[ix32 + 25], tr[ix32 + 27]); \
		I[ix32 + 31] = _mm_unpackhi_epi64(tr[ix32 + 29], tr[ix32 + 31]); \
	}\

	TRANSPOSE_Nx32_32BIT(src_128);
#undef TRANSPOSE_Nx32_32BIT

	// Transpose coefficient matrix
#define TRANSPOSE_32x32_32BIT(I) \
	for (int i = 0; i < 8; ++i) { \
		const int ix32 = i * 32;\
		tr_coef[ix32 + 0] = _mm_unpacklo_epi32(I[i + 0], I[i + 8]); \
		tr_coef[ix32 + 1] = _mm_unpackhi_epi32(I[i + 0], I[i + 8]); \
		tr_coef[ix32 + 2] = _mm_unpacklo_epi32(I[i + 2 * 8], I[i + 3 * 8]); \
		tr_coef[ix32 + 3] = _mm_unpackhi_epi32(I[i + 2 * 8], I[i + 3 * 8]); \
		tr_coef[ix32 + 4] = _mm_unpacklo_epi32(I[i + 4 * 8], I[i + 5 * 8]); \
		tr_coef[ix32 + 5] = _mm_unpackhi_epi32(I[i + 4 * 8], I[i + 5 * 8]); \
		tr_coef[ix32 + 6] = _mm_unpacklo_epi32(I[i + 6 * 8], I[i + 7 * 8]); \
		tr_coef[ix32 + 7] = _mm_unpackhi_epi32(I[i + 6 * 8], I[i + 7 * 8]); \
		tr_coef[ix32 + 8] = _mm_unpacklo_epi32(I[i + 8 * 8], I[i + 9 * 8]); \
		tr_coef[ix32 + 9] = _mm_unpackhi_epi32(I[i + 8 * 8], I[i + 9 * 8]); \
		tr_coef[ix32 + 10] = _mm_unpacklo_epi32(I[i + 10 * 8], I[i + 11 * 8]); \
		tr_coef[ix32 + 11] = _mm_unpackhi_epi32(I[i + 10 * 8], I[i + 11 * 8]); \
		tr_coef[ix32 + 12] = _mm_unpacklo_epi32(I[i + 12 * 8], I[i + 13 * 8]); \
		tr_coef[ix32 + 13] = _mm_unpackhi_epi32(I[i + 12 * 8], I[i + 13 * 8]); \
		tr_coef[ix32 + 14] = _mm_unpacklo_epi32(I[i + 14 * 8], I[i + 15 * 8]); \
		tr_coef[ix32 + 15] = _mm_unpackhi_epi32(I[i + 14 * 8], I[i + 15 * 8]); \
		tr_coef[ix32 + 16] = _mm_unpacklo_epi32(I[i + 16 * 8], I[i + 17 * 8]); \
		tr_coef[ix32 + 17] = _mm_unpackhi_epi32(I[i + 16 * 8], I[i + 17 * 8]); \
		tr_coef[ix32 + 18] = _mm_unpacklo_epi32(I[i + 18 * 8], I[i + 19 * 8]); \
		tr_coef[ix32 + 19] = _mm_unpackhi_epi32(I[i + 18 * 8], I[i + 19 * 8]); \
		tr_coef[ix32 + 20] = _mm_unpacklo_epi32(I[i + 20 * 8], I[i + 21 * 8]); \
		tr_coef[ix32 + 21] = _mm_unpackhi_epi32(I[i + 20 * 8], I[i + 21 * 8]); \
		tr_coef[ix32 + 22] = _mm_unpacklo_epi32(I[i + 22 * 8], I[i + 23 * 8]); \
		tr_coef[ix32 + 23] = _mm_unpackhi_epi32(I[i + 22 * 8], I[i + 23 * 8]); \
		tr_coef[ix32 + 24] = _mm_unpacklo_epi32(I[i + 24 * 8], I[i + 25 * 8]); \
		tr_coef[ix32 + 25] = _mm_unpackhi_epi32(I[i + 24 * 8], I[i + 25 * 8]); \
		tr_coef[ix32 + 26] = _mm_unpacklo_epi32(I[i + 26 * 8], I[i + 27 * 8]); \
		tr_coef[ix32 + 27] = _mm_unpackhi_epi32(I[i + 26 * 8], I[i + 27 * 8]); \
		tr_coef[ix32 + 28] = _mm_unpacklo_epi32(I[i + 28 * 8], I[i + 29 * 8]); \
		tr_coef[ix32 + 29] = _mm_unpackhi_epi32(I[i + 28 * 8], I[i + 29 * 8]); \
		tr_coef[ix32 + 30] = _mm_unpacklo_epi32(I[i + 30 * 8], I[i + 31 * 8]); \
		tr_coef[ix32 + 31] = _mm_unpackhi_epi32(I[i + 30 * 8], I[i + 31 * 8]); \
	} \
	for (int i = 0; i < 8; ++i) { \
		const int ix32 = i * 32;\
		I[ix32 + 0] = _mm_unpacklo_epi64(tr_coef[ix32 + 0], tr_coef[ix32 + 2]); \
		I[ix32 + 1] = _mm_unpacklo_epi64(tr_coef[ix32 + 4], tr_coef[ix32 + 6]); \
		I[ix32 + 2] = _mm_unpacklo_epi64(tr_coef[ix32 + 8], tr_coef[ix32 + 10]); \
		I[ix32 + 3] = _mm_unpacklo_epi64(tr_coef[ix32 + 12], tr_coef[ix32 + 14]); \
		I[ix32 + 4] = _mm_unpacklo_epi64(tr_coef[ix32 + 16], tr_coef[ix32 + 18]); \
		I[ix32 + 5] = _mm_unpacklo_epi64(tr_coef[ix32 + 20], tr_coef[ix32 + 22]); \
		I[ix32 + 6] = _mm_unpacklo_epi64(tr_coef[ix32 + 24], tr_coef[ix32 + 26]); \
		I[ix32 + 7] = _mm_unpacklo_epi64(tr_coef[ix32 + 28], tr_coef[ix32 + 30]); \
		I[ix32 + 8] = _mm_unpackhi_epi64(tr_coef[ix32 + 0], tr_coef[ix32 + 2]); \
		I[ix32 + 9] = _mm_unpackhi_epi64(tr_coef[ix32 + 4], tr_coef[ix32 + 6]); \
		I[ix32 + 10] = _mm_unpackhi_epi64(tr_coef[ix32 + 8], tr_coef[ix32 + 10]); \
		I[ix32 + 11] = _mm_unpackhi_epi64(tr_coef[ix32 + 12], tr_coef[ix32 + 14]); \
		I[ix32 + 12] = _mm_unpackhi_epi64(tr_coef[ix32 + 16], tr_coef[ix32 + 18]); \
		I[ix32 + 13] = _mm_unpackhi_epi64(tr_coef[ix32 + 20], tr_coef[ix32 + 22]); \
		I[ix32 + 14] = _mm_unpackhi_epi64(tr_coef[ix32 + 24], tr_coef[ix32 + 26]); \
		I[ix32 + 15] = _mm_unpackhi_epi64(tr_coef[ix32 + 28], tr_coef[ix32 + 30]); \
		I[ix32 + 16] = _mm_unpacklo_epi64(tr_coef[ix32 + 1], tr_coef[ix32 + 3]); \
		I[ix32 + 17] = _mm_unpacklo_epi64(tr_coef[ix32 + 5], tr_coef[ix32 + 7]); \
		I[ix32 + 18] = _mm_unpacklo_epi64(tr_coef[ix32 + 9], tr_coef[ix32 + 11]); \
		I[ix32 + 19] = _mm_unpacklo_epi64(tr_coef[ix32 + 13], tr_coef[ix32 + 15]); \
		I[ix32 + 20] = _mm_unpacklo_epi64(tr_coef[ix32 + 17], tr_coef[ix32 + 19]); \
		I[ix32 + 21] = _mm_unpacklo_epi64(tr_coef[ix32 + 21], tr_coef[ix32 + 23]); \
		I[ix32 + 22] = _mm_unpacklo_epi64(tr_coef[ix32 + 25], tr_coef[ix32 + 27]); \
		I[ix32 + 23] = _mm_unpacklo_epi64(tr_coef[ix32 + 29], tr_coef[ix32 + 31]); \
		I[ix32 + 24] = _mm_unpackhi_epi64(tr_coef[ix32 + 1], tr_coef[ix32 + 3]); \
		I[ix32 + 25] = _mm_unpackhi_epi64(tr_coef[ix32 + 5], tr_coef[ix32 + 7]); \
		I[ix32 + 26] = _mm_unpackhi_epi64(tr_coef[ix32 + 9], tr_coef[ix32 + 11]); \
		I[ix32 + 27] = _mm_unpackhi_epi64(tr_coef[ix32 + 13], tr_coef[ix32 + 15]); \
		I[ix32 + 28] = _mm_unpackhi_epi64(tr_coef[ix32 + 17], tr_coef[ix32 + 19]); \
		I[ix32 + 29] = _mm_unpackhi_epi64(tr_coef[ix32 + 21], tr_coef[ix32 + 23]); \
		I[ix32 + 30] = _mm_unpackhi_epi64(tr_coef[ix32 + 25], tr_coef[ix32 + 27]); \
		I[ix32 + 31] = _mm_unpackhi_epi64(tr_coef[ix32 + 29], tr_coef[ix32 + 31]); \
	}\

	TRANSPOSE_32x32_32BIT(coef_128);
#undef TRANSPOSE_32x32_32BIT

	int a = 0;

	for (int i = 0; i < WIDTH; i++) {
		for (int j = 0; j < 8; j++) {
			// Four results
			// 0
			sum_0to3 = _mm_mullo_epi32(coef_128[j * 32 + 0], src_128[i * 8]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 32 + 1], src_128[i * 8 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[j * 32 + 2], src_128[i * 8 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[j * 32 + 3], src_128[i * 8 + 3]);
			sum_16to19 = _mm_mullo_epi32(coef_128[j * 32 + 4], src_128[i * 8 + 4]);
			sum_20to23 = _mm_mullo_epi32(coef_128[j * 32 + 5], src_128[i * 8 + 5]);
			sum_24to27 = _mm_mullo_epi32(coef_128[j * 32 + 6], src_128[i * 8 + 6]);
			sum_28to31 = _mm_mullo_epi32(coef_128[j * 32 + 7], src_128[i * 8 + 7]);

			sum0 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum0 = _mm_add_epi32(sum0, sum_8to11);
			sum0 = _mm_add_epi32(sum0, sum_12to15);
			sum0 = _mm_add_epi32(sum0, sum_16to19);
			sum0 = _mm_add_epi32(sum0, sum_20to23);
			sum0 = _mm_add_epi32(sum0, sum_24to27);
			sum0 = _mm_add_epi32(sum0, sum_28to31);
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 8));
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

			// 1
			sum_0to3 = _mm_mullo_epi32(coef_128[j * 32 + 8], src_128[i * 8]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 32 + 9], src_128[i * 8 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[j * 32 + 10], src_128[i * 8 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[j * 32 + 11], src_128[i * 8 + 3]);
			sum_16to19 = _mm_mullo_epi32(coef_128[j * 32 + 12], src_128[i * 8 + 4]);
			sum_20to23 = _mm_mullo_epi32(coef_128[j * 32 + 13], src_128[i * 8 + 5]);
			sum_24to27 = _mm_mullo_epi32(coef_128[j * 32 + 14], src_128[i * 8 + 6]);
			sum_28to31 = _mm_mullo_epi32(coef_128[j * 32 + 15], src_128[i * 8 + 7]);

			sum1 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum1 = _mm_add_epi32(sum1, sum_8to11);
			sum1 = _mm_add_epi32(sum1, sum_12to15);
			sum1 = _mm_add_epi32(sum1, sum_16to19);
			sum1 = _mm_add_epi32(sum1, sum_20to23);
			sum1 = _mm_add_epi32(sum1, sum_24to27);
			sum1 = _mm_add_epi32(sum1, sum_28to31);
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 8));
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

			// 2
			sum_0to3 = _mm_mullo_epi32(coef_128[j * 32 + 16], src_128[i * 8]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 32 + 17], src_128[i * 8 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[j * 32 + 18], src_128[i * 8 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[j * 32 + 19], src_128[i * 8 + 3]);
			sum_16to19 = _mm_mullo_epi32(coef_128[j * 32 + 20], src_128[i * 8 + 4]);
			sum_20to23 = _mm_mullo_epi32(coef_128[j * 32 + 21], src_128[i * 8 + 5]);
			sum_24to27 = _mm_mullo_epi32(coef_128[j * 32 + 22], src_128[i * 8 + 6]);
			sum_28to31 = _mm_mullo_epi32(coef_128[j * 32 + 23], src_128[i * 8 + 7]);

			sum2 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum2 = _mm_add_epi32(sum2, sum_8to11);
			sum2 = _mm_add_epi32(sum2, sum_12to15);
			sum2 = _mm_add_epi32(sum2, sum_16to19);
			sum2 = _mm_add_epi32(sum2, sum_20to23);
			sum2 = _mm_add_epi32(sum2, sum_24to27);
			sum2 = _mm_add_epi32(sum2, sum_28to31);
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 8));
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

			// 3
			sum_0to3 = _mm_mullo_epi32(coef_128[j * 32 + 24], src_128[i * 8]);
			sum_4to7 = _mm_mullo_epi32(coef_128[j * 32 + 25], src_128[i * 8 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[j * 32 + 26], src_128[i * 8 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[j * 32 + 27], src_128[i * 8 + 3]);
			sum_16to19 = _mm_mullo_epi32(coef_128[j * 32 + 28], src_128[i * 8 + 4]);
			sum_20to23 = _mm_mullo_epi32(coef_128[j * 32 + 29], src_128[i * 8 + 5]);
			sum_24to27 = _mm_mullo_epi32(coef_128[j * 32 + 30], src_128[i * 8 + 6]);
			sum_28to31 = _mm_mullo_epi32(coef_128[j * 32 + 31], src_128[i * 8 + 7]);

			sum3 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum3 = _mm_add_epi32(sum3, sum_8to11);
			sum3 = _mm_add_epi32(sum3, sum_12to15);
			sum3 = _mm_add_epi32(sum3, sum_16to19);
			sum3 = _mm_add_epi32(sum3, sum_20to23);
			sum3 = _mm_add_epi32(sum3, sum_24to27);
			sum3 = _mm_add_epi32(sum3, sum_28to31);
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 8));
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 4));

			// Sum mul of result 0 1 2 3
			sum0 = _mm_unpacklo_epi32(sum0, sum1);
			sum2 = _mm_unpacklo_epi32(sum2, sum3);
			allSum = _mm_unpacklo_epi64(sum0, sum2);
			tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

			//clip
			tmpDst = _mm_min_epi32(tmpDst, max_val);
			tmpDst = _mm_max_epi32(tmpDst, min_val);

			_mm_storeu_si128((__m128i*)&dst[i * 32 + j * 4], tmpDst);
		}
	}

}

template<int HEIGHT>
inline void _fastForwardMM_B64_10bit_sse128(const int *src, int *dst, int shift, int line, int iSkipLine, int iSkipLine2, const short* tc)
{
	const int m128iNUM = HEIGHT * 64 / 4;
	const int HEIGHT_4 = HEIGHT / 4;
	__m128i src_128[m128iNUM], coef_128[1024];
	__m128i allSum, tmpDst, sum_0to3, sum_4to7, sum_8to11, sum_12to15, sum_16to19, sum_20to23, sum_24to27, sum_28to31,
		sum_32to35, sum_36to39, sum_40to43, sum_44to47, sum_48to51, sum_52to55, sum_56to59, sum_60to63, sum0, sum1, sum2, sum3;
	__m128i factor;

	// shift
	factor = _mm_set1_epi32(1 << (shift - 1));

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < m128iNUM; ++i) {
		int p_s = i * 4;
		src_128[i] = _mm_loadu_si128((__m128i*)(src + p_s));
	}
	for (int i = 0; i < 1024; ++i) {
		int p_s = i * 4;
		coef_128[i] = _mm_set_epi32(tc[p_s + 3], tc[p_s + 2], tc[p_s + 1], tc[p_s + 0]);
	}

	for (int i = 0; i < 64; i++) {
		const int ix16 = i * 16;
		for (int j = 0; j < HEIGHT_4; j++) {
			const int jx64 = j * 64;
			// Four results
			// 0
			sum_0to3 = _mm_mullo_epi32(src_128[jx64 + 0], coef_128[ix16 + 0]);
			sum_4to7 = _mm_mullo_epi32(src_128[jx64 + 1], coef_128[ix16 + 1]);
			sum_8to11 = _mm_mullo_epi32(src_128[jx64 + 2], coef_128[ix16 + 2]);
			sum_12to15 = _mm_mullo_epi32(src_128[jx64 + 3], coef_128[ix16 + 3]);
			sum_16to19 = _mm_mullo_epi32(src_128[jx64 + 4], coef_128[ix16 + 4]);
			sum_20to23 = _mm_mullo_epi32(src_128[jx64 + 5], coef_128[ix16 + 5]);
			sum_24to27 = _mm_mullo_epi32(src_128[jx64 + 6], coef_128[ix16 + 6]);
			sum_28to31 = _mm_mullo_epi32(src_128[jx64 + 7], coef_128[ix16 + 7]);
			sum_32to35 = _mm_mullo_epi32(src_128[jx64 + 8], coef_128[ix16 + 8]);
			sum_36to39 = _mm_mullo_epi32(src_128[jx64 + 9], coef_128[ix16 + 9]);
			sum_40to43 = _mm_mullo_epi32(src_128[jx64 + 10], coef_128[ix16 + 10]);
			sum_44to47 = _mm_mullo_epi32(src_128[jx64 + 11], coef_128[ix16 + 11]);
			sum_48to51 = _mm_mullo_epi32(src_128[jx64 + 12], coef_128[ix16 + 12]);
			sum_52to55 = _mm_mullo_epi32(src_128[jx64 + 13], coef_128[ix16 + 13]);
			sum_56to59 = _mm_mullo_epi32(src_128[jx64 + 14], coef_128[ix16 + 14]);
			sum_60to63 = _mm_mullo_epi32(src_128[jx64 + 15], coef_128[ix16 + 15]);

			sum0 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum0 = _mm_add_epi32(sum0, sum_8to11);
			sum0 = _mm_add_epi32(sum0, sum_12to15);
			sum0 = _mm_add_epi32(sum0, sum_16to19);
			sum0 = _mm_add_epi32(sum0, sum_20to23);
			sum0 = _mm_add_epi32(sum0, sum_24to27);
			sum0 = _mm_add_epi32(sum0, sum_28to31);
			sum0 = _mm_add_epi32(sum0, sum_32to35);
			sum0 = _mm_add_epi32(sum0, sum_36to39);
			sum0 = _mm_add_epi32(sum0, sum_40to43);
			sum0 = _mm_add_epi32(sum0, sum_44to47);
			sum0 = _mm_add_epi32(sum0, sum_48to51);
			sum0 = _mm_add_epi32(sum0, sum_52to55);
			sum0 = _mm_add_epi32(sum0, sum_56to59);
			sum0 = _mm_add_epi32(sum0, sum_60to63);
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 8));
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

			// 1
			sum_0to3 = _mm_mullo_epi32(src_128[jx64 + 16], coef_128[ix16 + 0]);
			sum_4to7 = _mm_mullo_epi32(src_128[jx64 + 17], coef_128[ix16 + 1]);
			sum_8to11 = _mm_mullo_epi32(src_128[jx64 + 18], coef_128[ix16 + 2]);
			sum_12to15 = _mm_mullo_epi32(src_128[jx64 + 19], coef_128[ix16 + 3]);
			sum_16to19 = _mm_mullo_epi32(src_128[jx64 + 20], coef_128[ix16 + 4]);
			sum_20to23 = _mm_mullo_epi32(src_128[jx64 + 21], coef_128[ix16 + 5]);
			sum_24to27 = _mm_mullo_epi32(src_128[jx64 + 22], coef_128[ix16 + 6]);
			sum_28to31 = _mm_mullo_epi32(src_128[jx64 + 23], coef_128[ix16 + 7]);
			sum_32to35 = _mm_mullo_epi32(src_128[jx64 + 24], coef_128[ix16 + 8]);
			sum_36to39 = _mm_mullo_epi32(src_128[jx64 + 25], coef_128[ix16 + 9]);
			sum_40to43 = _mm_mullo_epi32(src_128[jx64 + 26], coef_128[ix16 + 10]);
			sum_44to47 = _mm_mullo_epi32(src_128[jx64 + 27], coef_128[ix16 + 11]);
			sum_48to51 = _mm_mullo_epi32(src_128[jx64 + 28], coef_128[ix16 + 12]);
			sum_52to55 = _mm_mullo_epi32(src_128[jx64 + 29], coef_128[ix16 + 13]);
			sum_56to59 = _mm_mullo_epi32(src_128[jx64 + 30], coef_128[ix16 + 14]);
			sum_60to63 = _mm_mullo_epi32(src_128[jx64 + 31], coef_128[ix16 + 15]);

			sum1 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum1 = _mm_add_epi32(sum1, sum_8to11);
			sum1 = _mm_add_epi32(sum1, sum_12to15);
			sum1 = _mm_add_epi32(sum1, sum_16to19);
			sum1 = _mm_add_epi32(sum1, sum_20to23);
			sum1 = _mm_add_epi32(sum1, sum_24to27);
			sum1 = _mm_add_epi32(sum1, sum_28to31);
			sum1 = _mm_add_epi32(sum1, sum_32to35);
			sum1 = _mm_add_epi32(sum1, sum_36to39);
			sum1 = _mm_add_epi32(sum1, sum_40to43);
			sum1 = _mm_add_epi32(sum1, sum_44to47);
			sum1 = _mm_add_epi32(sum1, sum_48to51);
			sum1 = _mm_add_epi32(sum1, sum_52to55);
			sum1 = _mm_add_epi32(sum1, sum_56to59);
			sum1 = _mm_add_epi32(sum1, sum_60to63);
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 8));
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

			// 2
			sum_0to3 = _mm_mullo_epi32(src_128[jx64 + 32], coef_128[ix16 + 0]);
			sum_4to7 = _mm_mullo_epi32(src_128[jx64 + 33], coef_128[ix16 + 1]);
			sum_8to11 = _mm_mullo_epi32(src_128[jx64 + 34], coef_128[ix16 + 2]);
			sum_12to15 = _mm_mullo_epi32(src_128[jx64 + 35], coef_128[ix16 + 3]);
			sum_16to19 = _mm_mullo_epi32(src_128[jx64 + 36], coef_128[ix16 + 4]);
			sum_20to23 = _mm_mullo_epi32(src_128[jx64 + 37], coef_128[ix16 + 5]);
			sum_24to27 = _mm_mullo_epi32(src_128[jx64 + 38], coef_128[ix16 + 6]);
			sum_28to31 = _mm_mullo_epi32(src_128[jx64 + 39], coef_128[ix16 + 7]);
			sum_32to35 = _mm_mullo_epi32(src_128[jx64 + 40], coef_128[ix16 + 8]);
			sum_36to39 = _mm_mullo_epi32(src_128[jx64 + 41], coef_128[ix16 + 9]);
			sum_40to43 = _mm_mullo_epi32(src_128[jx64 + 42], coef_128[ix16 + 10]);
			sum_44to47 = _mm_mullo_epi32(src_128[jx64 + 43], coef_128[ix16 + 11]);
			sum_48to51 = _mm_mullo_epi32(src_128[jx64 + 44], coef_128[ix16 + 12]);
			sum_52to55 = _mm_mullo_epi32(src_128[jx64 + 45], coef_128[ix16 + 13]);
			sum_56to59 = _mm_mullo_epi32(src_128[jx64 + 46], coef_128[ix16 + 14]);
			sum_60to63 = _mm_mullo_epi32(src_128[jx64 + 47], coef_128[ix16 + 15]);

			sum2 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum2 = _mm_add_epi32(sum2, sum_8to11);
			sum2 = _mm_add_epi32(sum2, sum_12to15);
			sum2 = _mm_add_epi32(sum2, sum_16to19);
			sum2 = _mm_add_epi32(sum2, sum_20to23);
			sum2 = _mm_add_epi32(sum2, sum_24to27);
			sum2 = _mm_add_epi32(sum2, sum_28to31);
			sum2 = _mm_add_epi32(sum2, sum_32to35);
			sum2 = _mm_add_epi32(sum2, sum_36to39);
			sum2 = _mm_add_epi32(sum2, sum_40to43);
			sum2 = _mm_add_epi32(sum2, sum_44to47);
			sum2 = _mm_add_epi32(sum2, sum_48to51);
			sum2 = _mm_add_epi32(sum2, sum_52to55);
			sum2 = _mm_add_epi32(sum2, sum_56to59);
			sum2 = _mm_add_epi32(sum2, sum_60to63);
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 8));
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

			// 3
			sum_0to3 = _mm_mullo_epi32(src_128[jx64 + 48], coef_128[ix16 + 0]);
			sum_4to7 = _mm_mullo_epi32(src_128[jx64 + 49], coef_128[ix16 + 1]);
			sum_8to11 = _mm_mullo_epi32(src_128[jx64 + 50], coef_128[ix16 + 2]);
			sum_12to15 = _mm_mullo_epi32(src_128[jx64 + 51], coef_128[ix16 + 3]);
			sum_16to19 = _mm_mullo_epi32(src_128[jx64 + 52], coef_128[ix16 + 4]);
			sum_20to23 = _mm_mullo_epi32(src_128[jx64 + 53], coef_128[ix16 + 5]);
			sum_24to27 = _mm_mullo_epi32(src_128[jx64 + 54], coef_128[ix16 + 6]);
			sum_28to31 = _mm_mullo_epi32(src_128[jx64 + 55], coef_128[ix16 + 7]);
			sum_32to35 = _mm_mullo_epi32(src_128[jx64 + 56], coef_128[ix16 + 8]);
			sum_36to39 = _mm_mullo_epi32(src_128[jx64 + 57], coef_128[ix16 + 9]);
			sum_40to43 = _mm_mullo_epi32(src_128[jx64 + 58], coef_128[ix16 + 10]);
			sum_44to47 = _mm_mullo_epi32(src_128[jx64 + 59], coef_128[ix16 + 11]);
			sum_48to51 = _mm_mullo_epi32(src_128[jx64 + 60], coef_128[ix16 + 12]);
			sum_52to55 = _mm_mullo_epi32(src_128[jx64 + 61], coef_128[ix16 + 13]);
			sum_56to59 = _mm_mullo_epi32(src_128[jx64 + 62], coef_128[ix16 + 14]);
			sum_60to63 = _mm_mullo_epi32(src_128[jx64 + 63], coef_128[ix16 + 15]);

			sum3 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum3 = _mm_add_epi32(sum3, sum_8to11);
			sum3 = _mm_add_epi32(sum3, sum_12to15);
			sum3 = _mm_add_epi32(sum3, sum_16to19);
			sum3 = _mm_add_epi32(sum3, sum_20to23);
			sum3 = _mm_add_epi32(sum3, sum_24to27);
			sum3 = _mm_add_epi32(sum3, sum_28to31);
			sum3 = _mm_add_epi32(sum3, sum_32to35);
			sum3 = _mm_add_epi32(sum3, sum_36to39);
			sum3 = _mm_add_epi32(sum3, sum_40to43);
			sum3 = _mm_add_epi32(sum3, sum_44to47);
			sum3 = _mm_add_epi32(sum3, sum_48to51);
			sum3 = _mm_add_epi32(sum3, sum_52to55);
			sum3 = _mm_add_epi32(sum3, sum_56to59);
			sum3 = _mm_add_epi32(sum3, sum_60to63);
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 8));
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 4));

			// Sum mul of result 0 1 2 3
			sum0 = _mm_unpacklo_epi32(sum0, sum1);
			sum2 = _mm_unpacklo_epi32(sum2, sum3);
			allSum = _mm_unpacklo_epi64(sum0, sum2);
			tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

			_mm_storeu_si128((__m128i*)&dst[i * HEIGHT + j * 4], tmpDst);
		}
	}
}

template<int WIDTH>
inline void _fastInverseMM_B64_10bit_sse128(const int *src, int *dst, int shift, int line, int iSkipLine, int iSkipLine2, const int outputMinimum, const int outputMaximum, const short* iT) {
	const int m128iNUM = WIDTH * 64 / 4;
	const int WIDTH_4 = WIDTH / 4;
	__m128i src_128[m128iNUM], coef_128[1024];
	__m128i allSum, tmpDst, sum_0to3, sum_4to7, sum_8to11, sum_12to15, sum_16to19, sum_20to23, sum_24to27, sum_28to31,
		sum_32to35, sum_36to39, sum_40to43, sum_44to47, sum_48to51, sum_52to55, sum_56to59, sum_60to63, sum0, sum1, sum2, sum3;
	__m128i factor, min_val, max_val;
	__m128i tr[m128iNUM], tr_coef[1024];

	// shift
	factor = _mm_set1_epi32(1 << (shift - 1));
	min_val = _mm_set1_epi32(outputMinimum);
	max_val = _mm_set1_epi32(outputMaximum);

	// Load source data and coefficient matrix as 32-bit unsigned int
	for (int i = 0; i < m128iNUM; ++i) {
		int p_s = i * 4;
		src_128[i] = _mm_loadu_si128((__m128i*)(src + p_s));
	}
	for (int i = 0; i < 1024; ++i) {
		int p_s = i * 4;
		coef_128[i] = _mm_set_epi32(iT[p_s + 3], iT[p_s + 2], iT[p_s + 1], iT[p_s + 0]);
	}

	// Transpose coefficient matrix
#define TRANSPOSE_Nx64_32BIT(I) \
	for (int i = 0; i < m128iNUM / 64; ++i) { \
		const int ix64 = i * 64;\
		for (int j = 0; j < 64; j += 2) { \
			tr[ix64 + j] = _mm_unpacklo_epi32(I[i + j * WIDTH_4], I[i + (j+1) * WIDTH_4]); \
			tr[ix64 + j + 1] = _mm_unpackhi_epi32(I[i + j * WIDTH_4], I[i + (j+1) * WIDTH_4]); \
		} \
	} \
	for (int i = 0; i < m128iNUM / 64; ++i) { \
		const int ix64 = i * 64;\
		for (int j = 0; j < 16; ++j) { \
				I[ix64 + j] = _mm_unpacklo_epi64(tr[ix64 + j * 4], tr[ix64 + j * 4 + 2]); \
				I[ix64 + j + 16] = _mm_unpackhi_epi64(tr[ix64 + j * 4], tr[ix64 + j * 4 + 2]); \
		} \
		for (int j = 0; j < 16; ++j) { \
				I[ix64 + 32 + j] = _mm_unpacklo_epi64(tr[ix64 + j * 4 + 1], tr[ix64 + j * 4 + 3]); \
				I[ix64 + 48 + j] = _mm_unpackhi_epi64(tr[ix64 + j * 4 + 1], tr[ix64 + j * 4 + 3]); \
		} \
	}\

	TRANSPOSE_Nx64_32BIT(src_128);
#undef TRANSPOSE_Nx64_32BIT

	// Transpose coefficient matrix
#define TRANSPOSE_64x64_32BIT(I) \
	for (int i = 0; i < 16; ++i) { \
		const int ix64 = i * 64;\
		for (int j = 0; j < 64; j += 2) { \
			tr_coef[ix64 + j] = _mm_unpacklo_epi32(I[i + j * 16], I[i + (j+1) * 16]); \
			tr_coef[ix64 + j + 1] = _mm_unpackhi_epi32(I[i + j * 16], I[i + (j+1) * 16]); \
		} \
	} \
	for (int i = 0; i < 16; ++i) { \
		const int ix64 = i * 64;\
		for (int j = 0; j < 16; ++j) { \
				I[ix64 + j] = _mm_unpacklo_epi64(tr_coef[ix64 + j * 4], tr_coef[ix64 + j * 4 + 2]); \
				I[ix64 + j + 16] = _mm_unpackhi_epi64(tr_coef[ix64 + j * 4], tr_coef[ix64 + j * 4 + 2]); \
		} \
		for (int j = 0; j < 16; ++j) { \
				I[ix64 + 32 + j] = _mm_unpacklo_epi64(tr_coef[ix64 + j * 4 + 1], tr_coef[ix64 + j * 4 + 3]); \
				I[ix64 + 48 + j] = _mm_unpackhi_epi64(tr_coef[ix64 + j * 4 + 1], tr_coef[ix64 + j * 4 + 3]); \
		} \
	}\

	TRANSPOSE_64x64_32BIT(coef_128);
#undef TRANSPOSE_64x64_32BIT

	for (int i = 0; i < WIDTH; i++) {
		const int ix16 = i * 16;
		for (int j = 0; j < 16; j++) {
			const int jx64 = j * 64;
			// Four results
			// 0
			sum_0to3 = _mm_mullo_epi32(coef_128[jx64 + 0], src_128[ix16 + 0]);
			sum_4to7 = _mm_mullo_epi32(coef_128[jx64 + 1], src_128[ix16 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[jx64 + 2], src_128[ix16 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[jx64 + 3], src_128[ix16 + 3]);
			sum_16to19 = _mm_mullo_epi32(coef_128[jx64 + 4], src_128[ix16 + 4]);
			sum_20to23 = _mm_mullo_epi32(coef_128[jx64 + 5], src_128[ix16 + 5]);
			sum_24to27 = _mm_mullo_epi32(coef_128[jx64 + 6], src_128[ix16 + 6]);
			sum_28to31 = _mm_mullo_epi32(coef_128[jx64 + 7], src_128[ix16 + 7]);
			sum_32to35 = _mm_mullo_epi32(coef_128[jx64 + 8], src_128[ix16 + 8]);
			sum_36to39 = _mm_mullo_epi32(coef_128[jx64 + 9], src_128[ix16 + 9]);
			sum_40to43 = _mm_mullo_epi32(coef_128[jx64 + 10], src_128[ix16 + 10]);
			sum_44to47 = _mm_mullo_epi32(coef_128[jx64 + 11], src_128[ix16 + 11]);
			sum_48to51 = _mm_mullo_epi32(coef_128[jx64 + 12], src_128[ix16 + 12]);
			sum_52to55 = _mm_mullo_epi32(coef_128[jx64 + 13], src_128[ix16 + 13]);
			sum_56to59 = _mm_mullo_epi32(coef_128[jx64 + 14], src_128[ix16 + 14]);
			sum_60to63 = _mm_mullo_epi32(coef_128[jx64 + 15], src_128[ix16 + 15]);

			sum0 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum0 = _mm_add_epi32(sum0, sum_8to11);
			sum0 = _mm_add_epi32(sum0, sum_12to15);
			sum0 = _mm_add_epi32(sum0, sum_16to19);
			sum0 = _mm_add_epi32(sum0, sum_20to23);
			sum0 = _mm_add_epi32(sum0, sum_24to27);
			sum0 = _mm_add_epi32(sum0, sum_28to31);
			sum0 = _mm_add_epi32(sum0, sum_32to35);
			sum0 = _mm_add_epi32(sum0, sum_36to39);
			sum0 = _mm_add_epi32(sum0, sum_40to43);
			sum0 = _mm_add_epi32(sum0, sum_44to47);
			sum0 = _mm_add_epi32(sum0, sum_48to51);
			sum0 = _mm_add_epi32(sum0, sum_52to55);
			sum0 = _mm_add_epi32(sum0, sum_56to59);
			sum0 = _mm_add_epi32(sum0, sum_60to63);
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 8));
			sum0 = _mm_add_epi32(sum0, _mm_srli_si128(sum0, 4));

			// 1
			sum_0to3 = _mm_mullo_epi32(coef_128[jx64 + 16], src_128[ix16 + 0]);
			sum_4to7 = _mm_mullo_epi32(coef_128[jx64 + 17], src_128[ix16 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[jx64 + 18], src_128[ix16 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[jx64 + 19], src_128[ix16 + 3]);
			sum_16to19 = _mm_mullo_epi32(coef_128[jx64 + 20], src_128[ix16 + 4]);
			sum_20to23 = _mm_mullo_epi32(coef_128[jx64 + 21], src_128[ix16 + 5]);
			sum_24to27 = _mm_mullo_epi32(coef_128[jx64 + 22], src_128[ix16 + 6]);
			sum_28to31 = _mm_mullo_epi32(coef_128[jx64 + 23], src_128[ix16 + 7]);
			sum_32to35 = _mm_mullo_epi32(coef_128[jx64 + 24], src_128[ix16 + 8]);
			sum_36to39 = _mm_mullo_epi32(coef_128[jx64 + 25], src_128[ix16 + 9]);
			sum_40to43 = _mm_mullo_epi32(coef_128[jx64 + 26], src_128[ix16 + 10]);
			sum_44to47 = _mm_mullo_epi32(coef_128[jx64 + 27], src_128[ix16 + 11]);
			sum_48to51 = _mm_mullo_epi32(coef_128[jx64 + 28], src_128[ix16 + 12]);
			sum_52to55 = _mm_mullo_epi32(coef_128[jx64 + 29], src_128[ix16 + 13]);
			sum_56to59 = _mm_mullo_epi32(coef_128[jx64 + 30], src_128[ix16 + 14]);
			sum_60to63 = _mm_mullo_epi32(coef_128[jx64 + 31], src_128[ix16 + 15]);

			sum1 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum1 = _mm_add_epi32(sum1, sum_8to11);
			sum1 = _mm_add_epi32(sum1, sum_12to15);
			sum1 = _mm_add_epi32(sum1, sum_16to19);
			sum1 = _mm_add_epi32(sum1, sum_20to23);
			sum1 = _mm_add_epi32(sum1, sum_24to27);
			sum1 = _mm_add_epi32(sum1, sum_28to31);
			sum1 = _mm_add_epi32(sum1, sum_32to35);
			sum1 = _mm_add_epi32(sum1, sum_36to39);
			sum1 = _mm_add_epi32(sum1, sum_40to43);
			sum1 = _mm_add_epi32(sum1, sum_44to47);
			sum1 = _mm_add_epi32(sum1, sum_48to51);
			sum1 = _mm_add_epi32(sum1, sum_52to55);
			sum1 = _mm_add_epi32(sum1, sum_56to59);
			sum1 = _mm_add_epi32(sum1, sum_60to63);
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 8));
			sum1 = _mm_add_epi32(sum1, _mm_srli_si128(sum1, 4));

			// 2
			sum_0to3 = _mm_mullo_epi32(coef_128[jx64 + 32], src_128[ix16 + 0]);
			sum_4to7 = _mm_mullo_epi32(coef_128[jx64 + 33], src_128[ix16 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[jx64 + 34], src_128[ix16 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[jx64 + 35], src_128[ix16 + 3]);
			sum_16to19 = _mm_mullo_epi32(coef_128[jx64 + 36], src_128[ix16 + 4]);
			sum_20to23 = _mm_mullo_epi32(coef_128[jx64 + 37], src_128[ix16 + 5]);
			sum_24to27 = _mm_mullo_epi32(coef_128[jx64 + 38], src_128[ix16 + 6]);
			sum_28to31 = _mm_mullo_epi32(coef_128[jx64 + 39], src_128[ix16 + 7]);
			sum_32to35 = _mm_mullo_epi32(coef_128[jx64 + 40], src_128[ix16 + 8]);
			sum_36to39 = _mm_mullo_epi32(coef_128[jx64 + 41], src_128[ix16 + 9]);
			sum_40to43 = _mm_mullo_epi32(coef_128[jx64 + 42], src_128[ix16 + 10]);
			sum_44to47 = _mm_mullo_epi32(coef_128[jx64 + 43], src_128[ix16 + 11]);
			sum_48to51 = _mm_mullo_epi32(coef_128[jx64 + 44], src_128[ix16 + 12]);
			sum_52to55 = _mm_mullo_epi32(coef_128[jx64 + 45], src_128[ix16 + 13]);
			sum_56to59 = _mm_mullo_epi32(coef_128[jx64 + 46], src_128[ix16 + 14]);
			sum_60to63 = _mm_mullo_epi32(coef_128[jx64 + 47], src_128[ix16 + 15]);

			sum2 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum2 = _mm_add_epi32(sum2, sum_8to11);
			sum2 = _mm_add_epi32(sum2, sum_12to15);
			sum2 = _mm_add_epi32(sum2, sum_16to19);
			sum2 = _mm_add_epi32(sum2, sum_20to23);
			sum2 = _mm_add_epi32(sum2, sum_24to27);
			sum2 = _mm_add_epi32(sum2, sum_28to31);
			sum2 = _mm_add_epi32(sum2, sum_32to35);
			sum2 = _mm_add_epi32(sum2, sum_36to39);
			sum2 = _mm_add_epi32(sum2, sum_40to43);
			sum2 = _mm_add_epi32(sum2, sum_44to47);
			sum2 = _mm_add_epi32(sum2, sum_48to51);
			sum2 = _mm_add_epi32(sum2, sum_52to55);
			sum2 = _mm_add_epi32(sum2, sum_56to59);
			sum2 = _mm_add_epi32(sum2, sum_60to63);
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 8));
			sum2 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

			// 3
			sum_0to3 = _mm_mullo_epi32(coef_128[jx64 + 48], src_128[ix16 + 0]);
			sum_4to7 = _mm_mullo_epi32(coef_128[jx64 + 49], src_128[ix16 + 1]);
			sum_8to11 = _mm_mullo_epi32(coef_128[jx64 + 50], src_128[ix16 + 2]);
			sum_12to15 = _mm_mullo_epi32(coef_128[jx64 + 51], src_128[ix16 + 3]);
			sum_16to19 = _mm_mullo_epi32(coef_128[jx64 + 52], src_128[ix16 + 4]);
			sum_20to23 = _mm_mullo_epi32(coef_128[jx64 + 53], src_128[ix16 + 5]);
			sum_24to27 = _mm_mullo_epi32(coef_128[jx64 + 54], src_128[ix16 + 6]);
			sum_28to31 = _mm_mullo_epi32(coef_128[jx64 + 55], src_128[ix16 + 7]);
			sum_32to35 = _mm_mullo_epi32(coef_128[jx64 + 56], src_128[ix16 + 8]);
			sum_36to39 = _mm_mullo_epi32(coef_128[jx64 + 57], src_128[ix16 + 9]);
			sum_40to43 = _mm_mullo_epi32(coef_128[jx64 + 58], src_128[ix16 + 10]);
			sum_44to47 = _mm_mullo_epi32(coef_128[jx64 + 59], src_128[ix16 + 11]);
			sum_48to51 = _mm_mullo_epi32(coef_128[jx64 + 60], src_128[ix16 + 12]);
			sum_52to55 = _mm_mullo_epi32(coef_128[jx64 + 61], src_128[ix16 + 13]);
			sum_56to59 = _mm_mullo_epi32(coef_128[jx64 + 62], src_128[ix16 + 14]);
			sum_60to63 = _mm_mullo_epi32(coef_128[jx64 + 63], src_128[ix16 + 15]);

			sum3 = _mm_add_epi32(sum_0to3, sum_4to7);
			sum3 = _mm_add_epi32(sum3, sum_8to11);
			sum3 = _mm_add_epi32(sum3, sum_12to15);
			sum3 = _mm_add_epi32(sum3, sum_16to19);
			sum3 = _mm_add_epi32(sum3, sum_20to23);
			sum3 = _mm_add_epi32(sum3, sum_24to27);
			sum3 = _mm_add_epi32(sum3, sum_28to31);
			sum3 = _mm_add_epi32(sum3, sum_32to35);
			sum3 = _mm_add_epi32(sum3, sum_36to39);
			sum3 = _mm_add_epi32(sum3, sum_40to43);
			sum3 = _mm_add_epi32(sum3, sum_44to47);
			sum3 = _mm_add_epi32(sum3, sum_48to51);
			sum3 = _mm_add_epi32(sum3, sum_52to55);
			sum3 = _mm_add_epi32(sum3, sum_56to59);
			sum3 = _mm_add_epi32(sum3, sum_60to63);
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 8));
			sum3 = _mm_add_epi32(sum3, _mm_srli_si128(sum3, 4));

			// Sum mul of result 0 1 2 3
			sum0 = _mm_unpacklo_epi32(sum0, sum1);
			sum2 = _mm_unpacklo_epi32(sum2, sum3);
			allSum = _mm_unpacklo_epi64(sum0, sum2);
			tmpDst = _mm_srai_epi32(_mm_add_epi32(allSum, factor), shift);

			//clip
			tmpDst = _mm_min_epi32(tmpDst, max_val);
			tmpDst = _mm_max_epi32(tmpDst, min_val);

			_mm_storeu_si128((__m128i*)&dst[i * 64 + j * 4], tmpDst);
		}
	}

}

#endif	// #if x86_SSE_128


/** 8x8 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
void fastForwardDCT2_B8( const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use )
{
  Int j, k;
  TCoeff E[4], O[4];
  TCoeff EE[2], EO[2];
  TCoeff add = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr8[DCT2][0] : g_aiT8[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const Int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* E and O*/
    for( k = 0; k < 4; k++ )
    {
      E[k] = src[k] + src[7 - k];
      O[k] = src[k] - src[7 - k];
    }
    /* EE and EO */
    EE[0] = E[0] + E[3];
    EO[0] = E[0] - E[3];
    EE[1] = E[1] + E[2];
    EO[1] = E[1] - E[2];

    dst[0       ] = (iT[ 0] * EE[0] + iT[ 1] * EE[1] + add) >> shift;
    dst[4 * line] = (iT[32] * EE[0] + iT[33] * EE[1] + add) >> shift;
    dst[2 * line] = (iT[16] * EO[0] + iT[17] * EO[1] + add) >> shift;
    dst[6 * line] = (iT[48] * EO[0] + iT[49] * EO[1] + add) >> shift;

    dst[    line] = (iT[ 8] * O[0] + iT[ 9] * O[1] + iT[10] * O[2] + iT[11] * O[3] + add) >> shift;
    dst[3 * line] = (iT[24] * O[0] + iT[25] * O[1] + iT[26] * O[2] + iT[27] * O[3] + add) >> shift;
    dst[5 * line] = (iT[40] * O[0] + iT[41] * O[1] + iT[42] * O[2] + iT[43] * O[3] + add) >> shift;
    dst[7 * line] = (iT[56] * O[0] + iT[57] * O[1] + iT[58] * O[2] + iT[59] * O[3] + add) >> shift;

    src += 8;
    dst++;
  }
  if( iSkipLine )
  {
    dst = pCoef + reducedLine;
    for( j = 0; j < 8; j++ )
    {
      memset( dst, 0, sizeof( TCoeff )*iSkipLine );
      dst += line;
    }
  }
}

/** 8x8 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B8(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j, k;
  Int E[4], O[4];
  Int EE[2], EO[2];
  Int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr8[DCT2][0] : g_aiT8[TRANSFORM_INVERSE][0];

  const Int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for( k = 0; k < 4; k++ )
    {
      O[k] = iT[1 * 8 + k] * src[line] + iT[3 * 8 + k] * src[3 * line] + iT[5 * 8 + k] * src[5 * line] + iT[7 * 8 + k] * src[7 * line];
    }

    EO[0] = iT[2 * 8 + 0] * src[2 * line] + iT[6 * 8 + 0] * src[6 * line];
    EO[1] = iT[2 * 8 + 1] * src[2 * line] + iT[6 * 8 + 1] * src[6 * line];
    EE[0] = iT[0 * 8 + 0] * src[0       ] + iT[4 * 8 + 0] * src[4 * line];
    EE[1] = iT[0 * 8 + 1] * src[0       ] + iT[4 * 8 + 1] * src[4 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];

    for( k = 0; k < 4; k++ )
    {
      dst[k    ] = Clip3( outputMinimum, outputMaximum, ( E[    k] + O[    k] + add ) >> shift );
      dst[k + 4] = Clip3( outputMinimum, outputMaximum, ( E[3 - k] - O[3 - k] + add ) >> shift );
    }
    src++;
    dst += 8;
  }
  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 3 ) * sizeof( TCoeff ) );
  }
}


/** 16x16 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
void fastForwardDCT2_B16(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use)
{
  Int j, k;
  TCoeff E  [8], O  [8];
  TCoeff EE [4], EO [4];
  TCoeff EEE[2], EEO[2];
  TCoeff add = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr16[DCT2][0] : g_aiT16[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const Int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* E and O*/
    for( k = 0; k < 8; k++ )
    {
      E[k] = src[k] + src[15 - k];
      O[k] = src[k] - src[15 - k];
    }
    /* EE and EO */
    for( k = 0; k < 4; k++ )
    {
      EE[k] = E[k] + E[7 - k];
      EO[k] = E[k] - E[7 - k];
    }
    /* EEE and EEO */
    EEE[0] = EE[0] + EE[3];
    EEO[0] = EE[0] - EE[3];
    EEE[1] = EE[1] + EE[2];
    EEO[1] = EE[1] - EE[2];

    dst[ 0       ] = ( iT[ 0     ] * EEE[0] + iT[          1] * EEE[1] + add ) >> shift;
    dst[ 8 * line] = ( iT[ 8 * 16] * EEE[0] + iT[ 8 * 16 + 1] * EEE[1] + add ) >> shift;
    dst[ 4 * line] = ( iT[ 4 * 16] * EEO[0] + iT[ 4 * 16 + 1] * EEO[1] + add ) >> shift;
    dst[12 * line] = ( iT[12 * 16] * EEO[0] + iT[12 * 16 + 1] * EEO[1] + add ) >> shift;

    for( k = 2; k < 16; k += 4 )
    {
      dst[k*line] = ( iT[k * 16] * EO[0] + iT[k * 16 + 1] * EO[1] + iT[k * 16 + 2] * EO[2] + iT[k * 16 + 3] * EO[3] + add ) >> shift;
    }

    for( k = 1; k < 16; k += 2 )
    {
      dst[k*line] = ( iT[k * 16    ] * O[0] + iT[k * 16 + 1] * O[1] + iT[k * 16 + 2] * O[2] + iT[k * 16 + 3] * O[3] +
                      iT[k * 16 + 4] * O[4] + iT[k * 16 + 5] * O[5] + iT[k * 16 + 6] * O[6] + iT[k * 16 + 7] * O[7] + add ) >> shift;
    }

    src += 16;
    dst++;

  }
  if( iSkipLine )
  {
    dst = pCoef + reducedLine;
    for( j = 0; j < 16; j++ )
    {
      memset( dst, 0, sizeof( TCoeff )*iSkipLine );
      dst += line;
    }
  }
}

/** 16x16 inverse transform implemented using partial butterfly structure (1D)
*  \param src            input data (transform coefficients)
*  \param dst            output data (residual)
*  \param shift          specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B16( const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  Int j, k;
  Int E  [8], O  [8];
  Int EE [4], EO [4];
  Int EEE[2], EEO[2];
  Int add = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr16[DCT2][0] : g_aiT16[TRANSFORM_INVERSE][0];

  const Int  reducedLine = line - iSkipLine;

  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for( k = 0; k < 8; k++ )
    {
      O[k] = iT[1 * 16 + k] * src[    line] + iT[ 3 * 16 + k] * src[ 3 * line] + iT[ 5 * 16 + k] * src[ 5 * line] + iT[ 7 * 16 + k] * src[ 7 * line] +
        iT[9 * 16 + k] * src[9 * line] + iT[11 * 16 + k] * src[11 * line] + iT[13 * 16 + k] * src[13 * line] + iT[15 * 16 + k] * src[15 * line];
    }
    for( k = 0; k < 4; k++ )
    {
      EO[k] = iT[2 * 16 + k] * src[2 * line] + iT[6 * 16 + k] * src[6 * line] + iT[10 * 16 + k] * src[10 * line] + iT[14 * 16 + k] * src[14 * line];
    }
    EEO[0] = iT[4 * 16    ] * src[4 * line] + iT[12 * 16    ] * src[12 * line];
    EEE[0] = iT[0         ] * src[0       ] + iT[ 8 * 16    ] * src[ 8 * line];
    EEO[1] = iT[4 * 16 + 1] * src[4 * line] + iT[12 * 16 + 1] * src[12 * line];
    EEE[1] = iT[0 * 16 + 1] * src[0       ] + iT[ 8 * 16 + 1] * src[ 8 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for( k = 0; k < 2; k++ )
    {
      EE[k    ] = EEE[    k] + EEO[    k];
      EE[k + 2] = EEE[1 - k] - EEO[1 - k];
    }
    for( k = 0; k < 4; k++ )
    {
      E[k    ] = EE[    k] + EO[    k];
      E[k + 4] = EE[3 - k] - EO[3 - k];
    }
    for( k = 0; k < 8; k++ )
    {
      dst[k    ] = Clip3( outputMinimum, outputMaximum, ( E[    k] + O[    k] + add ) >> shift );
      dst[k + 8] = Clip3( outputMinimum, outputMaximum, ( E[7 - k] - O[7 - k] + add ) >> shift );
    }
    src++;
    dst += 16;
  }
  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 4 ) * sizeof( TCoeff ) );
  }
}



/** 32x32 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
void fastForwardDCT2_B32( const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use )
{
  Int j, k;
  TCoeff E   [16], O   [16];
  TCoeff EE  [ 8], EO  [ 8];
  TCoeff EEE [ 4], EEO [ 4];
  TCoeff EEEE[ 2], EEEO[ 2];
  TCoeff add = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr32[DCT2][0] : g_aiT32[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const Int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* E and O*/
    for (k = 0;k<16;k++)
    {
      E[k] = src[k] + src[31 - k];
      O[k] = src[k] - src[31 - k];
    }
    /* EE and EO */
    for (k = 0;k<8;k++)
    {
      EE[k] = E[k] + E[15 - k];
      EO[k] = E[k] - E[15 - k];
    }
    /* EEE and EEO */
    for (k = 0;k<4;k++)
    {
      EEE[k] = EE[k] + EE[7 - k];
      EEO[k] = EE[k] - EE[7 - k];
    }
    /* EEEE and EEEO */
    EEEE[0] = EEE[0] + EEE[3];
    EEEO[0] = EEE[0] - EEE[3];
    EEEE[1] = EEE[1] + EEE[2];
    EEEO[1] = EEE[1] - EEE[2];

    dst[0] = (iT[0 * 32 + 0] * EEEE[0] + iT[0 * 32 + 1] * EEEE[1] + add) >> shift;
    dst[16 * line] = (iT[16 * 32 + 0] * EEEE[0] + iT[16 * 32 + 1] * EEEE[1] + add) >> shift;
    dst[8 * line] = (iT[8 * 32 + 0] * EEEO[0] + iT[8 * 32 + 1] * EEEO[1] + add) >> shift;
    dst[24 * line] = (iT[24 * 32 + 0] * EEEO[0] + iT[24 * 32 + 1] * EEEO[1] + add) >> shift;
    for (k = 4;k<32;k += 8)
    {
      dst[k*line] = (iT[k * 32 + 0] * EEO[0] + iT[k * 32 + 1] * EEO[1] + iT[k * 32 + 2] * EEO[2] + iT[k * 32 + 3] * EEO[3] + add) >> shift;
    }
    for (k = 2;k<32;k += 4)
    {
      dst[k*line] = (iT[k * 32 + 0] * EO[0] + iT[k * 32 + 1] * EO[1] + iT[k * 32 + 2] * EO[2] + iT[k * 32 + 3] * EO[3] +
                      iT[k * 32 + 4] * EO[4] + iT[k * 32 + 5] * EO[5] + iT[k * 32 + 6] * EO[6] + iT[k * 32 + 7] * EO[7] + add) >> shift;
    }
    for (k = 1;k<32;k += 2)
    {
      dst[k*line] = (iT[k * 32 + 0] * O[0] + iT[k * 32 + 1] * O[1] + iT[k * 32 + 2] * O[2] + iT[k * 32 + 3] * O[3] +
                      iT[k * 32 + 4] * O[4] + iT[k * 32 + 5] * O[5] + iT[k * 32 + 6] * O[6] + iT[k * 32 + 7] * O[7] +
                      iT[k * 32 + 8] * O[8] + iT[k * 32 + 9] * O[9] + iT[k * 32 + 10] * O[10] + iT[k * 32 + 11] * O[11] +
                      iT[k * 32 + 12] * O[12] + iT[k * 32 + 13] * O[13] + iT[k * 32 + 14] * O[14] + iT[k * 32 + 15] * O[15] + add) >> shift;
    }
    src += 32;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j<32; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

/** 32x32 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B32(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{

  Int j, k;
  Int E[16], O[16];
  Int EE[8], EO[8];
  Int EEE[4], EEO[4];
  Int EEEE[2], EEEO[2];
  Int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = (use & 1) ? g_aiTr32[DCT2][0] : g_aiT32[TRANSFORM_INVERSE][0];

  const Int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k = 0;k<16;k++)
    {
      O[k] = iT[1 * 32 + k] * src[line] + iT[3 * 32 + k] * src[3 * line] + iT[5 * 32 + k] * src[5 * line] + iT[7 * 32 + k] * src[7 * line] +
        iT[9 * 32 + k] * src[9 * line] + iT[11 * 32 + k] * src[11 * line] + iT[13 * 32 + k] * src[13 * line] + iT[15 * 32 + k] * src[15 * line] +
        iT[17 * 32 + k] * src[17 * line] + iT[19 * 32 + k] * src[19 * line] + iT[21 * 32 + k] * src[21 * line] + iT[23 * 32 + k] * src[23 * line] +
        iT[25 * 32 + k] * src[25 * line] + iT[27 * 32 + k] * src[27 * line] + iT[29 * 32 + k] * src[29 * line] + iT[31 * 32 + k] * src[31 * line];
    }
    for (k = 0;k<8;k++)
    {
      EO[k] = iT[2 * 32 + k] * src[2 * line] + iT[6 * 32 + k] * src[6 * line] + iT[10 * 32 + k] * src[10 * line] + iT[14 * 32 + k] * src[14 * line] +
        iT[18 * 32 + k] * src[18 * line] + iT[22 * 32 + k] * src[22 * line] + iT[26 * 32 + k] * src[26 * line] + iT[30 * 32 + k] * src[30 * line];
    }
    for (k = 0;k<4;k++)
    {
      EEO[k] = iT[4 * 32 + k] * src[4 * line] + iT[12 * 32 + k] * src[12 * line] + iT[20 * 32 + k] * src[20 * line] + iT[28 * 32 + k] * src[28 * line];
    }
    EEEO[0] = iT[8 * 32 + 0] * src[8 * line] + iT[24 * 32 + 0] * src[24 * line];
    EEEO[1] = iT[8 * 32 + 1] * src[8 * line] + iT[24 * 32 + 1] * src[24 * line];
    EEEE[0] = iT[0 * 32 + 0] * src[0] + iT[16 * 32 + 0] * src[16 * line];
    EEEE[1] = iT[0 * 32 + 1] * src[0] + iT[16 * 32 + 1] * src[16 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EEE[0] = EEEE[0] + EEEO[0];
    EEE[3] = EEEE[0] - EEEO[0];
    EEE[1] = EEEE[1] + EEEO[1];
    EEE[2] = EEEE[1] - EEEO[1];
    for (k = 0;k<4;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 4] = EEE[3 - k] - EEO[3 - k];
    }
    for (k = 0;k<8;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 8] = EE[7 - k] - EO[7 - k];
    }
    for (k = 0;k<16;k++)
    {
      dst[k] = Clip3(outputMinimum, outputMaximum, (E[k] + O[k] + add) >> shift);
      dst[k + 16] = Clip3(outputMinimum, outputMaximum, (E[15 - k] - O[15 - k] + add) >> shift);
    }
    src++;
    dst += 32;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 5) * sizeof(TCoeff));
  }
}



void fastForwardDCT2_B64(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use)
{
  int rnd_factor = 1 << (shift - 1);

  const Int uiTrSize = 64;
  const TMatrixCoeff *iT = (use & 1) ? g_aiTr64[DCT2][0] : g_aiT64[0][0];

  int   j, k;
  TCoeff E[32], O[32];
  TCoeff EE[16], EO[16];
  TCoeff EEE[8], EEO[8];
  TCoeff EEEE[4], EEEO[4];
  TCoeff EEEEE[2], EEEEO[2];
  TCoeff *tmp = dst;

  //bool zo = iSkipLine2 >= 32;
  bool zo = iSkipLine2 != 0;
  for (j = 0; j<line - iSkipLine; j++)
  {
    /* E and O*/
    for (k = 0;k<32;k++)
    {
      E[k] = src[k] + src[63 - k];
      O[k] = src[k] - src[63 - k];
    }
    /* EE and EO */
    for (k = 0;k<16;k++)
    {
      EE[k] = E[k] + E[31 - k];
      EO[k] = E[k] - E[31 - k];
    }
    /* EEE and EEO */
    for (k = 0;k<8;k++)
    {
      EEE[k] = EE[k] + EE[15 - k];
      EEO[k] = EE[k] - EE[15 - k];
    }
    /* EEEE and EEEO */
    for (k = 0;k<4;k++)
    {
      EEEE[k] = EEE[k] + EEE[7 - k];
      EEEO[k] = EEE[k] - EEE[7 - k];
    }
    /* EEEEE and EEEEO */
    EEEEE[0] = EEEE[0] + EEEE[3];
    EEEEO[0] = EEEE[0] - EEEE[3];
    EEEEE[1] = EEEE[1] + EEEE[2];
    EEEEO[1] = EEEE[1] - EEEE[2];

    dst[0] = (iT[0 * 64 + 0] * EEEEE[0] + iT[0 * 64 + 1] * EEEEE[1] + rnd_factor) >> shift;
    dst[16 * line] = (iT[16 * 64 + 0] * EEEEO[0] + iT[16 * 64 + 1] * EEEEO[1] + rnd_factor) >> shift;

    if (!zo)
    {
      dst[32 * line] = (iT[32 * 64 + 0] * EEEEE[0] + iT[32 * 64 + 1] * EEEEE[1] + rnd_factor) >> shift;
      dst[48 * line] = (iT[48 * 64 + 0] * EEEEO[0] + iT[48 * 64 + 1] * EEEEO[1] + rnd_factor) >> shift;
    }
    for (k = 8;k<(zo ? 32 : 64);k += 16)
    {
      dst[k*line] = (iT[k * 64 + 0] * EEEO[0] + iT[k * 64 + 1] * EEEO[1] + iT[k * 64 + 2] * EEEO[2] + iT[k * 64 + 3] * EEEO[3] + rnd_factor) >> shift;
    }
    for (k = 4;k<(zo ? 32 : 64);k += 8)
    {
      dst[k*line] = (iT[k * 64 + 0] * EEO[0] + iT[k * 64 + 1] * EEO[1] + iT[k * 64 + 2] * EEO[2] + iT[k * 64 + 3] * EEO[3] +
                      iT[k * 64 + 4] * EEO[4] + iT[k * 64 + 5] * EEO[5] + iT[k * 64 + 6] * EEO[6] + iT[k * 64 + 7] * EEO[7] + rnd_factor) >> shift;
    }
    for (k = 2;k<(zo ? 32 : 64);k += 4)
    {
      dst[k*line] = (iT[k * 64 + 0] * EO[0] + iT[k * 64 + 1] * EO[1] + iT[k * 64 + 2] * EO[2] + iT[k * 64 + 3] * EO[3] +
                      iT[k * 64 + 4] * EO[4] + iT[k * 64 + 5] * EO[5] + iT[k * 64 + 6] * EO[6] + iT[k * 64 + 7] * EO[7] +
                      iT[k * 64 + 8] * EO[8] + iT[k * 64 + 9] * EO[9] + iT[k * 64 + 10] * EO[10] + iT[k * 64 + 11] * EO[11] +
                      iT[k * 64 + 12] * EO[12] + iT[k * 64 + 13] * EO[13] + iT[k * 64 + 14] * EO[14] + iT[k * 64 + 15] * EO[15] + rnd_factor) >> shift;
    }
    for (k = 1;k<(zo ? 32 : 64);k += 2)
    {
      dst[k*line] = (iT[k * 64 + 0] * O[0] + iT[k * 64 + 1] * O[1] + iT[k * 64 + 2] * O[2] + iT[k * 64 + 3] * O[3] +
                      iT[k * 64 + 4] * O[4] + iT[k * 64 + 5] * O[5] + iT[k * 64 + 6] * O[6] + iT[k * 64 + 7] * O[7] +
                      iT[k * 64 + 8] * O[8] + iT[k * 64 + 9] * O[9] + iT[k * 64 + 10] * O[10] + iT[k * 64 + 11] * O[11] +
                      iT[k * 64 + 12] * O[12] + iT[k * 64 + 13] * O[13] + iT[k * 64 + 14] * O[14] + iT[k * 64 + 15] * O[15] +
                      iT[k * 64 + 16] * O[16] + iT[k * 64 + 17] * O[17] + iT[k * 64 + 18] * O[18] + iT[k * 64 + 19] * O[19] +
                      iT[k * 64 + 20] * O[20] + iT[k * 64 + 21] * O[21] + iT[k * 64 + 22] * O[22] + iT[k * 64 + 23] * O[23] +
                      iT[k * 64 + 24] * O[24] + iT[k * 64 + 25] * O[25] + iT[k * 64 + 26] * O[26] + iT[k * 64 + 27] * O[27] +
                      iT[k * 64 + 28] * O[28] + iT[k * 64 + 29] * O[29] + iT[k * 64 + 30] * O[30] + iT[k * 64 + 31] * O[31] + rnd_factor) >> shift;
    }
    src += uiTrSize;
    dst++;
  }

  const Int  reducedLine = line - iSkipLine;
  const Int  cutoff = uiTrSize - iSkipLine2;
  if (iSkipLine)
  {
    dst = tmp + reducedLine;
    for (j = 0; j<cutoff; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
  if (iSkipLine2)
  {
    dst = tmp + line*cutoff;
    memset(dst, 0, sizeof(TCoeff)*line*iSkipLine2);
  }
}

void fastInverseDCT2_B64(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  int rnd_factor = 1 << (shift - 1);
  const int uiTrSize = 64;
  const TMatrixCoeff *iT = (use & 1) ? g_aiTr64[DCT2][0] : g_aiT64[TRANSFORM_INVERSE][0];

  int    j, k;
  TCoeff E[32], O[32];
  TCoeff EE[16], EO[16];
  TCoeff EEE[8], EEO[8];
  TCoeff EEEE[4], EEEO[4];
  TCoeff EEEEE[2], EEEEO[2];
  bool zo = iSkipLine2 >= 32;
  for (j = 0; j<line - iSkipLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k = 0;k<32;k++)
    {
      O[k] = iT[1 * 64 + k] * src[line] + iT[3 * 64 + k] * src[3 * line] + iT[5 * 64 + k] * src[5 * line] + iT[7 * 64 + k] * src[7 * line] +
        iT[9 * 64 + k] * src[9 * line] + iT[11 * 64 + k] * src[11 * line] + iT[13 * 64 + k] * src[13 * line] + iT[15 * 64 + k] * src[15 * line] +
        iT[17 * 64 + k] * src[17 * line] + iT[19 * 64 + k] * src[19 * line] + iT[21 * 64 + k] * src[21 * line] + iT[23 * 64 + k] * src[23 * line] +
        iT[25 * 64 + k] * src[25 * line] + iT[27 * 64 + k] * src[27 * line] + iT[29 * 64 + k] * src[29 * line] + iT[31 * 64 + k] * src[31 * line] +
        (zo ? 0 : (
        iT[33 * 64 + k] * src[33 * line] + iT[35 * 64 + k] * src[35 * line] + iT[37 * 64 + k] * src[37 * line] + iT[39 * 64 + k] * src[39 * line] +
        iT[41 * 64 + k] * src[41 * line] + iT[43 * 64 + k] * src[43 * line] + iT[45 * 64 + k] * src[45 * line] + iT[47 * 64 + k] * src[47 * line] +
        iT[49 * 64 + k] * src[49 * line] + iT[51 * 64 + k] * src[51 * line] + iT[53 * 64 + k] * src[53 * line] + iT[55 * 64 + k] * src[55 * line] +
        iT[57 * 64 + k] * src[57 * line] + iT[59 * 64 + k] * src[59 * line] + iT[61 * 64 + k] * src[61 * line] + iT[63 * 64 + k] * src[63 * line]));
    }
    for (k = 0;k<16;k++)
    {
      EO[k] = iT[2 * 64 + k] * src[2 * line] + iT[6 * 64 + k] * src[6 * line] + iT[10 * 64 + k] * src[10 * line] + iT[14 * 64 + k] * src[14 * line] +
        iT[18 * 64 + k] * src[18 * line] + iT[22 * 64 + k] * src[22 * line] + iT[26 * 64 + k] * src[26 * line] + iT[30 * 64 + k] * src[30 * line] +
        (zo ? 0 : (
        iT[34 * 64 + k] * src[34 * line] + iT[38 * 64 + k] * src[38 * line] + iT[42 * 64 + k] * src[42 * line] + iT[46 * 64 + k] * src[46 * line] +
        iT[50 * 64 + k] * src[50 * line] + iT[54 * 64 + k] * src[54 * line] + iT[58 * 64 + k] * src[58 * line] + iT[62 * 64 + k] * src[62 * line]));
    }
    for (k = 0;k<8;k++)
    {
      EEO[k] = iT[4 * 64 + k] * src[4 * line] + iT[12 * 64 + k] * src[12 * line] + iT[20 * 64 + k] * src[20 * line] + iT[28 * 64 + k] * src[28 * line] +
        (zo ? 0 : (
        iT[36 * 64 + k] * src[36 * line] + iT[44 * 64 + k] * src[44 * line] + iT[52 * 64 + k] * src[52 * line] + iT[60 * 64 + k] * src[60 * line]));
    }
    for (k = 0;k<4;k++)
    {
      EEEO[k] = iT[8 * 64 + k] * src[8 * line] + iT[24 * 64 + k] * src[24 * line] + (zo ? 0 : (iT[40 * 64 + k] * src[40 * line] + iT[56 * 64 + k] * src[56 * line]));
    }
    EEEEO[0] = iT[16 * 64 + 0] * src[16 * line] + (zo ? 0 : iT[48 * 64 + 0] * src[48 * line]);
    EEEEO[1] = iT[16 * 64 + 1] * src[16 * line] + (zo ? 0 : iT[48 * 64 + 1] * src[48 * line]);
    EEEEE[0] = iT[0 * 64 + 0] * src[0] + (zo ? 0 : iT[32 * 64 + 0] * src[32 * line]);
    EEEEE[1] = iT[0 * 64 + 1] * src[0] + (zo ? 0 : iT[32 * 64 + 1] * src[32 * line]);

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for (k = 0;k<2;k++)
    {
      EEEE[k] = EEEEE[k] + EEEEO[k];
      EEEE[k + 2] = EEEEE[1 - k] - EEEEO[1 - k];
    }
    for (k = 0;k<4;k++)
    {
      EEE[k] = EEEE[k] + EEEO[k];
      EEE[k + 4] = EEEE[3 - k] - EEEO[3 - k];
    }
    for (k = 0;k<8;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 8] = EEE[7 - k] - EEO[7 - k];
    }
    for (k = 0;k<16;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 16] = EE[15 - k] - EO[15 - k];
    }
    for (k = 0;k<32;k++)
    {
      dst[k] = Clip3(outputMinimum, outputMaximum, (E[k] + O[k] + rnd_factor) >> shift);
      dst[k + 32] = Clip3(outputMinimum, outputMaximum, (E[31 - k] - O[31 - k] + rnd_factor) >> shift);
    }
    src++;
    dst += uiTrSize;
  }

  memset(dst, 0, uiTrSize*iSkipLine * sizeof(TCoeff));
}




void fastForwardDCT2_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use)
{
  int    j, k;
  TCoeff E[64], O[64];
  TCoeff EE[32], EO[32];
  TCoeff EEE[16], EEO[16];
  TCoeff EEEE[8], EEEO[8];
  TCoeff EEEEE[4], EEEEO[4];
  TCoeff EEEEEE[2], EEEEEO[2];
  TCoeff add = 1 << (shift - 1);

  const TMatrixCoeff(*iT)[128] = (use & 1) ? g_aiTr128[DCT2] : g_aiT128[TRANSFORM_FORWARD];

  TCoeff* tmp = dst;
  for (j = 0; j<line - iSkipLine; j++)
  {
    /* E and O*/
    for (k = 0;k< 64;k++)
    {
      E[k] = src[k] + src[127 - k];
      O[k] = src[k] - src[127 - k];
    }
    /* EE and EO */
    for (k = 0;k< 32;k++)
    {
      EE[k] = E[k] + E[63 - k];
      EO[k] = E[k] - E[63 - k];
    }

    /* EEE and EEO */
    for (k = 0;k< 16;k++)
    {
      EEE[k] = EE[k] + EE[31 - k];
      EEO[k] = EE[k] - EE[31 - k];
    }

    /* EEEE and EEEO */
    for (k = 0; k< 8; k++)
    {
      EEEE[k] = EEE[k] + EEE[15 - k];
      EEEO[k] = EEE[k] - EEE[15 - k];
    }

    for (k = 0; k< 4; k++)
    {
      EEEEE[k] = EEEE[k] + EEEE[7 - k];
      EEEEO[k] = EEEE[k] - EEEE[7 - k];
    }

    for (k = 0; k< 2; k++)
    {
      EEEEEE[k] = EEEEE[k] + EEEEE[3 - k];
      EEEEEO[k] = EEEEE[k] - EEEEE[3 - k];
    }

    //0
    dst[0] = (iT[0][0] * EEEEEE[0]
               + iT[0][1] * EEEEEE[1]
               + add) >> shift;
    dst[64 * line] = (iT[64][0] * EEEEEE[0]
                       + iT[64][1] * EEEEEE[1]
                       + add) >> shift;

    //2
    for (k = 32;k<128;k += 64)
    {
      dst[k*line] = (iT[k][0] * EEEEEO[0]
                      + iT[k][1] * EEEEEO[1]
                      + add) >> shift;
    }

    //4
    for (k = 16;k<128;k += 32)
    {
      dst[k*line] =
        (iT[k][0] * EEEEO[0]
          + iT[k][1] * EEEEO[1]
          + iT[k][2] * EEEEO[2]
          + iT[k][3] * EEEEO[3]
          + add) >> shift;
    }

    //8
    for (k = 8;k<128;k += 16)
    {
      dst[k*line] =
        (iT[k][0] * EEEO[0]
          + iT[k][1] * EEEO[1]
          + iT[k][2] * EEEO[2]
          + iT[k][3] * EEEO[3]
          + iT[k][4] * EEEO[4]
          + iT[k][5] * EEEO[5]
          + iT[k][6] * EEEO[6]
          + iT[k][7] * EEEO[7]
          + add) >> shift;
    }

    //16
    for (k = 4;k<128;k += 8)
    {
      dst[k*line] =
        (iT[k][0] * EEO[0]
          + iT[k][1] * EEO[1]
          + iT[k][2] * EEO[2]
          + iT[k][3] * EEO[3]
          + iT[k][4] * EEO[4]
          + iT[k][5] * EEO[5]
          + iT[k][6] * EEO[6]
          + iT[k][7] * EEO[7]
          + iT[k][8] * EEO[8]
          + iT[k][9] * EEO[9]
          + iT[k][10] * EEO[10]
          + iT[k][11] * EEO[11]
          + iT[k][12] * EEO[12]
          + iT[k][13] * EEO[13]
          + iT[k][14] * EEO[14]
          + iT[k][15] * EEO[15]
          + add) >> shift;
    }


    //32
    for (k = 2;k<128;k += 4)
    {
      dst[k*line] = (iT[k][0] * EO[0]
                      + iT[k][1] * EO[1]
                      + iT[k][2] * EO[2]
                      + iT[k][3] * EO[3]
                      + iT[k][4] * EO[4]
                      + iT[k][5] * EO[5]
                      + iT[k][6] * EO[6]
                      + iT[k][7] * EO[7]
                      + iT[k][8] * EO[8]
                      + iT[k][9] * EO[9]
                      + iT[k][10] * EO[10]
                      + iT[k][11] * EO[11]
                      + iT[k][12] * EO[12]
                      + iT[k][13] * EO[13]
                      + iT[k][14] * EO[14]
                      + iT[k][15] * EO[15]
                      + iT[k][16] * EO[16]
                      + iT[k][17] * EO[17]
                      + iT[k][18] * EO[18]
                      + iT[k][19] * EO[19]
                      + iT[k][20] * EO[20]
                      + iT[k][21] * EO[21]
                      + iT[k][22] * EO[22]
                      + iT[k][23] * EO[23]
                      + iT[k][24] * EO[24]
                      + iT[k][25] * EO[25]
                      + iT[k][26] * EO[26]
                      + iT[k][27] * EO[27]
                      + iT[k][28] * EO[28]
                      + iT[k][29] * EO[29]
                      + iT[k][30] * EO[30]
                      + iT[k][31] * EO[31]
                      + add) >> shift;
    }

    //64
    for (k = 1;k<128;k += 2)
    {
      dst[k*line] = (iT[k][0] * O[0]
                      + iT[k][1] * O[1]
                      + iT[k][2] * O[2]
                      + iT[k][3] * O[3]
                      + iT[k][4] * O[4]
                      + iT[k][5] * O[5]
                      + iT[k][6] * O[6]
                      + iT[k][7] * O[7]
                      + iT[k][8] * O[8]
                      + iT[k][9] * O[9]
                      + iT[k][10] * O[10]
                      + iT[k][11] * O[11]
                      + iT[k][12] * O[12]
                      + iT[k][13] * O[13]
                      + iT[k][14] * O[14]
                      + iT[k][15] * O[15]
                      + iT[k][16] * O[16]
                      + iT[k][17] * O[17]
                      + iT[k][18] * O[18]
                      + iT[k][19] * O[19]
                      + iT[k][20] * O[20]
                      + iT[k][21] * O[21]
                      + iT[k][22] * O[22]
                      + iT[k][23] * O[23]
                      + iT[k][24] * O[24]
                      + iT[k][25] * O[25]
                      + iT[k][26] * O[26]
                      + iT[k][27] * O[27]
                      + iT[k][28] * O[28]
                      + iT[k][29] * O[29]
                      + iT[k][30] * O[30]
                      + iT[k][31] * O[31]

                      + iT[k][32] * O[32]
                      + iT[k][33] * O[33]
                      + iT[k][34] * O[34]
                      + iT[k][35] * O[35]
                      + iT[k][36] * O[36]
                      + iT[k][37] * O[37]
                      + iT[k][38] * O[38]
                      + iT[k][39] * O[39]
                      + iT[k][40] * O[40]
                      + iT[k][41] * O[41]
                      + iT[k][42] * O[42]
                      + iT[k][43] * O[43]
                      + iT[k][44] * O[44]
                      + iT[k][45] * O[45]
                      + iT[k][46] * O[46]
                      + iT[k][47] * O[47]
                      + iT[k][48] * O[48]
                      + iT[k][49] * O[49]
                      + iT[k][50] * O[50]
                      + iT[k][51] * O[51]
                      + iT[k][52] * O[52]
                      + iT[k][53] * O[53]
                      + iT[k][54] * O[54]
                      + iT[k][55] * O[55]
                      + iT[k][56] * O[56]
                      + iT[k][57] * O[57]
                      + iT[k][58] * O[58]
                      + iT[k][59] * O[59]
                      + iT[k][60] * O[60]
                      + iT[k][61] * O[61]
                      + iT[k][62] * O[62]
                      + iT[k][63] * O[63]
                      + add) >> shift;
    }
    src += 128;
    dst++;
  }

  const UInt uiTrSize = 128;
  const Int  reducedLine = line - iSkipLine;
  const Int  cutoff = uiTrSize - iSkipLine2;
  if (iSkipLine)
  {
    dst = tmp + reducedLine;
    for (j = 0; j<cutoff; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
  if (iSkipLine2)
  {
    dst = tmp + line*cutoff;
    memset(dst, 0, sizeof(TCoeff)*line*iSkipLine2);
  }
}

void fastInverseDCT2_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  int    j, k;
  TCoeff E[64], O[64];
  TCoeff EE[32], EO[32];
  TCoeff EEE[16], EEO[16];
  TCoeff EEEE[8], EEEO[8];
  TCoeff EEEEE[4], EEEEO[4];
  TCoeff EEEEEE[2], EEEEEO[2];
  TCoeff add = 1 << (shift - 1);

  const TMatrixCoeff(*iT)[128] = (use & 1) ? g_aiTr128[DCT2] : g_aiT128[TRANSFORM_INVERSE];

  bool c1 = iSkipLine2 >= 96;
  bool c2 = iSkipLine2 >= 64;
  bool c3 = iSkipLine2 >= 32;

  for (j = 0; j<line - iSkipLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    if (c1)
    {
      for (k = 0;k<64;k++) //+2
      {
        O[k] = iT[1][k] * src[line]
          + iT[3][k] * src[3 * line]
          + iT[5][k] * src[5 * line]
          + iT[7][k] * src[7 * line]
          + iT[9][k] * src[9 * line]
          + iT[11][k] * src[11 * line]
          + iT[13][k] * src[13 * line]
          + iT[15][k] * src[15 * line]
          + iT[17][k] * src[17 * line]
          + iT[19][k] * src[19 * line]
          + iT[21][k] * src[21 * line]
          + iT[23][k] * src[23 * line]
          + iT[25][k] * src[25 * line]
          + iT[27][k] * src[27 * line]
          + iT[29][k] * src[29 * line]
          + iT[31][k] * src[31 * line]
          ;
      }

      for (k = 0;k<32;k++) //+4
      {
        EO[k] = iT[2][k] * src[2 * line]
          + iT[6][k] * src[6 * line]
          + iT[10][k] * src[10 * line]
          + iT[14][k] * src[14 * line]
          + iT[18][k] * src[18 * line]
          + iT[22][k] * src[22 * line]
          + iT[26][k] * src[26 * line]
          + iT[30][k] * src[30 * line]
          ;
      }
    }
    else if (c2)
    {
      for (k = 0;k<64;k++) //+2
      {
        O[k] = iT[1][k] * src[line]
          + iT[3][k] * src[3 * line]
          + iT[5][k] * src[5 * line]
          + iT[7][k] * src[7 * line]
          + iT[9][k] * src[9 * line]
          + iT[11][k] * src[11 * line]
          + iT[13][k] * src[13 * line]
          + iT[15][k] * src[15 * line]
          + iT[17][k] * src[17 * line]
          + iT[19][k] * src[19 * line]
          + iT[21][k] * src[21 * line]
          + iT[23][k] * src[23 * line]
          + iT[25][k] * src[25 * line]
          + iT[27][k] * src[27 * line]
          + iT[29][k] * src[29 * line]
          + iT[31][k] * src[31 * line]
          + iT[33][k] * src[33 * line]
          + iT[35][k] * src[35 * line]
          + iT[37][k] * src[37 * line]
          + iT[39][k] * src[39 * line]
          + iT[41][k] * src[41 * line]
          + iT[43][k] * src[43 * line]
          + iT[45][k] * src[45 * line]
          + iT[47][k] * src[47 * line]
          + iT[49][k] * src[49 * line]
          + iT[51][k] * src[51 * line]
          + iT[53][k] * src[53 * line]
          + iT[55][k] * src[55 * line]
          + iT[57][k] * src[57 * line]
          + iT[59][k] * src[59 * line]
          + iT[61][k] * src[61 * line]
          + iT[63][k] * src[63 * line]
          ;
      }

      for (k = 0;k<32;k++) //+4
      {
        EO[k] = iT[2][k] * src[2 * line]
          + iT[6][k] * src[6 * line]
          + iT[10][k] * src[10 * line]
          + iT[14][k] * src[14 * line]
          + iT[18][k] * src[18 * line]
          + iT[22][k] * src[22 * line]
          + iT[26][k] * src[26 * line]
          + iT[30][k] * src[30 * line]
          + iT[34][k] * src[34 * line]
          + iT[38][k] * src[38 * line]
          + iT[42][k] * src[42 * line]
          + iT[46][k] * src[46 * line]
          + iT[50][k] * src[50 * line]
          + iT[54][k] * src[54 * line]
          + iT[58][k] * src[58 * line]
          + iT[62][k] * src[62 * line]
          ;
      }
    }
    else if (c3)
    {
      for (k = 0;k<64;k++) //+2
      {
        O[k] = iT[1][k] * src[line]
          + iT[3][k] * src[3 * line]
          + iT[5][k] * src[5 * line]
          + iT[7][k] * src[7 * line]
          + iT[9][k] * src[9 * line]
          + iT[11][k] * src[11 * line]
          + iT[13][k] * src[13 * line]
          + iT[15][k] * src[15 * line]
          + iT[17][k] * src[17 * line]
          + iT[19][k] * src[19 * line]
          + iT[21][k] * src[21 * line]
          + iT[23][k] * src[23 * line]
          + iT[25][k] * src[25 * line]
          + iT[27][k] * src[27 * line]
          + iT[29][k] * src[29 * line]
          + iT[31][k] * src[31 * line]
          + iT[33][k] * src[33 * line]
          + iT[35][k] * src[35 * line]
          + iT[37][k] * src[37 * line]
          + iT[39][k] * src[39 * line]
          + iT[41][k] * src[41 * line]
          + iT[43][k] * src[43 * line]
          + iT[45][k] * src[45 * line]
          + iT[47][k] * src[47 * line]
          + iT[49][k] * src[49 * line]
          + iT[51][k] * src[51 * line]
          + iT[53][k] * src[53 * line]
          + iT[55][k] * src[55 * line]
          + iT[57][k] * src[57 * line]
          + iT[59][k] * src[59 * line]
          + iT[61][k] * src[61 * line]
          + iT[63][k] * src[63 * line]
          + iT[65][k] * src[65 * line]
          + iT[67][k] * src[67 * line]
          + iT[69][k] * src[69 * line]
          + iT[71][k] * src[71 * line]
          + iT[73][k] * src[73 * line]
          + iT[75][k] * src[75 * line]
          + iT[77][k] * src[77 * line]
          + iT[79][k] * src[79 * line]
          + iT[81][k] * src[81 * line]
          + iT[83][k] * src[83 * line]
          + iT[85][k] * src[85 * line]
          + iT[87][k] * src[87 * line]
          + iT[89][k] * src[89 * line]
          + iT[91][k] * src[91 * line]
          + iT[93][k] * src[93 * line]
          + iT[95][k] * src[95 * line]
          ;
      }

      for (k = 0;k<32;k++) //+4
      {
        EO[k] = iT[2][k] * src[2 * line]
          + iT[6][k] * src[6 * line]
          + iT[10][k] * src[10 * line]
          + iT[14][k] * src[14 * line]
          + iT[18][k] * src[18 * line]
          + iT[22][k] * src[22 * line]
          + iT[26][k] * src[26 * line]
          + iT[30][k] * src[30 * line]
          + iT[34][k] * src[34 * line]
          + iT[38][k] * src[38 * line]
          + iT[42][k] * src[42 * line]
          + iT[46][k] * src[46 * line]
          + iT[50][k] * src[50 * line]
          + iT[54][k] * src[54 * line]
          + iT[58][k] * src[58 * line]
          + iT[62][k] * src[62 * line]
          + iT[66][k] * src[66 * line]
          + iT[70][k] * src[70 * line]
          + iT[74][k] * src[74 * line]
          + iT[78][k] * src[78 * line]
          + iT[82][k] * src[82 * line]
          + iT[86][k] * src[86 * line]
          + iT[90][k] * src[90 * line]
          + iT[94][k] * src[94 * line]
          ;
      }
    }
    else
    {
      for (k = 0;k<64;k++) //+2
      {
        O[k] =
          iT[1][k] * src[line]
          + iT[3][k] * src[3 * line]
          + iT[5][k] * src[5 * line]
          + iT[7][k] * src[7 * line]
          + iT[9][k] * src[9 * line]
          + iT[11][k] * src[11 * line]
          + iT[13][k] * src[13 * line]
          + iT[15][k] * src[15 * line]
          + iT[17][k] * src[17 * line]
          + iT[19][k] * src[19 * line]
          + iT[21][k] * src[21 * line]
          + iT[23][k] * src[23 * line]
          + iT[25][k] * src[25 * line]
          + iT[27][k] * src[27 * line]
          + iT[29][k] * src[29 * line]
          + iT[31][k] * src[31 * line]
          + iT[33][k] * src[33 * line]
          + iT[35][k] * src[35 * line]
          + iT[37][k] * src[37 * line]
          + iT[39][k] * src[39 * line]
          + iT[41][k] * src[41 * line]
          + iT[43][k] * src[43 * line]
          + iT[45][k] * src[45 * line]
          + iT[47][k] * src[47 * line]
          + iT[49][k] * src[49 * line]
          + iT[51][k] * src[51 * line]
          + iT[53][k] * src[53 * line]
          + iT[55][k] * src[55 * line]
          + iT[57][k] * src[57 * line]
          + iT[59][k] * src[59 * line]
          + iT[61][k] * src[61 * line]
          + iT[63][k] * src[63 * line]
          + iT[65][k] * src[65 * line]
          + iT[67][k] * src[67 * line]
          + iT[69][k] * src[69 * line]
          + iT[71][k] * src[71 * line]
          + iT[73][k] * src[73 * line]
          + iT[75][k] * src[75 * line]
          + iT[77][k] * src[77 * line]
          + iT[79][k] * src[79 * line]
          + iT[81][k] * src[81 * line]
          + iT[83][k] * src[83 * line]
          + iT[85][k] * src[85 * line]
          + iT[87][k] * src[87 * line]
          + iT[89][k] * src[89 * line]
          + iT[91][k] * src[91 * line]
          + iT[93][k] * src[93 * line]
          + iT[95][k] * src[95 * line]
          + iT[97][k] * src[97 * line]
          + iT[99][k] * src[99 * line]
          + iT[101][k] * src[101 * line]
          + iT[103][k] * src[103 * line]
          + iT[105][k] * src[105 * line]
          + iT[107][k] * src[107 * line]
          + iT[109][k] * src[109 * line]
          + iT[111][k] * src[111 * line]
          + iT[113][k] * src[113 * line]
          + iT[115][k] * src[115 * line]
          + iT[117][k] * src[117 * line]
          + iT[119][k] * src[119 * line]
          + iT[121][k] * src[121 * line]
          + iT[123][k] * src[123 * line]
          + iT[125][k] * src[125 * line]
          + iT[127][k] * src[127 * line]
          ;
      }

      for (k = 0;k<32;k++) //+4
      {
        EO[k] = iT[2][k] * src[2 * line]
          + iT[6][k] * src[6 * line]
          + iT[10][k] * src[10 * line]
          + iT[14][k] * src[14 * line]
          + iT[18][k] * src[18 * line]
          + iT[22][k] * src[22 * line]
          + iT[26][k] * src[26 * line]
          + iT[30][k] * src[30 * line]
          + iT[34][k] * src[34 * line]
          + iT[38][k] * src[38 * line]
          + iT[42][k] * src[42 * line]
          + iT[46][k] * src[46 * line]
          + iT[50][k] * src[50 * line]
          + iT[54][k] * src[54 * line]
          + iT[58][k] * src[58 * line]
          + iT[62][k] * src[62 * line]
          + iT[66][k] * src[66 * line]
          + iT[70][k] * src[70 * line]
          + iT[74][k] * src[74 * line]
          + iT[78][k] * src[78 * line]
          + iT[82][k] * src[82 * line]
          + iT[86][k] * src[86 * line]
          + iT[90][k] * src[90 * line]
          + iT[94][k] * src[94 * line]
          + iT[98][k] * src[98 * line]
          + iT[102][k] * src[102 * line]
          + iT[106][k] * src[106 * line]
          + iT[110][k] * src[110 * line]
          + iT[114][k] * src[114 * line]
          + iT[118][k] * src[118 * line]
          + iT[122][k] * src[122 * line]
          + iT[126][k] * src[126 * line]
          ;
      }
    }

    for (k = 0;k<16;k++) //+8
    {
      EEO[k] = iT[4][k] * src[4 * line]
        + iT[12][k] * src[12 * line]
        + iT[20][k] * src[20 * line]
        + iT[28][k] * src[28 * line]
        + iT[36][k] * src[36 * line]
        + iT[44][k] * src[44 * line]
        + iT[52][k] * src[52 * line]
        + iT[60][k] * src[60 * line]
        + iT[68][k] * src[68 * line]
        + iT[76][k] * src[76 * line]
        + iT[84][k] * src[84 * line]
        + iT[92][k] * src[92 * line]
        + iT[100][k] * src[100 * line]
        + iT[108][k] * src[108 * line]
        + iT[116][k] * src[116 * line]
        + iT[124][k] * src[124 * line]
        ;
    }

    for (k = 0;k<8;k++) //+16
    {
      EEEO[k] = iT[8][k] * src[8 * line]
        + iT[24][k] * src[24 * line]
        + iT[40][k] * src[40 * line]
        + iT[56][k] * src[56 * line]
        + iT[72][k] * src[72 * line]
        + iT[88][k] * src[88 * line]
        + iT[104][k] * src[104 * line]
        + iT[120][k] * src[120 * line]
        ;
    }


    for (k = 0; k< 4; k++) //+32
    {
      EEEEO[k] = iT[16][k] * src[16 * line]
        + iT[48][k] * src[48 * line]
        + iT[80][k] * src[80 * line]
        + iT[112][k] * src[112 * line]
        ;
    }

    for (k = 0; k< 2; k++) //+64
    {
      EEEEEO[k] = iT[32][k] * src[32 * line]
        + iT[96][k] * src[96 * line]
        ;
    }

    EEEEEE[0] = iT[0][0] * src[0] + iT[64][0] * src[64 * line];
    EEEEEE[1] = iT[0][1] * src[0] + iT[64][1] * src[64 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for (k = 0;k<2;k++)
    {
      EEEEE[k] = EEEEEE[k] + EEEEEO[k];
      EEEEE[k + 2] = EEEEEE[1 - k] - EEEEEO[1 - k];
    }

    for (k = 0;k<4;k++)
    {
      EEEE[k] = EEEEE[k] + EEEEO[k];
      EEEE[k + 4] = EEEEE[3 - k] - EEEEO[3 - k];
    }

    for (k = 0;k<8;k++)
    {
      EEE[k] = EEEE[k] + EEEO[k];
      EEE[k + 8] = EEEE[7 - k] - EEEO[7 - k];
    }

    for (k = 0;k<16;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 16] = EEE[15 - k] - EEO[15 - k];
    }

    for (k = 0;k<32;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 32] = EE[31 - k] - EO[31 - k];
    }

    for (k = 0;k<64;k++)
    {
      dst[k] = Clip3(outputMinimum, outputMaximum, (E[k] + O[k] + add) >> shift);
      dst[k + 64] = Clip3(outputMinimum, outputMaximum, (E[63 - k] - O[63 - k] + add) >> shift);
    }
    src++;
    dst += 128;
  }

  memset(dst, 0, 128 * iSkipLine * sizeof(TCoeff));
}

// ********************************** DST-VII **********************************
void fastForwardDST7_B4(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use)
{
  Int i;
  TCoeff rnd_factor = (shift > 0) ? (1 << (shift - 1)) : 0;

#if HEVC_USE_4x4_DSTVII
  const TMatrixCoeff *iT = (use & 1) ? g_aiTr4[DST7][0] : g_as_DST_MAT_4[TRANSFORM_FORWARD][0];
#else
  const TMatrixCoeff *iT = g_aiTr4[DST7][0];
#endif

  Int c[4];
  TCoeff *pCoeff = dst;
  const Int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0] + src[3];
    c[1] = src[1] + src[3];
    c[2] = src[0] - src[1];
    c[3] = iT[2] * src[2];

    dst[0 * line] = (iT[0] * c[0] + iT[1] * c[1] + c[3] + rnd_factor) >> shift;
    dst[1 * line] = (iT[2] * (src[0] + src[1] - src[3]) + rnd_factor) >> shift;
    dst[2 * line] = (iT[0] * c[2] + iT[1] * c[0] - c[3] + rnd_factor) >> shift;
    dst[3 * line] = (iT[1] * c[2] - iT[0] * c[1] + c[3] + rnd_factor) >> shift;

    src += 4;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoeff + reducedLine;
    for (i = 0; i<4; i++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

void fastInverseDST7_B4(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1 << (shift - 1)) : 0;

#if HEVC_USE_4x4_DSTVII
  const TMatrixCoeff *iT = (use & 1) ? g_aiTr4[DST7][0] : g_as_DST_MAT_4[TRANSFORM_INVERSE][0];
#else
  const TMatrixCoeff *iT = g_aiTr4[DST7][0];
#endif

  const Int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0 * line] + src[2 * line];
    c[1] = src[2 * line] + src[3 * line];
    c[2] = src[0 * line] - src[3 * line];
    c[3] = iT[2] * src[1 * line];

    dst[0] = Clip3(outputMinimum, outputMaximum, (iT[0] * c[0] + iT[1] * c[1] + c[3] + rnd_factor) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (iT[1] * c[2] - iT[0] * c[1] + c[3] + rnd_factor) >> shift);
    dst[2] = Clip3(outputMinimum, outputMaximum, (iT[2] * (src[0 * line] - src[2 * line] + src[3 * line]) + rnd_factor) >> shift);
    dst[3] = Clip3(outputMinimum, outputMaximum, (iT[1] * c[0] + iT[0] * c[2] - c[3] + rnd_factor) >> shift);

    dst += 4;
    src++;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 2) * sizeof(TCoeff));
  }
}

#if SEPARABLE_KLT
// 4x4
void fastForwardKLT_B4(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType)
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
/*
	if (4 == line) {
		_fastForwardMM_B4_10bit_sse128(src, dst, shift, line, iSkipLine, iSkipLine2, use, use ? g_aiKLT4HP[0] : g_aiKLT4[0]);
#if DEBUG_GSXU
		// debug by gsxu
		TCoeff *dst_anchor = (TCoeff*)alloca(line * line * sizeof(TCoeff));
		_fastForwardMM< 4 >(src, dst_anchor, shift, line, iSkipLine, iSkipLine2, use, use ? g_aiKLT4HP[0] : g_aiKLT4[0]);
		for (int i = 0; i < line; ++i) {
			for (int j = 0; j < line; ++j) {
				std::cout << (dst[i * line + j] - dst_anchor[i * line + j]) << " ";
			}
			std::cout << std::endl;
	}
#endif
	}
	else 
		_fastForwardMM< 4 >(src, dst, shift, line, iSkipLine, iSkipLine2, use, use ? g_aiKLT4HP[0] : g_aiKLT4[0]);
*/
	switch (line) {
	case 4: _fastForwardMM_B4_10bit_sse128<4>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 8: _fastForwardMM_B4_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 16: _fastForwardMM_B4_10bit_sse128<16>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 32: _fastForwardMM_B4_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 64: _fastForwardMM_B4_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 128: _fastForwardMM_B4_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	default: _fastForwardMM< 4 >(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]);
	}
#else
  _fastForwardMM< 4 >( src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0] );
#endif
}

void fastInverseKLT_B4(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
	switch (line) {
	case 4: _fastInverseMM_B4_10bit_sse128<4>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 8: _fastInverseMM_B4_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 16: _fastInverseMM_B4_10bit_sse128<16>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 32: _fastInverseMM_B4_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 64: _fastInverseMM_B4_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	case 128: _fastInverseMM_B4_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]); break;
	default: _fastInverseMM< 4 >(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0]);
}
#else
  _fastInverseMM< 4 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT4HP[bIntra][0] : g_aiKLT4[bIntra][0] );
#endif
}

// 8x8
void fastForwardKLT_B8(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType)
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
	switch (line) {
	case 4: _fastForwardMM_B8_10bit_sse128<4>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 8: _fastForwardMM_B8_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 16: _fastForwardMM_B8_10bit_sse128<16>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 32: _fastForwardMM_B8_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 64: _fastForwardMM_B8_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 128: _fastForwardMM_B8_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	default: _fastForwardMM< 8 >(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]);
	}
#else
  _fastForwardMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0] );
#endif
}

void fastInverseKLT_B8(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
	switch (line) {
	case 4: _fastInverseMM_B8_10bit_sse128<4>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 8: _fastInverseMM_B8_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 16: _fastInverseMM_B8_10bit_sse128<16>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 32: _fastInverseMM_B8_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 64: _fastInverseMM_B8_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	case 128: _fastInverseMM_B8_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]); break;
	default: _fastInverseMM< 8 >(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0]);
	}
#else
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT8HP[bIntra][0] : g_aiKLT8[bIntra][0] );
#endif
}

// 16x16
void fastForwardKLT_B16(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType)
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
	switch (line) {
	case 4: _fastForwardMM_B16_10bit_sse128<4>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 8: _fastForwardMM_B16_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 16: _fastForwardMM_B16_10bit_sse128<16>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 32: _fastForwardMM_B16_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 64: _fastForwardMM_B16_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 128: _fastForwardMM_B16_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	default: _fastForwardMM< 16 >(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]);
	}
#else
  _fastForwardMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0] );
#endif
}

void fastInverseKLT_B16(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
	switch (line) {
	case 4: _fastInverseMM_B16_10bit_sse128<4>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 8: _fastInverseMM_B16_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 16: _fastInverseMM_B16_10bit_sse128<16>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 32: _fastInverseMM_B16_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 64: _fastInverseMM_B16_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	case 128: _fastInverseMM_B16_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]); break;
	default: _fastInverseMM< 16 >(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0]);
	}
#else
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT16HP[bIntra][0] : g_aiKLT16[bIntra][0] );
#endif
}

// 32x32
void fastForwardKLT_B32(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType)
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
	switch (line) {
	case 8: _fastForwardMM_B32_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	case 16: _fastForwardMM_B32_10bit_sse128<16>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	case 32: _fastForwardMM_B32_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	case 64: _fastForwardMM_B32_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	case 128: _fastForwardMM_B32_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	// SSE can't speedup case 4x32
	default: _fastForwardMM< 32 >(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]);
	}
#else
  _fastForwardMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0] );
#endif
}

void fastInverseKLT_B32(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
	switch (line) {
	case 4: _fastInverseMM_B32_10bit_sse128<4>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	case 8: _fastInverseMM_B32_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	case 16: _fastInverseMM_B32_10bit_sse128<16>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	case 32: _fastInverseMM_B32_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	case 64: _fastInverseMM_B32_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	case 128: _fastInverseMM_B32_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]); break;
	default: _fastInverseMM< 32 >(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0]);
	}
#else
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT32HP[bIntra][0] : g_aiKLT32[bIntra][0] );
#endif
}


void fastForwardKLT_B64(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType )
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
	switch (line) {
	case 8: _fastForwardMM_B64_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	case 32: _fastForwardMM_B64_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	case 64: _fastForwardMM_B64_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	case 128: _fastForwardMM_B64_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	// SSE2 can't speedup 64x16 case and 64x4 case
	default: _fastForwardMM< 64 >(src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]);
	}
#else
  _fastForwardMM< 64 >( src, dst, shift, line, iSkipLine, iSkipLine2, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0] );
#endif
}

void fastInverseKLT_B64(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int iTransType, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int bIntra = iTransType >> 1;
  Int bHighPrec = iTransType & 1;
#if x86_SSE_128
	switch (line) {
	case 4: _fastInverseMM_B64_10bit_sse128<4>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	case 8: _fastInverseMM_B64_10bit_sse128<8>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	case 16: _fastInverseMM_B64_10bit_sse128<16>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	case 32: _fastInverseMM_B64_10bit_sse128<32>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	case 64: _fastInverseMM_B64_10bit_sse128<64>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	case 128: _fastInverseMM_B64_10bit_sse128<128>(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]); break;
	default: _fastInverseMM< 64 >(src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0]);
	}
#else
  _fastInverseMM< 64 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, bHighPrec ? g_aiKLT64HP[bIntra][0] : g_aiKLT64[bIntra][0] );
#endif
}

#endif