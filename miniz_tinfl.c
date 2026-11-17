/**************************************************************************
 *
 * Copyright 2013-2014 RAD Game Tools and Valve Software
 * Copyright 2010-2014 Rich Geldreich and Tenacious Software LLC
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 **************************************************************************/

#include "miniz.h"

#ifndef MINIZ_NO_INFLATE_APIS

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

    /* ------------------- Low-level Decompression (completely independent from all compression API's) */

#define TINFL_MEMCPY(d, s, l) memcpy(d, s, l)
#define TINFL_MEMSET(p, c, l) memset(p, c, l)

#define TINFL_CR_BEGIN  \
    switch (r->m_state) \
    {                   \
        case 0:
#define TINFL_CR_RETURN(state_index, result) \
    do                                       \
    {                                        \
        status = result;                     \
        r->m_state = state_index;            \
        goto common_exit;                    \
        case state_index:;                   \
    }                                        \
    MZ_MACRO_END
#define TINFL_CR_RETURN_FOREVER(state_index, result) \
    do                                               \
    {                                                \
        for (;;)                                     \
        {                                            \
            TINFL_CR_RETURN(state_index, result);    \
        }                                            \
    }                                                \
    MZ_MACRO_END
#define TINFL_CR_FINISH }

#define TINFL_GET_BYTE(state_index, c)                                                                                                                           \
    do                                                                                                                                                           \
    {                                                                                                                                                            \
        while (pIn_buf_cur >= pIn_buf_end)                                                                                                                       \
        {                                                                                                                                                        \
            TINFL_CR_RETURN(state_index, (decomp_flags & TINFL_FLAG_HAS_MORE_INPUT) ? TINFL_STATUS_NEEDS_MORE_INPUT : TINFL_STATUS_FAILED_CANNOT_MAKE_PROGRESS); \
        }                                                                                                                                                        \
        c = *pIn_buf_cur++;                                                                                                                                      \
    }                                                                                                                                                            \
    MZ_MACRO_END

#define TINFL_NEED_BITS(state_index, n)                \
    do                                                 \
    {                                                  \
        mz_uint c;                                     \
        TINFL_GET_BYTE(state_index, c);                \
        bit_buf |= (((tinfl_bit_buf_t)c) << num_bits); \
        num_bits += 8;                                 \
    } while (num_bits < (mz_uint)(n))
#define TINFL_SKIP_BITS(state_index, n)      \
    do                                       \
    {                                        \
        if (num_bits < (mz_uint)(n))         \
        {                                    \
            TINFL_NEED_BITS(state_index, n); \
        }                                    \
        bit_buf >>= (n);                     \
        num_bits -= (n);                     \
    }                                        \
    MZ_MACRO_END
#define TINFL_GET_BITS(state_index, b, n)    \
    do                                       \
    {                                        \
        if (num_bits < (mz_uint)(n))         \
        {                                    \
            TINFL_NEED_BITS(state_index, n); \
        }                                    \
        b = bit_buf & ((1 << (n)) - 1);      \
        bit_buf >>= (n);                     \
        num_bits -= (n);                     \
    }                                        \
    MZ_MACRO_END

/* TINFL_HUFF_BITBUF_FILL() is only used rarely, when the number of bytes remaining in the input buffer falls below 2. */
/* It reads just enough bytes from the input stream that are needed to decode the next Huffman code (and absolutely no more). It works by trying to fully decode a */
/* Huffman code by using whatever bits are currently present in the bit buffer. If this fails, it reads another byte, and tries again until it succeeds or until the */
/* bit buffer contains >=15 bits (deflate's max. Huffman code size). */
#define TINFL_HUFF_BITBUF_FILL(state_index, pLookUp, pTree)          \
    do                                                               \
    {                                                                \
        temp = pLookUp[bit_buf & (TINFL_FAST_LOOKUP_SIZE - 1)];      \
        if (temp >= 0)                                               \
        {                                                            \
            code_len = temp >> 9;                                    \
            if ((code_len) && (num_bits >= code_len))                \
                break;                                               \
        }                                                            \
        else if (num_bits > TINFL_FAST_LOOKUP_BITS)                  \
        {                                                            \
            code_len = TINFL_FAST_LOOKUP_BITS;                       \
            do                                                       \
            {                                                        \
                temp = pTree[~temp + ((bit_buf >> code_len++) & 1)]; \
            } while ((temp < 0) && (num_bits >= (code_len + 1)));    \
            if (temp >= 0)                                           \
                break;                                               \
        }                                                            \
        TINFL_GET_BYTE(state_index, c);                              \
        bit_buf |= (((tinfl_bit_buf_t)c) << num_bits);               \
        num_bits += 8;                                               \
    } while (num_bits < 15);

/* TINFL_HUFF_DECODE() decodes the next Huffman coded symbol. It's more complex than you would initially expect because the zlib API expects the decompressor to never read */
/* beyond the final byte of the deflate stream. (In other words, when this macro wants to read another byte from the input, it REALLY needs another byte in order to fully */
/* decode the next Huffman code.) Handling this properly is particularly important on raw deflate (non-zlib) streams, which aren't followed by a byte aligned adler-32. */
/* The slow path is only executed at the very end of the input buffer. */
/* v1.16: The original macro handled the case at the very end of the passed-in input buffer, but we also need to handle the case where the user passes in 1+zillion bytes */
/* following the deflate data and our non-conservative read-ahead path won't kick in here on this code. This is much trickier. */
#define TINFL_HUFF_DECODE(state_index, sym, pLookUp, pTree)                                                                         \
    do                                                                                                                              \
    {                                                                                                                               \
        int temp;                                                                                                                   \
        mz_uint code_len, c;                                                                                                        \
        if (num_bits < 15)                                                                                                          \
        {                                                                                                                           \
            if ((pIn_buf_end - pIn_buf_cur) < 2)                                                                                    \
            {                                                                                                                       \
                TINFL_HUFF_BITBUF_FILL(state_index, pLookUp, pTree);                                                                \
            }                                                                                                                       \
            else                                                                                                                    \
            {                                                                                                                       \
                bit_buf |= (((tinfl_bit_buf_t)pIn_buf_cur[0]) << num_bits) | (((tinfl_bit_buf_t)pIn_buf_cur[1]) << (num_bits + 8)); \
                pIn_buf_cur += 2;                                                                                                   \
                num_bits += 16;                                                                                                     \
            }                                                                                                                       \
        }                                                                                                                           \
        if ((temp = pLookUp[bit_buf & (TINFL_FAST_LOOKUP_SIZE - 1)]) >= 0)                                                          \
            code_len = temp >> 9, temp &= 511;                                                                                      \
        else                                                                                                                        \
        {                                                                                                                           \
            code_len = TINFL_FAST_LOOKUP_BITS;                                                                                      \
            do                                                                                                                      \
            {                                                                                                                       \
                temp = pTree[~temp + ((bit_buf >> code_len++) & 1)];                                                                \
            } while (temp < 0);                                                                                                     \
        }                                                                                                                           \
        sym = temp;                                                                                                                 \
        bit_buf >>= code_len;                                                                                                       \
        num_bits -= code_len;                                                                                                       \
    }                                                                                                                               \
    MZ_MACRO_END

    /* s_tinfl_rev8[b] is the byte b with its 8 bits in reverse order. */
    static const mz_uint8 s_tinfl_rev8[256] = {
        0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
        0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
        0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
        0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
        0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
        0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
        0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
        0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
        0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
        0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
        0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
        0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
        0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
        0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
        0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
        0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
    };

/* The len low bits (1 <= len <= 15) of the code c (c < 2^len), in reverse order. */
#define TINFL_REV_CODE(c, len) ((((mz_uint)s_tinfl_rev8[(c)&0xFF] << 8) | s_tinfl_rev8[(c) >> 8]) >> (16 - (len)))

    /* Builds the decoding tables (pLookUp, TINFL_FAST_LOOKUP_SIZE entries, and pTree, tree_size bytes) of the
       Huffman code whose num_syms code lengths are in pCode_size. Returns 0, with both tables zeroed, if the
       lengths do not make a valid code. The tables are those the straightforward construction gives (clear both,
       then, symbol after symbol, fill the look-up table in steps of 1 << code_size), built with less work:
       - the symbols are first sorted by code length, into pTree, which is free until the long codes need it;
       - a code of at most TINFL_FAST_LOOKUP_BITS bits is written once, at its reversed code, in a table that
         holds 1 << len entries while the codes of len bits are written, and is then doubled by copying it onto
         its upper half: every entry is written exactly once more per doubling, by memcpy, and the entries no
         short code covers stay 0, as the long codes below expect, without clearing the whole table first;
       - the codes are reversed with a table of bytes, not bit by bit.
       The codes longer than TINFL_FAST_LOOKUP_BITS go into the tree in the order of the symbols, as before:
       the tree's nodes are numbered in that order. */
    static int tinfl_build_table(mz_int16 *pLookUp, mz_int16 *pTree, size_t tree_size, const mz_uint8 *pCode_size, mz_uint num_syms)
    {
        int tree_next, tree_cur;
        mz_uint i, j, len, used_syms, total, sym_index, next_code[17], total_syms[16], sorted_pos[16];
        const mz_int16 *pSorted;
        MZ_CLEAR_ARR(total_syms);
        for (i = 0; i < num_syms; ++i)
            total_syms[pCode_size[i]]++;
        used_syms = 0, total = 0;
        next_code[0] = next_code[1] = 0;
        sorted_pos[0] = 0;
        for (i = 1; i <= 15; ++i)
        {
            used_syms += total_syms[i];
            next_code[i + 1] = (total = ((total + total_syms[i]) << 1));
            sorted_pos[i] = sorted_pos[i - 1] + total_syms[i - 1];
        }
        if ((65536 != total) && (used_syms > 1))
        {
            TINFL_MEMSET(pLookUp, 0, sizeof(mz_int16) * TINFL_FAST_LOOKUP_SIZE);
            TINFL_MEMSET(pTree, 0, tree_size);
            return 0;
        }
        /* num_syms is at most half the entries of pTree: room for every symbol, the unused ones first. */
        for (sym_index = 0; sym_index < num_syms; ++sym_index)
            pTree[sorted_pos[pCode_size[sym_index]]++] = (mz_int16)sym_index;
        pSorted = pTree + total_syms[0];
        pLookUp[0] = pLookUp[1] = 0;
        for (len = 1;; ++len)
        {
            mz_uint cur_code = next_code[len], end_code = cur_code + total_syms[len];
            for (; cur_code < end_code; ++cur_code)
                pLookUp[TINFL_REV_CODE(cur_code, len)] = (mz_int16)((len << 9) | (mz_uint)*pSorted++);
            if (len == TINFL_FAST_LOOKUP_BITS)
                break;
            TINFL_MEMCPY(pLookUp + ((size_t)1 << len), pLookUp, sizeof(mz_int16) << len);
        }
        TINFL_MEMSET(pTree, 0, tree_size);
        if ((mz_uint)(pSorted - pTree) == num_syms)
            return 1;
        for (tree_next = -1, sym_index = 0; sym_index < num_syms; ++sym_index)
        {
            mz_uint rev_code, cur_code, code_size = pCode_size[sym_index];
            if (code_size <= TINFL_FAST_LOOKUP_BITS)
                continue;
            cur_code = next_code[code_size]++;
            rev_code = TINFL_REV_CODE(cur_code, code_size);
            if (0 == (tree_cur = pLookUp[rev_code & (TINFL_FAST_LOOKUP_SIZE - 1)]))
            {
                pLookUp[rev_code & (TINFL_FAST_LOOKUP_SIZE - 1)] = (mz_int16)tree_next;
                tree_cur = tree_next;
                tree_next -= 2;
            }
            rev_code >>= (TINFL_FAST_LOOKUP_BITS - 1);
            for (j = code_size; j > (TINFL_FAST_LOOKUP_BITS + 1); j--)
            {
                tree_cur -= ((rev_code >>= 1) & 1);
                if (!pTree[-tree_cur - 1])
                {
                    pTree[-tree_cur - 1] = (mz_int16)tree_next;
                    tree_cur = tree_next;
                    tree_next -= 2;
                }
                else
                    tree_cur = pTree[-tree_cur - 1];
            }
            tree_cur -= ((rev_code >>= 1) & 1);
            pTree[-tree_cur - 1] = (mz_int16)sym_index;
        }
        return 1;
    }

#if MINIZ_LITTLE_ENDIAN
#if defined(__GNUC__)
#define TINFL_NOINLINE __attribute__((__noinline__))
#elif defined(_MSC_VER)
#define TINFL_NOINLINE __declspec(noinline)
#else
#define TINFL_NOINLINE
#endif
    /* Writes the len bytes (len >= 4) of an overlapping match whose source starts dist bytes (1 to 7) before
       pOut: the output is the pattern of those dist bytes, repeated. Builds once, from byte loads, a 64-bit word
       holding the pattern repeated, and stores it in steps of the largest multiple of dist that fits in 8
       bytes, so that the same word is right at every step: nothing just stored is read back (no store-to-load
       forwarding stall), and nothing is written past the end of the match. Kept out of line so that
       tinfl_decompress's hot loop keeps its registers. */
    static TINFL_NOINLINE void tinfl_copy_overlap(mz_uint8 *pOut, size_t dist, size_t len)
    {
        static const mz_uint8 s_step[8] = { 0, 8, 8, 6, 8, 5, 6, 7 };
        const size_t step = s_step[dist];
        mz_uint8 *const pEnd = pOut + len;
        const mz_uint8 *pSrc = pOut - dist;
        size_t left;
        mz_uint64 v = pSrc[0];
        if (dist == 1)
            v *= 0x0101010101010101ULL;
        else
        {
            size_t i = 1;
            do
            {
                v |= (mz_uint64)pSrc[i] << (8 * i);
            } while (++i < dist);
            v |= v << (8 * dist);
            if (dist < 4)
                v |= v << (16 * dist);
        }
        while ((pEnd - pOut) >= 8)
        {
            memcpy(pOut, &v, 8);
            pOut += step;
        }
        /* The bytes left are the first ones of the word. */
        left = (size_t)(pEnd - pOut);
        if (left >= 4)
        {
            mz_uint32 w = (mz_uint32)v;
            memcpy(pOut, &w, 4);
            w = (mz_uint32)(v >> (8 * (left - 4)));
            memcpy(pEnd - 4, &w, 4);
        }
        else if (left)
        {
            pOut[0] = (mz_uint8)v;
            if (left > 1)
            {
                pOut[1] = (mz_uint8)(v >> 8);
                if (left > 2)
                    pOut[2] = (mz_uint8)(v >> 16);
            }
        }
    }
#endif /* MINIZ_LITTLE_ENDIAN */

    tinfl_status tinfl_decompress(tinfl_decompressor *r, const mz_uint8 *pIn_buf_next, size_t *pIn_buf_size, mz_uint8 *pOut_buf_start, mz_uint8 *pOut_buf_next, size_t *pOut_buf_size, const mz_uint32 decomp_flags)
    {
        static const mz_uint16 s_length_base[31] = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258, 0, 0 };
        static const mz_uint8 s_length_extra[31] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0, 0, 0 };
        static const mz_uint16 s_dist_base[32] = { 1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577, 0, 0 };
        static const mz_uint8 s_dist_extra[32] = { 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13 };
        static const mz_uint8 s_length_dezigzag[19] = { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };
        static const mz_uint16 s_min_table_sizes[3] = { 257, 1, 4 };

        mz_int16 *pTrees[3];
        mz_uint8 *pCode_sizes[3];

        tinfl_status status = TINFL_STATUS_FAILED;
        mz_uint32 num_bits, dist, counter, num_extra;
        tinfl_bit_buf_t bit_buf;
        const mz_uint8 *pIn_buf_cur = pIn_buf_next, *const pIn_buf_end = pIn_buf_next + *pIn_buf_size;
        mz_uint8 *pOut_buf_cur = pOut_buf_next, *const pOut_buf_end = pOut_buf_next ? pOut_buf_next + *pOut_buf_size : NULL;
        size_t out_buf_size_mask = (decomp_flags & TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF) ? (size_t)-1 : ((pOut_buf_next - pOut_buf_start) + *pOut_buf_size) - 1, dist_from_out_buf_start;

        /* Ensure the output buffer's size is a power of 2, unless the output buffer is large enough to hold the entire output file (in which case it doesn't matter). */
        if (((out_buf_size_mask + 1) & out_buf_size_mask) || (pOut_buf_next < pOut_buf_start))
        {
            *pIn_buf_size = *pOut_buf_size = 0;
            return TINFL_STATUS_BAD_PARAM;
        }

        pTrees[0] = r->m_tree_0;
        pTrees[1] = r->m_tree_1;
        pTrees[2] = r->m_tree_2;
        pCode_sizes[0] = r->m_code_size_0;
        pCode_sizes[1] = r->m_code_size_1;
        pCode_sizes[2] = r->m_code_size_2;

        num_bits = r->m_num_bits;
        bit_buf = r->m_bit_buf;
        dist = r->m_dist;
        counter = r->m_counter;
        num_extra = r->m_num_extra;
        dist_from_out_buf_start = r->m_dist_from_out_buf_start;
        TINFL_CR_BEGIN

        bit_buf = num_bits = dist = counter = num_extra = r->m_zhdr0 = r->m_zhdr1 = 0;
        r->m_z_adler32 = r->m_check_adler32 = 1;
        if (decomp_flags & TINFL_FLAG_PARSE_ZLIB_HEADER)
        {
            TINFL_GET_BYTE(1, r->m_zhdr0);
            TINFL_GET_BYTE(2, r->m_zhdr1);
            counter = (((r->m_zhdr0 * 256 + r->m_zhdr1) % 31 != 0) || (r->m_zhdr1 & 32) || ((r->m_zhdr0 & 15) != 8));
            if (!(decomp_flags & TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF))
                counter |= (((1U << (8U + (r->m_zhdr0 >> 4))) > 32768U) || ((out_buf_size_mask + 1) < (size_t)((size_t)1 << (8U + (r->m_zhdr0 >> 4)))));
            if (counter)
            {
                TINFL_CR_RETURN_FOREVER(36, TINFL_STATUS_FAILED);
            }
        }

        do
        {
            TINFL_GET_BITS(3, r->m_final, 3);
            r->m_type = r->m_final >> 1;
            if (r->m_type == 0)
            {
                TINFL_SKIP_BITS(5, num_bits & 7);
                for (counter = 0; counter < 4; ++counter)
                {
                    if (num_bits)
                        TINFL_GET_BITS(6, r->m_raw_header[counter], 8);
                    else
                        TINFL_GET_BYTE(7, r->m_raw_header[counter]);
                }
                if ((counter = (r->m_raw_header[0] | (r->m_raw_header[1] << 8))) != (mz_uint)(0xFFFF ^ (r->m_raw_header[2] | (r->m_raw_header[3] << 8))))
                {
                    TINFL_CR_RETURN_FOREVER(39, TINFL_STATUS_FAILED);
                }
                while ((counter) && (num_bits))
                {
                    TINFL_GET_BITS(51, dist, 8);
                    while (pOut_buf_cur >= pOut_buf_end)
                    {
                        TINFL_CR_RETURN(52, TINFL_STATUS_HAS_MORE_OUTPUT);
                    }
                    *pOut_buf_cur++ = (mz_uint8)dist;
                    counter--;
                }
                while (counter)
                {
                    size_t n;
                    while (pOut_buf_cur >= pOut_buf_end)
                    {
                        TINFL_CR_RETURN(9, TINFL_STATUS_HAS_MORE_OUTPUT);
                    }
                    while (pIn_buf_cur >= pIn_buf_end)
                    {
                        TINFL_CR_RETURN(38, (decomp_flags & TINFL_FLAG_HAS_MORE_INPUT) ? TINFL_STATUS_NEEDS_MORE_INPUT : TINFL_STATUS_FAILED_CANNOT_MAKE_PROGRESS);
                    }
                    n = MZ_MIN(MZ_MIN((size_t)(pOut_buf_end - pOut_buf_cur), (size_t)(pIn_buf_end - pIn_buf_cur)), counter);
                    TINFL_MEMCPY(pOut_buf_cur, pIn_buf_cur, n);
                    pIn_buf_cur += n;
                    pOut_buf_cur += n;
                    counter -= (mz_uint)n;
                }
            }
            else if (r->m_type == 3)
            {
                TINFL_CR_RETURN_FOREVER(10, TINFL_STATUS_FAILED);
            }
            else
            {
                if (r->m_type == 1)
                {
                    mz_uint8 *p = r->m_code_size_0;
                    mz_uint i;
                    r->m_table_sizes[0] = 288;
                    r->m_table_sizes[1] = 32;
                    TINFL_MEMSET(r->m_code_size_1, 5, 32);
                    for (i = 0; i <= 143; ++i)
                        *p++ = 8;
                    for (; i <= 255; ++i)
                        *p++ = 9;
                    for (; i <= 279; ++i)
                        *p++ = 7;
                    for (; i <= 287; ++i)
                        *p++ = 8;
                }
                else
                {
                    for (counter = 0; counter < 3; counter++)
                    {
                        TINFL_GET_BITS(11, r->m_table_sizes[counter], "\05\05\04"[counter]);
                        r->m_table_sizes[counter] += s_min_table_sizes[counter];
                    }
                    MZ_CLEAR_ARR(r->m_code_size_2);
                    for (counter = 0; counter < r->m_table_sizes[2]; counter++)
                    {
                        mz_uint s;
                        TINFL_GET_BITS(14, s, 3);
                        r->m_code_size_2[s_length_dezigzag[counter]] = (mz_uint8)s;
                    }
                    r->m_table_sizes[2] = 19;
                }
                for (; (int)r->m_type >= 0; r->m_type--)
                {
                    if (!tinfl_build_table(r->m_look_up[r->m_type], pTrees[r->m_type], (r->m_type == 0) ? sizeof(r->m_tree_0) : ((r->m_type == 1) ? sizeof(r->m_tree_1) : sizeof(r->m_tree_2)), pCode_sizes[r->m_type], r->m_table_sizes[r->m_type]))
                    {
                        TINFL_CR_RETURN_FOREVER(35, TINFL_STATUS_FAILED);
                    }
                    if (r->m_type == 2)
                    {
                        for (counter = 0; counter < (r->m_table_sizes[0] + r->m_table_sizes[1]);)
                        {
                            mz_uint s;
                            TINFL_HUFF_DECODE(16, dist, r->m_look_up[2], r->m_tree_2);
                            if (dist < 16)
                            {
                                r->m_len_codes[counter++] = (mz_uint8)dist;
                                continue;
                            }
                            if ((dist == 16) && (!counter))
                            {
                                TINFL_CR_RETURN_FOREVER(17, TINFL_STATUS_FAILED);
                            }
                            num_extra = "\02\03\07"[dist - 16];
                            TINFL_GET_BITS(18, s, num_extra);
                            s += "\03\03\013"[dist - 16];
                            TINFL_MEMSET(r->m_len_codes + counter, (dist == 16) ? r->m_len_codes[counter - 1] : 0, s);
                            counter += s;
                        }
                        if ((r->m_table_sizes[0] + r->m_table_sizes[1]) != counter)
                        {
                            TINFL_CR_RETURN_FOREVER(21, TINFL_STATUS_FAILED);
                        }
                        TINFL_MEMCPY(r->m_code_size_0, r->m_len_codes, r->m_table_sizes[0]);
                        TINFL_MEMCPY(r->m_code_size_1, r->m_len_codes + r->m_table_sizes[0], r->m_table_sizes[1]);
                    }
                }
                for (;;)
                {
                    mz_uint8 *pSrc;
                    for (;;)
                    {
                        if (((pIn_buf_end - pIn_buf_cur) < 4) || ((pOut_buf_end - pOut_buf_cur) < 2))
                        {
                            TINFL_HUFF_DECODE(23, counter, r->m_look_up[0], r->m_tree_0);
                            if (counter >= 256)
                                break;
                            while (pOut_buf_cur >= pOut_buf_end)
                            {
                                TINFL_CR_RETURN(24, TINFL_STATUS_HAS_MORE_OUTPUT);
                            }
                            *pOut_buf_cur++ = (mz_uint8)counter;
                        }
                        else
                        {
                            int sym2;
                            mz_uint code_len;
#if TINFL_USE_64BIT_BITBUF
                            if (num_bits < 30)
                            {
                                bit_buf |= (((tinfl_bit_buf_t)MZ_READ_LE32(pIn_buf_cur)) << num_bits);
                                pIn_buf_cur += 4;
                                num_bits += 32;
                            }
#else
                        if (num_bits < 15)
                        {
                            bit_buf |= (((tinfl_bit_buf_t)MZ_READ_LE16(pIn_buf_cur)) << num_bits);
                            pIn_buf_cur += 2;
                            num_bits += 16;
                        }
#endif /* TINFL_USE_64BIT_BITBUF */
                            if ((sym2 = r->m_look_up[0][bit_buf & (TINFL_FAST_LOOKUP_SIZE - 1)]) >= 0)
                                code_len = sym2 >> 9;
                            else
                            {
                                code_len = TINFL_FAST_LOOKUP_BITS;
                                do
                                {
                                    sym2 = r->m_tree_0[~sym2 + ((bit_buf >> code_len++) & 1)];
                                } while (sym2 < 0);
                            }
                            counter = sym2;
                            bit_buf >>= code_len;
                            num_bits -= code_len;
                            if (code_len == 0)
                            {
                                TINFL_CR_RETURN_FOREVER(40, TINFL_STATUS_FAILED);
                            }
                            if (counter & 256)
                                break;

#if !TINFL_USE_64BIT_BITBUF
                            if (num_bits < 15)
                            {
                                bit_buf |= (((tinfl_bit_buf_t)MZ_READ_LE16(pIn_buf_cur)) << num_bits);
                                pIn_buf_cur += 2;
                                num_bits += 16;
                            }
#endif /* !TINFL_USE_64BIT_BITBUF */
                            if ((sym2 = r->m_look_up[0][bit_buf & (TINFL_FAST_LOOKUP_SIZE - 1)]) >= 0)
                                code_len = sym2 >> 9;
                            else
                            {
                                code_len = TINFL_FAST_LOOKUP_BITS;
                                do
                                {
                                    sym2 = r->m_tree_0[~sym2 + ((bit_buf >> code_len++) & 1)];
                                } while (sym2 < 0);
                            }
                            bit_buf >>= code_len;
                            num_bits -= code_len;
                            if (code_len == 0)
                            {
                                TINFL_CR_RETURN_FOREVER(54, TINFL_STATUS_FAILED);
                            }

                            pOut_buf_cur[0] = (mz_uint8)counter;
                            if (sym2 & 256)
                            {
                                pOut_buf_cur++;
                                counter = sym2;
                                break;
                            }
                            pOut_buf_cur[1] = (mz_uint8)sym2;
                            pOut_buf_cur += 2;
                        }
                    }
                    if ((counter &= 511) == 256)
                        break;

                    num_extra = s_length_extra[counter - 257];
                    counter = s_length_base[counter - 257];
                    if (num_extra)
                    {
                        mz_uint extra_bits;
                        TINFL_GET_BITS(25, extra_bits, num_extra);
                        counter += extra_bits;
                    }

                    TINFL_HUFF_DECODE(26, dist, r->m_look_up[1], r->m_tree_1);
                    num_extra = s_dist_extra[dist];
                    dist = s_dist_base[dist];
                    if (num_extra)
                    {
                        mz_uint extra_bits;
                        TINFL_GET_BITS(27, extra_bits, num_extra);
                        dist += extra_bits;
                    }

                    dist_from_out_buf_start = pOut_buf_cur - pOut_buf_start;
                    if ((dist == 0 || dist > dist_from_out_buf_start || dist_from_out_buf_start == 0) && (decomp_flags & TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF))
                    {
                        TINFL_CR_RETURN_FOREVER(37, TINFL_STATUS_FAILED);
                    }

                    pSrc = pOut_buf_start + ((dist_from_out_buf_start - dist) & out_buf_size_mask);

                    if ((MZ_MAX(pOut_buf_cur, pSrc) + counter) > pOut_buf_end)
                    {
                        while (counter--)
                        {
                            while (pOut_buf_cur >= pOut_buf_end)
                            {
                                TINFL_CR_RETURN(53, TINFL_STATUS_HAS_MORE_OUTPUT);
                            }
                            *pOut_buf_cur++ = pOut_buf_start[(dist_from_out_buf_start++ - dist) & out_buf_size_mask];
                        }
                        continue;
                    }
#if MINIZ_USE_UNALIGNED_LOADS_AND_STORES
                    else if ((counter >= 9) && (counter <= dist))
                    {
                        const mz_uint8 *pSrc_end = pSrc + (counter & ~7);
                        do
                        {
#ifdef MINIZ_UNALIGNED_USE_MEMCPY
                            memcpy(pOut_buf_cur, pSrc, sizeof(mz_uint32) * 2);
#else
                            ((mz_uint32 *)pOut_buf_cur)[0] = ((const mz_uint32 *)pSrc)[0];
                            ((mz_uint32 *)pOut_buf_cur)[1] = ((const mz_uint32 *)pSrc)[1];
#endif /* MINIZ_UNALIGNED_USE_MEMCPY */
                            pOut_buf_cur += 8;
                        } while ((pSrc += 8) < pSrc_end);
                        if ((counter &= 7) < 3)
                        {
                            if (counter)
                            {
                                pOut_buf_cur[0] = pSrc[0];
                                if (counter > 1)
                                    pOut_buf_cur[1] = pSrc[1];
                                pOut_buf_cur += counter;
                            }
                            continue;
                        }
                    }
#endif /* MINIZ_USE_UNALIGNED_LOADS_AND_STORES */
                    /* Source entirely before the destination in memory (no overlap) and match of 4 bytes or more:
                       copy with 4- or 8-byte moves through a local variable (memcpy, no pointer casts), never
                       writing past the end of the match; the last move may overlap the previous one and rewrites
                       the same bytes. Overlapping matches keep the byte-by-byte loop below: wide loads of bytes
                       stored just before would stall on store-to-load forwarding. */
                    if ((counter >= 4) && ((pOut_buf_cur - pSrc) >= (ptrdiff_t)counter))
                    {
                        mz_uint8 *const pOut_match_end = pOut_buf_cur + counter;
                        if (counter <= 8)
                        {
                            mz_uint32 w;
                            memcpy(&w, pSrc, 4);
                            memcpy(pOut_buf_cur, &w, 4);
                            memcpy(&w, pSrc + counter - 4, 4);
                            memcpy(pOut_match_end - 4, &w, 4);
                        }
                        else
                        {
                            mz_uint64 v;
                            do
                            {
                                memcpy(&v, pSrc, 8);
                                memcpy(pOut_buf_cur, &v, 8);
                                pSrc += 8;
                                pOut_buf_cur += 8;
                            } while ((pOut_match_end - pOut_buf_cur) >= 8);
                            memcpy(&v, pSrc + ((pOut_match_end - pOut_buf_cur) - 8), 8);
                            memcpy(pOut_match_end - 8, &v, 8);
                        }
                        pOut_buf_cur = pOut_match_end;
                        counter %= 3; /* same leftover value as the byte-by-byte loop */
                        continue;
                    }
#if MINIZ_LITTLE_ENDIAN
                    /* Overlapping match whose source starts 1 to 7 bytes before the destination: copied by words,
                       out of line (see tinfl_copy_overlap). */
                    if ((counter > 3) && ((size_t)((pOut_buf_cur - pSrc) - 1) < 7))
                    {
                        tinfl_copy_overlap(pOut_buf_cur, (size_t)(pOut_buf_cur - pSrc), counter);
                        pOut_buf_cur += counter;
                        counter %= 3; /* same leftover value as the byte-by-byte loop */
                        continue;
                    }
#endif /* MINIZ_LITTLE_ENDIAN */
                    while (counter > 2)
                    {
                        pOut_buf_cur[0] = pSrc[0];
                        pOut_buf_cur[1] = pSrc[1];
                        pOut_buf_cur[2] = pSrc[2];
                        pOut_buf_cur += 3;
                        pSrc += 3;
                        counter -= 3;
                    }
                    if (counter > 0)
                    {
                        pOut_buf_cur[0] = pSrc[0];
                        if (counter > 1)
                            pOut_buf_cur[1] = pSrc[1];
                        pOut_buf_cur += counter;
                    }
                }
            }
        } while (!(r->m_final & 1));

        /* Ensure byte alignment and put back any bytes from the bitbuf if we've looked ahead too far on gzip, or other Deflate streams followed by arbitrary data. */
        /* I'm being super conservative here. A number of simplifications can be made to the byte alignment part, and the Adler32 check shouldn't ever need to worry about reading from the bitbuf now. */
        TINFL_SKIP_BITS(32, num_bits & 7);
        while ((pIn_buf_cur > pIn_buf_next) && (num_bits >= 8))
        {
            --pIn_buf_cur;
            num_bits -= 8;
        }
        bit_buf &= ~(~(tinfl_bit_buf_t)0 << num_bits);
        MZ_ASSERT(!num_bits); /* if this assert fires then we've read beyond the end of non-deflate/zlib streams with following data (such as gzip streams). */

        if (decomp_flags & TINFL_FLAG_PARSE_ZLIB_HEADER)
        {
            for (counter = 0; counter < 4; ++counter)
            {
                mz_uint s;
                if (num_bits)
                    TINFL_GET_BITS(41, s, 8);
                else
                    TINFL_GET_BYTE(42, s);
                r->m_z_adler32 = (r->m_z_adler32 << 8) | s;
            }
        }
        TINFL_CR_RETURN_FOREVER(34, TINFL_STATUS_DONE);

        TINFL_CR_FINISH

    common_exit:
        /* As long as we aren't telling the caller that we NEED more input to make forward progress: */
        /* Put back any bytes from the bitbuf in case we've looked ahead too far on gzip, or other Deflate streams followed by arbitrary data. */
        /* We need to be very careful here to NOT push back any bytes we definitely know we need to make forward progress, though, or we'll lock the caller up into an inf loop. */
        if ((status != TINFL_STATUS_NEEDS_MORE_INPUT) && (status != TINFL_STATUS_FAILED_CANNOT_MAKE_PROGRESS))
        {
            while ((pIn_buf_cur > pIn_buf_next) && (num_bits >= 8))
            {
                --pIn_buf_cur;
                num_bits -= 8;
            }
        }
        r->m_num_bits = num_bits;
        r->m_bit_buf = bit_buf & ~(~(tinfl_bit_buf_t)0 << num_bits);
        r->m_dist = dist;
        r->m_counter = counter;
        r->m_num_extra = num_extra;
        r->m_dist_from_out_buf_start = dist_from_out_buf_start;
        *pIn_buf_size = pIn_buf_cur - pIn_buf_next;
        *pOut_buf_size = pOut_buf_cur - pOut_buf_next;
        if ((decomp_flags & (TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_COMPUTE_ADLER32)) && (status >= 0))
        {
            const mz_uint8 *ptr = pOut_buf_next;
            size_t buf_len = *pOut_buf_size;
            mz_uint32 i, s1 = r->m_check_adler32 & 0xffff, s2 = r->m_check_adler32 >> 16;
            size_t block_len = buf_len % 5552;
            while (buf_len)
            {
                i = 0;
#if MINIZ_LITTLE_ENDIAN && MINIZ_HAS_64BIT_REGISTERS
                /* Groupes de 160 octets, 20 tours de 8 octets (tâche miniz-0039). Par voie de 16 bits, e somme les octets pairs, z
                   les mots décalés de 8 bits, pe et pz somment e et z avant chaque tour. Ensuite o = z - ((e >> 8) & ~0xFF) somme
                   les octets impairs (de même po) ; x = e + o. Sur 160 octets : s2 += 160 * s1 + 8 * (somme de pe et po) +
                   [8,6,4,2].e + [7,5,3,1].o, et [8,6,4,2].e + [7,5,3,1].o = H(e) + H(x) + 2 * [3,2,1,0].x (H : somme des voies).
                   Bornes par voie : e, o <= 20 * 255, pe, po <= 190 * 255 = 48450, x <= 10200, 3x0 + 2x1 + x2 <= 61200 : aucune
                   retenue entre voies ; q, en voies de 32 bits, <= 4 * 48450. Mêmes entiers que la boucle octet par octet. */
                for (; i + 159 < block_len; i += 160)
                {
                    mz_uint64 e = 0, z = 0, pe = 0, pz = 0, w0, w1, q, x;
                    int k;
                    for (k = 0; k < 10; ++k, ptr += 16)
                    {
                        memcpy(&w0, ptr, sizeof(w0));
                        memcpy(&w1, ptr + 8, sizeof(w1));
                        pe += e, pz += z, e += w0 & 0x00FF00FF00FF00FFULL, z += w0 >> 8;
                        pe += e, pz += z, e += w1 & 0x00FF00FF00FF00FFULL, z += w1 >> 8;
                    }
                    z -= (e >> 8) & ~(mz_uint64)0xFF, pz -= (pe >> 8) & ~(mz_uint64)0xFF;
                    q = (pe & 0x0000FFFF0000FFFFULL) + ((pe >> 16) & 0x0000FFFF0000FFFFULL) + (pz & 0x0000FFFF0000FFFFULL) + ((pz >> 16) & 0x0000FFFF0000FFFFULL);
                    x = e + z;
                    s2 += 160 * s1 + (((mz_uint32)q + (mz_uint32)(q >> 32)) << 3) + (mz_uint32)((e * 0x0001000100010001ULL) >> 48) + (mz_uint32)((x * 0x0001000100010001ULL) >> 48) + 2 * (mz_uint32)((x * 0x0003000200010000ULL) >> 48);
                    s1 += (mz_uint32)((x * 0x0001000100010001ULL) >> 48);
                }
                /* Adler-32 par blocs de 16 octets (tâche miniz-0016). Deux mots de 64 bits lus par memcpy ; dans chacun, les octets
                   pairs (e) et impairs (o) sont isolés dans des voies de 16 bits. Un produit par une constante donne en voie 3 la
                   somme des 4 voies (s1) ou leur somme pondérée (s2) : au plus 2 * 255 * 36 = 18360 par voie, donc aucune retenue entre
                   voies. Sur 16 octets, s1 += a0 + a1 et s2 += 16 * s1 + 8 * a0 + d0 + d1 : exactement la boucle octet par octet, donc
                   les mêmes entiers avant la réduction modulo (sous 2^32 sur 5552 octets, borne de zlib : 255n(n+1)/2 + (n+1)*65520). */
                for (; i + 15 < block_len; i += 16, ptr += 16)
                {
                    mz_uint64 w0, w1, e0, o0, e1, o1;
                    mz_uint32 a0, a1;
                    memcpy(&w0, ptr, sizeof(w0));
                    memcpy(&w1, ptr + 8, sizeof(w1));
                    e0 = w0 & 0x00FF00FF00FF00FFULL, o0 = (w0 >> 8) & 0x00FF00FF00FF00FFULL;
                    e1 = w1 & 0x00FF00FF00FF00FFULL, o1 = (w1 >> 8) & 0x00FF00FF00FF00FFULL;
                    a0 = (mz_uint32)(((e0 + o0) * 0x0001000100010001ULL) >> 48);
                    a1 = (mz_uint32)(((e1 + o1) * 0x0001000100010001ULL) >> 48);
                    s2 += (s1 << 4) + (a0 << 3) + (mz_uint32)((e0 * 0x0008000600040002ULL + o0 * 0x0007000500030001ULL + e1 * 0x0008000600040002ULL + o1 * 0x0007000500030001ULL) >> 48);
                    s1 += a0 + a1;
                }
#endif
                for (; i + 7 < block_len; i += 8, ptr += 8)
                {
                    s1 += ptr[0], s2 += s1;
                    s1 += ptr[1], s2 += s1;
                    s1 += ptr[2], s2 += s1;
                    s1 += ptr[3], s2 += s1;
                    s1 += ptr[4], s2 += s1;
                    s1 += ptr[5], s2 += s1;
                    s1 += ptr[6], s2 += s1;
                    s1 += ptr[7], s2 += s1;
                }
                for (; i < block_len; ++i)
                    s1 += *ptr++, s2 += s1;
                s1 %= 65521U, s2 %= 65521U;
                buf_len -= block_len;
                block_len = 5552;
            }
            r->m_check_adler32 = (s2 << 16) + s1;
            if ((status == TINFL_STATUS_DONE) && (decomp_flags & TINFL_FLAG_PARSE_ZLIB_HEADER) && (r->m_check_adler32 != r->m_z_adler32))
                status = TINFL_STATUS_ADLER32_MISMATCH;
        }
        return status;
    }

    /* Higher level helper functions. */
    void *tinfl_decompress_mem_to_heap(const void *pSrc_buf, size_t src_buf_len, size_t *pOut_len, int flags)
    {
        tinfl_decompressor decomp;
        void *pBuf = NULL, *pNew_buf;
        size_t src_buf_ofs = 0, out_buf_capacity = 0;
        *pOut_len = 0;
        tinfl_init(&decomp);
        for (;;)
        {
            size_t src_buf_size = src_buf_len - src_buf_ofs, dst_buf_size = out_buf_capacity - *pOut_len, new_out_buf_capacity;
            tinfl_status status = tinfl_decompress(&decomp, (const mz_uint8 *)pSrc_buf + src_buf_ofs, &src_buf_size, (mz_uint8 *)pBuf, pBuf ? (mz_uint8 *)pBuf + *pOut_len : NULL, &dst_buf_size,
                                                   (flags & ~TINFL_FLAG_HAS_MORE_INPUT) | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
            if ((status < 0) || (status == TINFL_STATUS_NEEDS_MORE_INPUT))
            {
                MZ_FREE(pBuf);
                *pOut_len = 0;
                return NULL;
            }
            src_buf_ofs += src_buf_size;
            *pOut_len += dst_buf_size;
            if (status == TINFL_STATUS_DONE)
                break;
            new_out_buf_capacity = out_buf_capacity * 2;
            if (new_out_buf_capacity < 128)
                new_out_buf_capacity = 128;
            pNew_buf = MZ_REALLOC(pBuf, new_out_buf_capacity);
            if (!pNew_buf)
            {
                MZ_FREE(pBuf);
                *pOut_len = 0;
                return NULL;
            }
            pBuf = pNew_buf;
            out_buf_capacity = new_out_buf_capacity;
        }
        return pBuf;
    }

    size_t tinfl_decompress_mem_to_mem(void *pOut_buf, size_t out_buf_len, const void *pSrc_buf, size_t src_buf_len, int flags)
    {
        tinfl_decompressor decomp;
        tinfl_status status;
        tinfl_init(&decomp);
        status = tinfl_decompress(&decomp, (const mz_uint8 *)pSrc_buf, &src_buf_len, (mz_uint8 *)pOut_buf, (mz_uint8 *)pOut_buf, &out_buf_len, (flags & ~TINFL_FLAG_HAS_MORE_INPUT) | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
        return (status != TINFL_STATUS_DONE) ? TINFL_DECOMPRESS_MEM_TO_MEM_FAILED : out_buf_len;
    }

    int tinfl_decompress_mem_to_callback(const void *pIn_buf, size_t *pIn_buf_size, tinfl_put_buf_func_ptr pPut_buf_func, void *pPut_buf_user, int flags)
    {
        int result = 0;
        tinfl_decompressor decomp;
        mz_uint8 *pDict = (mz_uint8 *)MZ_MALLOC(TINFL_LZ_DICT_SIZE);
        size_t in_buf_ofs = 0, dict_ofs = 0;
        if (!pDict)
            return TINFL_STATUS_FAILED;
        memset(pDict, 0, TINFL_LZ_DICT_SIZE);
        tinfl_init(&decomp);
        for (;;)
        {
            size_t in_buf_size = *pIn_buf_size - in_buf_ofs, dst_buf_size = TINFL_LZ_DICT_SIZE - dict_ofs;
            tinfl_status status = tinfl_decompress(&decomp, (const mz_uint8 *)pIn_buf + in_buf_ofs, &in_buf_size, pDict, pDict + dict_ofs, &dst_buf_size,
                                                   (flags & ~(TINFL_FLAG_HAS_MORE_INPUT | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF)));
            in_buf_ofs += in_buf_size;
            if ((dst_buf_size) && (!(*pPut_buf_func)(pDict + dict_ofs, (int)dst_buf_size, pPut_buf_user)))
                break;
            if (status != TINFL_STATUS_HAS_MORE_OUTPUT)
            {
                result = (status == TINFL_STATUS_DONE);
                break;
            }
            dict_ofs = (dict_ofs + dst_buf_size) & (TINFL_LZ_DICT_SIZE - 1);
        }
        MZ_FREE(pDict);
        *pIn_buf_size = in_buf_ofs;
        return result;
    }

#ifndef MINIZ_NO_MALLOC
    tinfl_decompressor *tinfl_decompressor_alloc(void)
    {
        tinfl_decompressor *pDecomp = (tinfl_decompressor *)MZ_MALLOC(sizeof(tinfl_decompressor));
        if (pDecomp)
            tinfl_init(pDecomp);
        return pDecomp;
    }

    void tinfl_decompressor_free(tinfl_decompressor *pDecomp)
    {
        MZ_FREE(pDecomp);
    }
#endif /* MINIZ_NO_MALLOC */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MINIZ_NO_INFLATE_APIS */
