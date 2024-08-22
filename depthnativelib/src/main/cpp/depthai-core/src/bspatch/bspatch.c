/* Modified by Martin Peterlin from bsdiff-4.3 to be able to patch in memory */
/*-
 * Copyright 2003-2005 Colin Percival
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted providing that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "bspatch.h"

#include <bzlib.h>
#include <stdlib.h>
#include <string.h>

enum block_type { CONTROL_BLOCK = 0, DIFF_BLOCK, EXTRA_BLOCK, NUM_BLOCKS };

static int64_t offtin(const uint8_t* buf) {
    int64_t y;

    y = buf[7] & 0x7F;
    y = y * 256;
    y += buf[6];
    y = y * 256;
    y += buf[5];
    y = y * 256;
    y += buf[4];
    y = y * 256;
    y += buf[3];
    y = y * 256;
    y += buf[2];
    y = y * 256;
    y += buf[1];
    y = y * 256;
    y += buf[0];

    if(buf[7] & 0x80) y = -y;

    return y;
}

int64_t bspatch_mem_get_newsize(const uint8_t* patchfile_bin, const int64_t patchfile_size) {
    uint8_t header[32];

    /* Read header */
    if(patchfile_size < 32) return -1;
    memcpy(header, patchfile_bin, 32);

    /* Check for appropriate magic */
    if(memcmp(header, "BSDIFF40", 8) != 0) return -1;

    int64_t newsize = offtin(header + 24);
    if(newsize < 0) return -1;
    return newsize;
}

int bspatch_mem(const uint8_t* oldfile_bin, const int64_t oldfile_size, const uint8_t* patchfile_bin, const int64_t patchfile_size, uint8_t* newfile_bin) {
    int64_t oldsize = oldfile_size;
    int64_t newsize;
    int64_t bzctrllen, bzdatalen;
    uint8_t header[32];
    const uint8_t* old = oldfile_bin;
    uint8_t* new = newfile_bin;
    int64_t oldpos, newpos;
    int64_t ctrl[3];
    int64_t i;
    int64_t cpOffset, dpOffset, epOffset;
    int error = 0;

    /*
    File format:
            0	8	"BSDIFF40"
            8	8	X
            16	8	Y
            24	8	sizeof(newfile)
            32	X	bzip2(control block)
            32+X	Y	bzip2(diff block)
            32+X+Y	???	bzip2(extra block)
    with control block a set of triples (x,y,z) meaning "add x bytes
    from oldfile to x bytes from the diff block; copy y bytes from the
    extra block; seek forwards in oldfile by z bytes".
    */

    /* Read header */
    if(patchfile_size < 32) return -1;
    memcpy(header, patchfile_bin, 32);

    /* Check for appropriate magic */
    if(memcmp(header, "BSDIFF40", 8) != 0) return -1;

    /* Read lengths from header */
    bzctrllen = offtin(header + 8);
    bzdatalen = offtin(header + 16);
    newsize = offtin(header + 24);
    if((bzctrllen < 0) || (bzdatalen < 0) || (newsize < 0)) return -1;

    /* Close patch file and re-open it via libbzip2 at the right places */
    cpOffset = sizeof(header);
    dpOffset = sizeof(header) + bzctrllen;
    epOffset = sizeof(header) + bzctrllen + bzdatalen;

    /* Decompress control block, diff block and extra block */

    int64_t block_offset_bz2[NUM_BLOCKS] = {cpOffset, dpOffset, epOffset};
    int64_t block_size_bz2[NUM_BLOCKS] = {bzctrllen, bzdatalen, patchfile_size - epOffset};
    uint8_t* p_decompressed_block_original[NUM_BLOCKS] = {NULL, NULL, NULL};
    uint8_t* p_decompressed_block[NUM_BLOCKS] = {NULL, NULL, NULL};
    uint8_t* p_decompressed_block_end[NUM_BLOCKS] = {NULL, NULL, NULL};

    for(int i = 0; i < NUM_BLOCKS; i++) {
        unsigned int decompressed_size = (unsigned int)newsize * 4;
        p_decompressed_block_original[i] = malloc(decompressed_size);  // reserve enough memory

        int ret = 0;
        if((ret = BZ2_bzBuffToBuffDecompress(
                (char*)p_decompressed_block_original[i], &decompressed_size, (char*)patchfile_bin + block_offset_bz2[i], (unsigned int)block_size_bz2[i], 0, 0))
           != BZ_OK) {
            (void)ret;
            error = -1;
            break;
        }

        // realloc pointer (and save if changes)
        p_decompressed_block_original[i] = realloc(p_decompressed_block_original[i], decompressed_size);  // resize memory to not waste additional space

        // Create current and end pointer
        p_decompressed_block[i] = p_decompressed_block_original[i];
        p_decompressed_block_end[i] = p_decompressed_block[i] + decompressed_size;
    }
    if(error) {
        for(int i = 0; i < NUM_BLOCKS; i++) {
            free(p_decompressed_block_original[i]);
        }
        return -1;
    }

    // Patch algorithm

    oldpos = 0;
    newpos = 0;
    while(newpos < newsize) {
        /* Read control data */
        for(i = 0; i <= 2; i++) {
            // if end of control data
            if(p_decompressed_block[CONTROL_BLOCK] + 8 > p_decompressed_block_end[CONTROL_BLOCK]) {
                error = -1;
                break;
            }

            // read from control block
            ctrl[i] = offtin(p_decompressed_block[CONTROL_BLOCK]);

            // move pointer forward
            p_decompressed_block[CONTROL_BLOCK] += 8;
        };
        if(error) break;

        /* Sanity-check */
        if(newpos + ctrl[0] > newsize) {
            for(int i = 0; i < NUM_BLOCKS; i++) {
                free(p_decompressed_block_original[i]);
            }
            return -1;
        }

        /* Read diff string */
        if(p_decompressed_block[DIFF_BLOCK] + ctrl[0] > p_decompressed_block_end[DIFF_BLOCK]) {
            error = -1;
            break;
        }
        memcpy(new + newpos, p_decompressed_block[DIFF_BLOCK], ctrl[0]);
        p_decompressed_block[DIFF_BLOCK] += ctrl[0];

        /* Add old data to diff string */
        for(i = 0; i < ctrl[0]; i++) {
            if((oldpos + i >= 0) && (oldpos + i < oldsize)) {
                new[newpos + i] += old[oldpos + i];
            }
        }

        /* Adjust pointers */
        newpos += ctrl[0];
        oldpos += ctrl[0];

        /* Sanity-check */
        if(newpos + ctrl[1] > newsize) {
            error = -1;
            break;
        }

        /* Read extra string */
        if(p_decompressed_block[EXTRA_BLOCK] + ctrl[1] > p_decompressed_block_end[EXTRA_BLOCK]) {
            for(int i = 0; i < NUM_BLOCKS; i++) {
                free(p_decompressed_block_original[i]);
            }
            return -1;
        }
        memcpy(new + newpos, p_decompressed_block[EXTRA_BLOCK], ctrl[1]);
        p_decompressed_block[EXTRA_BLOCK] += ctrl[1];

        /* Adjust pointers */
        newpos += ctrl[1];
        oldpos += ctrl[2];
    };

    /* Clean up the bzip2 reads */
    for(int i = 0; i < NUM_BLOCKS; i++) {
        free(p_decompressed_block_original[i]);
    }

    return error;
}
