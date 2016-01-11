#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//#include <sys/queue.h>

#include "binHeap.h"

/**********************************************************************
* algo = 0:	classic array based.
* algo = 1:	ditto, but with index shifted one down to use index 0
* algo = 2:	VM aware, strict tree, but wasting to indicies per page
* algo = 3:	ditto, but put those to indicies per page to use
*/

static unsigned bh_psize;
static unsigned bh_shift;
static unsigned bh_mask;
static unsigned bh_hshift;
static unsigned bh_hmask;
static unsigned bh_half;
static unsigned bh_len;
static unsigned bh_algo;



static unsigned
bh_pg(unsigned idx)
{

	return (idx >> bh_shift);
}

static unsigned
bh_po(unsigned idx)
{

	return (idx & bh_mask);
}

static void
bh_bubble_up(unsigned idx, unsigned v)
{
	unsigned ip, pv;
	unsigned po, pg;

	while (idx > 1) {
		if (bh_algo < 2) {
			ip = idx / 2;
		}
		else if (bh_algo == 2) {
			pg = bh_pg(idx);
			po = bh_po(idx);
			if (pg > 0 && po < 4) {
				assert(po == 2 || po == 3);
				ip = ((pg - 1) >> bh_hshift) << bh_shift;
				ip += ((pg - 1) & bh_hmask) + bh_half;
			}
			else {
				ip = (idx & ~bh_mask) + po / 2;
			}
		}
		else if (bh_algo == 3) {
			po = bh_po(idx);
			if (idx < bh_psize || po > 3) {
				ip = (idx & ~bh_mask) | (po >> 1);
			}
			else if (po < 2) {
				ip = (idx - bh_psize) >> bh_shift;
				ip += (ip & ~bh_hmask);
				ip |= bh_psize / 2;
			}
			else {
				ip = idx - 2;
			}
		}
		else {
			ip = 0;
			assert(__LINE__);
		}

		
	}
}

static void
bh_bubble_down(unsigned idx, unsigned v)
{
	unsigned i1, i2, v1, v2;
	unsigned po, pg;

	while (idx < bh_len) {
		if (bh_algo < 2) {
			i1 = idx * 2;
			i2 = i1 + 1;
		}
		else if (bh_algo == 2) {
			pg = bh_pg(idx);
			po = bh_po(idx);
			if (po < bh_half) {
				i1 = (idx & ~bh_mask) + po * 2;
			}
			else {
				i1 = (pg << bh_hshift) + (po - bh_half) + 1;
				i1 <<= bh_shift;
				i1 += 2;
			}
			i2 = i1 + 1;
		}
		else if (bh_algo == 3) {
			if (idx > bh_mask && !(idx & (bh_mask - 1))) {
				/* first two elements in nonzero pages */
				i1 = i2 = idx + 2;
			}
			else if (idx & (bh_psize >> 1)) {
				/* Last row of page */
				i1 = (idx & ~bh_mask) >> 1;
				i1 |= idx & (bh_mask >> 1);
				i1 += 1;
				i1 <<= bh_shift;
				i2 = i1 + 1;
			}
			else {
				i1 = idx + (idx & bh_mask);
				i2 = i1 + 1;
			}
		}
		else {
			i1 = 0;
			i2 = i1 + 1;
			assert(__LINE__);
		}
		
}

void bh_init(unsigned algo, unsigned psz)
{
	unsigned u;

	/* Calculate the log2(psz) */
	assert((psz & (psz - 1)) == 0);	/* Must be power of two */
	for (u = 1; (1U << u) != psz; u++)
		;
	bh_shift = u;
	bh_mask = psz - 1;

	bh_half = psz / 2;
	bh_hshift = bh_shift - 1;
	bh_hmask = bh_mask >> 1;

	bh_len = 0;
	bh_algo = algo;
	bh_psize = psz;
}

void
bh_insert(unsigned val)
{

	bh_len++;
	if (bh_algo == 2) {
		if (bh_po(bh_len) == 0)
			bh_len += 2;
	}
	bh_wr(bh_len, val);
	bh_bubble_up(bh_len, val);
}

unsigned
bh_remove(void)
{
	unsigned val, retval;

	retval = bh_rd(1);
	val = bh_rd(bh_len);
	bh_len--;
	if (bh_len == 0)
		return (retval);
	if (bh_algo == 2) {
		if (bh_pg(bh_len) > 0 && bh_po(bh_len) == 1)
			bh_len -= 2;
	}
	bh_wr(1, val);
	bh_bubble_down(1, val);
	return (retval);
}