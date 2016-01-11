void VM_init(unsigned ncore, unsigned psize);
unsigned int  VM_rd(unsigned pgidx, unsigned idx);
void VM_wr(unsigned pgidx, unsigned idx, unsigned int  val);
void VM_finish(unsigned *npg, unsigned *npo);

void bh_init(unsigned algo, unsigned psz);

void bh_insert(unsigned val);

unsigned bh_remove(void);