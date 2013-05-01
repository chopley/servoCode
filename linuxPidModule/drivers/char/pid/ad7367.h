extern spinlock_t spilock;

unsigned int spi_out_ad7367(unsigned short value,int *output);
int spi_out_ad7367_no_dma(unsigned short value);
unsigned int spi_out_ad7367_bit_banging(unsigned short value,int *output);
int spi_bitbanging_ad7367(void);

