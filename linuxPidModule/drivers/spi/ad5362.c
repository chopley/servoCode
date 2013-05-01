


struct ad5362spi{
	struct spi_device	*spi;
	u8			*buffer;
	int			power;
	struct lcd_device	*ld;
};


static int __devinit ad5362_spi_probe(struct spi_device *spi)
{
    struct ad5362spi *ad5362_ptr;
    ad5362_ptr = kzalloc(sizeof(ad5362spi), GFP_KERNEL);
    if (!ad5362_ptr)
        return -ENOMEM;
   // spin_lock_init(&data->lock);
  //  dev_set_drvdata(&spi->dev, data);*/
    return 0;
}



static struct spi_driver ad5362_spi_driver = {
    .driver = {
        .name   = "ad5362_spi",
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
    },
    .probe  = ad5362_spi_probe,
    .remove = __devexit_p(ad5362_spi_remove),
};


spi_register_driver(&ad5362_spi_driver);

spi_unregister_driver(&ad5362_spi_driver);