#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>



/*---------------------------------------------------------------------------------------------------------*/
/*  I2C (Master) Callback Function									                                   */
/*---------------------------------------------------------------------------------------------------------*/
bool I2C_MasterSendData(struct i2c_client *client);
bool I2C_MasterRcvData(struct i2c_client *client);
bool SendData(struct i2c_client *client);
bool RcvData(struct i2c_client *client);