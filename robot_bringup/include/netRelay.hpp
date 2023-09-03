/*
 * @Author: Wei Zhifei
 * @Date: 2023-04-14 13:18:38
 */
#include <string>
#include <unistd.h>
#include <modbus/modbus.h>

#define SLAVE_ID    11

#define OUT1        0x0000
#define OUT2        0x0001
#define OUT3        0x0002
#define OUT4        0x0003

#define RELAY_ON    0xFF00
#define RELAY_OFF   0x0000

class netRelay
{
private:
    modbus_t *fd;
    uint16_t *tab_reg; //定义存放数据的数组
    int rc;
public:
    netRelay(modbus_t * ctx);
    ~netRelay();
    void setNetRelay(int slave, uint16_t value);
};

netRelay::netRelay(modbus_t * ctx)
{
    fd = ctx;
    tab_reg = new uint16_t(64);
    while(modbus_connect(fd) == -1) //等待连接设备
    {
		fprintf(stderr, "Connection failed:%s\n", modbus_strerror(errno));
		modbus_close(ctx);
		usleep(500000);
    }
}

netRelay::~netRelay()
{
    delete []tab_reg;
    modbus_close(fd);
}

void netRelay::setNetRelay(int slave, uint16_t value)
{
    modbus_set_slave(fd, slave);      //设置slave ID
    printf("slave id = %d \r\n",slave);
    /*rc = modbus_read_registers(fd, 3, 2, tab_reg);
    if (rc == -1)                   //读取保持寄存器的值，可读取多个连续输入保持寄存器
    {
        fprintf(stderr,"Reading error:%s\n", modbus_strerror(errno));
        return -1;
    }*/
	while(-1 ==  modbus_write_bit(fd, OUT2, value))
	{
		fprintf(stderr,"Writing error:%s\n", modbus_strerror(errno));
		// modbus_close(fd);
		// usleep(200000);
		// modbus_connect(fd);
		usleep(200000);
	}
}