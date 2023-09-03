#include <string>
#include <modbus/modbus.h>
#include <iostream>
class temp
{
private:
    modbus_t *fd;
    float temperature;
    uint16_t *tab_reg; //定义存放数据的数组
    int rc;
public:
    temp(modbus_t * ctx);
    ~temp();
    float getTemperature(void);
};

temp::temp(modbus_t * ctx)
{
    fd = ctx;
    tab_reg = new uint16_t(64);
}

temp::~temp()
{
    delete []tab_reg;
}

float temp::getTemperature(void)
{
    modbus_set_slave(fd, 1);      //设置slave ID
    if (modbus_connect(fd) == -1) //等待连接设备
	{
    	fprintf(stderr, "[temp]:Connection failed:%s\n", modbus_strerror(errno));
    	return -1;
	}

    rc = modbus_read_registers(fd, 0, 2, tab_reg);
    if (rc == -1)                   //读取保持寄存器的值，可读取多个连续输入保持寄存器
    {
        fprintf(stderr,"[temp]:%s\n", modbus_strerror(errno));
        return -1;
    }
    temperature = (tab_reg[1])/10.0f;

    return temperature;
}
