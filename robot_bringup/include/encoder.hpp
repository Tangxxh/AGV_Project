/*
 * @Author: Wei Zhifei
 * @Date: 2022-11-12 13:18:38
 */
#include <string>
#include <modbus/modbus.h>

#define BIGJOINT_MODBUS_ADDR 2
#define SMALLJOINT_MODBUS_ADDR 3

class encoder
{
private:
    modbus_t *fd;
    uint32_t pulse;
    uint16_t *tab_reg; //定义存放数据的数组
    int rc;
public:
    encoder(modbus_t * ctx);
    ~encoder();
    uint32_t getPulse(int slave);
    double getAbsolutelyDeg(int slave);
};

encoder::encoder(modbus_t * ctx)
{
    fd = ctx;
    tab_reg = new uint16_t(64);
    while(modbus_connect(fd) == -1) //等待连接设备
    {
		fprintf(stderr, "[encoder]:Connection failed:%s\n", modbus_strerror(errno));
		modbus_close(ctx);
		usleep(500000);
    }
}

encoder::~encoder()
{
    delete []tab_reg;
}

uint32_t encoder::getPulse(int slave)
{
    modbus_set_slave(fd, slave);      //设置slave ID
    /*rc = modbus_read_registers(fd, 3, 2, tab_reg);
    if (rc == -1)                   //读取保持寄存器的值，可读取多个连续输入保持寄存器
    {
        fprintf(stderr,"Reading error:%s\n", modbus_strerror(errno));
        return -1;
    }*/
	while(-1 ==  modbus_read_registers(fd, 3, 2, tab_reg))
	{
		fprintf(stderr,"[encoder]:Reading error:%s\n", modbus_strerror(errno));
		modbus_close(fd);
		usleep(200000);
		modbus_connect(fd);
		usleep(200000);
	}
    pulse = (uint32_t)((tab_reg[1] << 16) | tab_reg[0]);

    return pulse;
}


double encoder::getAbsolutelyDeg(int slave)
{
    double posi_deg = getPulse(slave)*360.0/4096;
    if(posi_deg > 180.0)
	    posi_deg -= 360.0;
    return posi_deg;
}
