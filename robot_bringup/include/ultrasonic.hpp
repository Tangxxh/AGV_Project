#include <string>
#include <unistd.h>
#include <modbus/modbus.h>
#include <iostream>
// #include <utilty.h>

#define FRONT_ULTRA 4
#define BACKWARD_ULTRA 5

class ultrasonic
{
private:
    modbus_t *fd;
    uint16_t *tab_reg; //定义存放数据的数组
    int rc;
public:
    ultrasonic(modbus_t * ctx);
    ~ultrasonic();
    uint32_t getDistance(int id);
};

ultrasonic::ultrasonic(modbus_t * ctx)
{
    fd = ctx;
    tab_reg = new uint16_t(64);
    while(modbus_connect(fd) == -1) //等待连接设备
    {
		fprintf(stderr, "[ultrasonic]:Connection failed:%s\n", modbus_strerror(errno));
		modbus_close(ctx);
		sleep(1);
    }
}

ultrasonic::~ultrasonic()
{
    modbus_close(fd);
    modbus_free(fd);
    delete []tab_reg;
}

uint32_t ultrasonic::getDistance(int id)
{
    uint16_t distance;
    modbus_set_slave(fd, id);      //设置slave ID
    // if (modbus_connect(fd) == -1) //等待连接设备
	// {
    // 	fprintf(stderr, "Connection failed:%s\n", modbus_strerror(errno));
    // 	return -1;
	// }

    while(-1 == modbus_read_registers(fd, 0x0101, 0x0001, &distance))//读取保持寄存器的值，可读取多个连续输入保持寄存器
    {
        fprintf(stderr,"[ultrasonic]:%s\n", modbus_strerror(errno));
        modbus_close(fd);
		usleep(200000);
		modbus_connect(fd);
		usleep(200000);
    }

    return (uint32_t)distance;
}
