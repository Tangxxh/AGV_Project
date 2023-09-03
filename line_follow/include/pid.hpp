#include <iostream>

typedef struct pidout
{
    /* data */
    int32_t value_right,value_left;
}speedOutStr;


class pid
{
private:
    /* data */
    double kp;
    int32_t value;
    int32_t temp;
    speedOutStr out;
public:
    pid(/* args */);
    ~pid();
    speedOutStr Robot_PID_Ctrl(int32_t speed,int16_t off); 
};
