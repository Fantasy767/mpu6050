#include "mpu6050.h"
#include "config.h"
#include "stm32f1xx_hal_i2c.h"
#include <stdint.h>
int32_t acc_x_offset=0;
int32_t acc_y_offset=0;
int32_t acc_z_offset=0;
int32_t gyro_x_offset=0;
int32_t gyro_y_offset=0;
int32_t gyro_z_offset=0;
void mpu6050_write_reg(uint8_t reg, uint8_t data)
{   
   //1.句柄2.从设备地址 mpu6050是0x68 3.寄存器地址reg 
   // 4.寄存器地址长度 5.写入的数据 6.数据长度 7.超时时间
    HAL_I2C_Mem_Write(&hi2c1, mpu6050_write_add, reg,I2C_MEMADD_SIZE_8BIT , &data, 1, 1000);

}
void mpu6050_read_reg(uint8_t reg, uint8_t *data)
{
    //1.句柄2.从设备地址 mpu6050是0x68 3.寄存器地址reg 
   // 4.寄存器地址长度 5.读出的数据 6.数据长度 7.超时时间
    HAL_I2C_Mem_Read(&hi2c1, mpu6050_write_add, reg,I2C_MEMADD_SIZE_8BIT , data, 1, 1000);
}
void mpu6050_calculate_offset()
{
    //等待飞机停放平稳
    //判断飞机停放平稳的金标准：前后的加速度 差值<200，连续100次
    accel current_accel={0};
    accel last_accel={0};
    uint8_t count=0;
    mpu6050_get_accel(&last_accel);
    while(count<100)
    {
        mpu6050_get_accel(&current_accel);
        if(abs(current_accel.accel_x-last_accel.accel_x)<500&&
            abs(current_accel.accel_y-last_accel.accel_y)<500&&
            abs(current_accel.accel_z-last_accel.accel_z)<500)
        {
            count++;
        }
        else
        {
            count=0;
        }
        last_accel=current_accel;
        HAL_Delay(6);
    }
    mpu6050_data flight_data={0};
    int32_t acc_x_sum=0;
    int32_t acc_y_sum=0;
    int32_t acc_z_sum=0;
    int32_t gyro_x_sum=0;
    int32_t gyro_y_sum=0;
    int32_t gyro_z_sum=0;   
    for(int i=0;i<100;i++)
    {
        mpu6050_get_alldata(&flight_data);
        acc_x_sum+=flight_data.accel.accel_x;
        acc_y_sum+=flight_data.accel.accel_y;
        //z轴的偏移量是16384  因为z轴的量程是+-2g  
        acc_z_sum+=flight_data.accel.accel_z-16384;
        gyro_x_sum+=flight_data.gyro.gyro_x;
        gyro_y_sum+=flight_data.gyro.gyro_y;
        gyro_z_sum+=flight_data.gyro.gyro_z;
        HAL_Delay(6);
    }
    acc_x_offset=acc_x_sum/100;
    acc_y_offset=acc_y_sum/100;
    acc_z_offset=acc_z_sum/100;
    gyro_x_offset=gyro_x_sum/100;
    gyro_y_offset=gyro_y_sum/100;
    gyro_z_offset=gyro_z_sum/100;

    
}
void mpu6050_init()
{
    //重启芯片，重置所有寄存器 reset
    mpu6050_write_reg(0x6b, 0x80);
    uint8_t data=0;
    while(data!=0x40)
    {
        mpu6050_read_reg(0x6b,&data);
    }
    //唤醒芯片
    mpu6050_write_reg(0x6b,0x00);
    //填写角速度的量程是+-2000
    mpu6050_write_reg(0x1b,0x18);
    //填写加速度量程为+-2g
    mpu6050_write_reg(0x1c,0x00);
    //关闭中断  因为没用到
    mpu6050_write_reg(0x38,0x00);
    //用户配置寄存器
    mpu6050_write_reg(0x6a,0x00);
    //设置采样频率  监控三周加速度和角速度  默认1khz
    mpu6050_write_reg(0x19,0x01);
    //设置低通滤波 184hz
    mpu6050_write_reg(0x1a,1);
    //设置系统时钟
    mpu6050_write_reg(0x6b,0x01);
    //打开加速度角速度传感器
    mpu6050_write_reg(0x6c,0x00);
    //零偏校准
    mpu6050_calculate_offset();
}
void mpu6050_get_gyro(gyro *gyro)
{
    //存储角速度的寄存器 从0x43开始 xyz顺序
    uint8_t high=0,low=0;
    //这里存的是原始值(LSB)，不要在这里做单位换算，也不要再除16
    //换算成°/s：raw * (2000/32768)   因为±2000dps档的灵敏度是16.4LSB/(°/s)
    //X
    mpu6050_read_reg(MPU_GYRO_XOUTH_REG,&high);
    mpu6050_read_reg(MPU_GYRO_XOUTL_REG,&low);
    gyro->gyro_x = (int16_t)((high<<8) | low)-gyro_x_offset;
    //Y
    mpu6050_read_reg(MPU_GYRO_YOUTH_REG,&high);
    mpu6050_read_reg(MPU_GYRO_YOUTL_REG,&low);
    gyro->gyro_y = (int16_t)((high<<8) | low)-gyro_y_offset;

    //Z
    mpu6050_read_reg(MPU_GYRO_ZOUTH_REG,&high);
    mpu6050_read_reg(MPU_GYRO_ZOUTL_REG,&low);
    gyro->gyro_z = (int16_t)((high<<8) | low)-gyro_z_offset;

}
void mpu6050_get_accel(accel *acc)
{
    uint8_t high=0,low=0;
    //X
    mpu6050_read_reg(MPU_ACCEL_XOUTH_REG,&high);
    mpu6050_read_reg(MPU_ACCEL_XOUTL_REG,&low);
    acc->accel_x = (int16_t)((high<<8) | low)-acc_x_offset;
    //Y
    mpu6050_read_reg(MPU_ACCEL_YOUTH_REG,&high);
    mpu6050_read_reg(MPU_ACCEL_YOUTL_REG,&low);
    acc->accel_y = (int16_t)((high<<8) | low)-acc_y_offset;
    //Z
    mpu6050_read_reg(MPU_ACCEL_ZOUTH_REG,&high);
    mpu6050_read_reg(MPU_ACCEL_ZOUTL_REG,&low);
    acc->accel_z = (int16_t)((high<<8) | low)-acc_z_offset;
}
void mpu6050_get_alldata(mpu6050_data *data )
{
    mpu6050_get_gyro(&data->gyro);
    mpu6050_get_accel(&data->accel);

}
