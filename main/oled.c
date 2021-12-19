//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//中景园电子
//店铺地址：http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  文 件 名   : main.c
//  版 本 号   : v2.0
//  作    者   : Evk123
//  生成日期   : 2014-0101
//  最近修改   : 
//  功能描述   : 0.69寸OLED 接口演示例程(STM32F103ZE系列IIC)
//              说明: 
//              ----------------------------------------------------------------
//              GND   电源地
//              VCC   接5V或3.3v电源
//              SCL   接PD6（SCL）
//              SDA   接PD7（SDA）            
//              ----------------------------------------------------------------
//Copyright(C) 中景园电子2014/3/16
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////?

#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 		
 
esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_IO_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_IO_NUM,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = SCL_SPEED,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

uint8_t u8x8_gpio_and_delay_esp32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch(msg)
    {
        case U8X8_MSG_DELAY_NANO:               // delay arg_int * 1 nano second  1s = 1000000000ns
            __asm__ volatile("nop");
            break;

        case U8X8_MSG_DELAY_100NANO:            // delay arg_int * 100 nano seconds
            __asm__ volatile("nop");
            break;

        case U8X8_MSG_DELAY_10MICRO:            // delay arg_int * 10 micro seconds 1s = 1000000us
            for (uint16_t n = 0; n < 320; n++)
            {
                __asm__ volatile("nop");
            }
        break;

        case U8X8_MSG_DELAY_MILLI:              // delay arg_int * 1 milli second
            vTaskDelay(arg_int);
            break;

        case U8X8_MSG_GPIO_AND_DELAY_INIT:  
            // Function which implements a delay, arg_int contains the amount of ms  
            // set menu pin mode
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_HOME],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_SELECT],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_PREV],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_NEXT],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_UP],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_DOWN],PIN_MODE_INPUT_PULLUP);

            break;

        case U8X8_MSG_DELAY_I2C:
            // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
            // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
            for (uint16_t n = 0; n < (arg_int<=2?160:40); n++)
            {
                __asm__ volatile("nop");
            }
            break;
        default:
                break;
    }
    return 0;
}

uint8_t u8x8_byte_esp32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    uint8_t *data;
    static i2c_cmd_handle_t cmd = 0;
    static uint8_t ctrordata = 0;
    static int start = 0;
    switch(msg)
    {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            if(start == 1)  //第一个数据决定是否是数据还是命令
            {
                start = 0;
                ctrordata = data[0];
                break;
            }
            for(int i=0; i<arg_int; i++)
            {
                cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, 0x78, ACK_CHECK_EN);
                i2c_master_write_byte(cmd, ctrordata, ACK_CHECK_EN);
                i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
                i2c_master_stop(cmd);
                esp_err_t ret = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
                if(ret != ESP_OK)
                	printf("write error %d\n", ret);
                i2c_cmd_link_delete(cmd);
            }
            break;

        case U8X8_MSG_BYTE_INIT:
            break;

        case U8X8_MSG_BYTE_SET_DC:
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            start = 1;
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            start = 0;
            break;

        default:
            break;
    }
    return 0;
}

esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ESP_SLAVE_ADDR, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    if(ret != ESP_OK)
        printf("write error %d\n", ret);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void Write_IIC_Command(unsigned char IIC_Command)
{
    unsigned char data[2];
    data[0]=0x00;
    data[1] = IIC_Command;
    i2c_master_write_slave(0, data, 2);
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
    unsigned char data[2];
    data[0] = 0x40;
    data[1] = IIC_Data;
    i2c_master_write_slave(0, data, 2);
}

void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd)
	{
   		Write_IIC_Data(dat);
	}
	else 
	{
   		Write_IIC_Command(dat);
	}
}

/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte(0xb0+m,0);		//page0-page1
		OLED_WR_Byte(0x00,0);		//low column start address
		OLED_WR_Byte(0x10,0);		//high column start address
		for(n=0;n<128;n++)
		{
			OLED_WR_Byte(fill_Data,1);
		}
	}
}

//坐标设置
void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD); 
}   	  
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //更新显示
}
void OLED_On(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
	} //更新显示
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{      	
	unsigned char c=0,i=0;	
	c=chr-' ';//得到偏移后的值			
	if(x>Max_Column-1){x=0;y=y+2;}
	if(Char_Size ==16)
	{
		OLED_Set_Pos(x,y);	
		for(i=0;i<8;i++)
		OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
		OLED_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
		OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
	}
	else 
	{	
		OLED_Set_Pos(x,y);
		for(i=0;i<6;i++)
		OLED_WR_Byte(F6x8[c][i],OLED_DATA);
		
	}
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 
//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		
		OLED_ShowChar(x,y,chr[j],Char_Size);
		x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}
//显示汉字
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{      			    
	u8 t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
	{
		OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
		adder+=1;
    }	
	OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
	{	
		OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
		adder+=1;
	}					
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
	unsigned int j=0;
	unsigned char x,y;

	if(y1%8==0) y=y1/8;      
	else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    	for(x=x0;x<x1;x++)
	    {      
	    	OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 

//初始化SSD1306					    
void OLED_Init(void)
{ 	
	OLED_WR_Byte(0xAE,OLED_CMD);//关闭显示
	
	OLED_WR_Byte(0x40,OLED_CMD);//---set low column address
	OLED_WR_Byte(0xB0,OLED_CMD);//---set high column address

	OLED_WR_Byte(0xC8,OLED_CMD);//-not offset

	OLED_WR_Byte(0x81,OLED_CMD);//设置对比度
	OLED_WR_Byte(0xff,OLED_CMD);

	OLED_WR_Byte(0xa1,OLED_CMD);//段重定向设置

	OLED_WR_Byte(0xa6,OLED_CMD);//
	
	OLED_WR_Byte(0xa8,OLED_CMD);//设置驱动路数
	OLED_WR_Byte(0x1f,OLED_CMD);
	
	OLED_WR_Byte(0xd3,OLED_CMD);
	OLED_WR_Byte(0x00,OLED_CMD);
	
	OLED_WR_Byte(0xd5,OLED_CMD);
	OLED_WR_Byte(0xf0,OLED_CMD);
	
	OLED_WR_Byte(0xd9,OLED_CMD);
	OLED_WR_Byte(0x22,OLED_CMD);
	
	OLED_WR_Byte(0xda,OLED_CMD);
	OLED_WR_Byte(0x02,OLED_CMD);
	
	OLED_WR_Byte(0xdb,OLED_CMD);
	OLED_WR_Byte(0x49,OLED_CMD);
	
	OLED_WR_Byte(0x8d,OLED_CMD);
	OLED_WR_Byte(0x14,OLED_CMD);
	
	OLED_WR_Byte(0xaf,OLED_CMD);
	OLED_Clear();
}  