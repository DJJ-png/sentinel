#include "bsp_buzzer.h"
#include "main.h"
//低音
//1 - do - 262Hz
//2 - re - 294Hz
//3 - mi - 330Hz
//4 - fa - 349Hz
//5 - so - 392Hz
//6 - la - 440Hz
//7 - si - 494Hz
//中音
//1 - do - 523Hz
//2 - re - 587Hz
//3 - mi - 659Hz
//4 - fa - 698Hz
//5 - so - 784Hz
//6 - la - 880Hz
//7 - si - 988Hz
//高音
//1 - do - 1046Hz
//2 - re - 1175Hz
//3 - mi - 1318Hz
//4 - fa - 1397Hz
//5 - so - 1568Hz
//6 - la - 1760Hz
//7 - si - 1976Hz
//高中低音的实际频率 Hz
const uint16_t music_steps[3][7] = {{262,294,330,349,392,440,494},
																		{523,587,659,698,784,880,988},
																		{1046,1175,1318,1397,1568,1760,1976}};
static uint16_t bzply_n = 0; //乐谱播放位置计数
static uint8_t bzply_count = 1; //单个音的节拍延时计数
extern TIM_HandleTypeDef htim4;
uint8_t buzzer_state = 1; //当前蜂鸣器播放状态
																		
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}


void SetBuzzerFrequence(uint16_t freq)
{
	//buzzer --> tim4.channel3
	//分频后为1000000Hz
	uint16_t period = 1000000/freq -1;
	
	__HAL_TIM_SET_AUTORELOAD(&htim4, period);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period/2);
}

void PlayingSong(const uint16_t *song, uint16_t len)
{
	uint8_t off_delay, on_delay, level, step;
	
	off_delay = ((song[bzply_n]&0xF000)>>12);
	on_delay = ((song[bzply_n]&0x0F00)>>8)*6;
	level = ((song[bzply_n]&0x00F0)>>4);
	step = (song[bzply_n]&0x000F)-1;
	
	if(bzply_count<on_delay)
	{
		bzply_count++;
		SetBuzzerFrequence(music_steps[level][step]);
	}
	else if(bzply_count<(on_delay+off_delay))
	{
		bzply_count++;
		buzzer_off();
	}
	else
	{
		bzply_count=1;
		bzply_n++; if(bzply_n>=len){bzply_n=0;buzzer_state=0;buzzer_off();}
	}
}

