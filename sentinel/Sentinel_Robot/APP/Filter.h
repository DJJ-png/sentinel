#ifndef Kalman_H
#define Kalman_H

#include "main.h"
#include "math.h"


#define MOVING_AVERAGE_SIZE 4

typedef struct 
{
	float fliter_num[3];
	float fliter_1,fliter_2,fliter_3;
	float out;
	float in;
	
}lpf_data_t;

typedef struct 
{
	float in;
	float Last_P;	//�ϴι���Э���� ������Ϊ0 ! ! ! ! ! 
	float Now_P;	//��ǰ����Э����
	float out;		//�������˲������
	float Kg;			//����������
	float Q;			//��������Э����
	float R;			//�۲�����Э����
	float ek;
	float rk;
}kf_data_t;

typedef struct {
    float buffer[MOVING_AVERAGE_SIZE];  // ���ݻ����������ڹ̶����ڣ�
    uint8_t index;                      // ��ǰ����
	float alpha;
    float ewma_value;                   // ָ����Ȩƽ��ֵ
	float oldest;
	uint8_t size;                       //<=MOVING_AVERAGE_SIZE
} WeightedFilter;

void LP_FilterInit(lpf_data_t *lp,float *fliter_num);
float LP_FilterCalc(lpf_data_t *lp,float input);
void Kalman_Init(kf_data_t *kf,float Q,float R);
float KalmanFilterCalc(kf_data_t *kf,float input);

void weighted_filter_init(WeightedFilter *filter,float alpha,uint8_t size);
float ewma_filter_update(WeightedFilter *filter, float input);

extern float fliter_num1[];

#endif
