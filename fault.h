#ifndef _FAULT_H_
#define _FAULT_H_
#include "Public.h"
#include "math.h"
#include "time_deal.h"

#define judge_buff_length           12
#define judge_buff_length_quarter   48
#define phase_a 0
#define phase_b 1
#define phase_c 2
#define phase_z 3 //����

#define PAHSE_A 0x01
#define PAHSE_B 0x02
#define PAHSE_C 0x04

#define  system_ungrounded              0
#define  system_small_current_grounded  1
#define  system_arc_grounded            2

#define  halfarray_64k      1280    //6.4k����ʱuint16_t��ADCValues������һ��Ĵ�С
#define  UP_enable_flag     0X0001
#define  DOWN_enable_flag   0x0002
#define  BRK_enable_flag    0x0004
#define  LOAD_enable_flag   0x0008

#define  RECORD_TAP         220 //¼�����
#define  VOLTAGE_CHANNEL_ENABLE 1

#ifdef WIN32
#define  vol_enlarge  1
#define  cur_enlarge  1
#else
#define  vol_enlarge  1
#define  cur_enlarge  1
#endif

#define vol_brk_chk   0
#define cur_brk_chk   1
#define cur_over_chk  2
#define vol_channel   0
#define cur_channel   1
#define MAX_INDEX     5
#define sin_const     0.06940//2*1.414*sin(w*t/2)

#define FFT_128_H

#define SIGLE_PASHE_SIZE 20

//�����¼�����
enum
{
  cur_base_over_serial = 0,
  cur_load_serial,
  cur_harm_serial,
  cur_harm_thr_serial,
  cur_harm_fiv_serial,
  cur_harm_sev_serial,
  cur_brk_serial,
  vol_base_over_serial,
  vol_base_lower_serial,
  vol_harm_serial,
  vol_harm_thr_serial,
  vol_harm_fiv_serial,
  vol_harm_sev_serial,
  vol_brk_serial,
  phase_brk_serial,
  cur_zero_over_serial = 0,
  cur_zero_brk_serial,
  vol_zero_over_serial = 0,
  vol_zero_brk_serial
};

typedef struct
{
  U8		counts;     //������
  U8		flag;       //���ϱ�־
  U8		fault_delay_start_flag; //������ʱ������־
  U8    lost_delay_start_flag; //ʧѹ��ʱ������־
  U8    vol_status;
  U8    trans_fault_flag; //����˲ʱ�Թ���
  U8    per_fault_flag; //���������Թ���
  //U8    fault_trip_flag; //��·�Ƿ������ϱ�־
  //U8    fault_snd_flag; //������Ϣ���ͱ�־
  stuevent_t event_data; //�����¼�
  stuUtctime fault_start_time; //��������ʱ��
  stuUtctime lost_start_time; //ʧѹ����ʱ��
}stujudge_t;

typedef struct
{
  U8		counts;     //������
  U8		flag;       //���ϱ�־
  U8		limit_flag; //Խ�ޱ�־
  U8		direction;  //Խ�޷���1��up,2:down;
}stutrip_t;

typedef struct
{
  float real;   //ʵ��
  float imagin; //�鲿
}stuComplex;

typedef struct
{
  U8 limit_flag; //Խ�ޱ�־
  U8 direction; //Խ�޷���1��up,2:down;
  float ratio;
}stuvolstatus;

typedef struct
{
  U8 aPhaBreak; //1 ͻ��,0 ��ͻ��
  U8 bPhaBreak; //1 ͻ��,0 ��ͻ��
  U8 cPhaBreak; //1 ͻ��,0 ��ͻ��
}PhaseBreakResult;

typedef struct
{
  U8 group; //��
  U8 phase; //��
  U8 channel; //ͨ�����
  U8 index; //��������
  U8 buff_len; //����������
  U8 base_flag; //��׼��ѹ��־,��ѹͨ����Ч
  U8 counts; //�������
  U8 fault_flag;
  stuSystime ffttime; //���ڲ���ʱ��
  U16 offset; //ƫ��
  short *ADC_sample; //��������ָ��
}stuFFt;

typedef struct
{
  U8 group;
  U8 phase;
  U8 channel_type;
  U8 judgetype; //��������
  U16 cycyle_serial;
  unsigned long eventTimeMs; //8byte
  stuevent_t event;
}stufaultinfo;

typedef struct
{
  U8 group; //��
  U8 phase; //��
  U8 channel; //ͨ�����
  U8 base_flag; //��׼��ѹ��־,��ѹͨ����Ч
  U8 counts; //�������
  U8 fault_flag; //���ϱ�־
  stuAnalog *p;
  stuSystime *event_time;
  float *value_data;
}stuWave;

typedef struct
{
  U8 group;  //��
  U8 phase;  //��
  U8 counts; //�������
  stuSystime *event_time;
  float *value_data;
}stutrip;

extern stujudge_t fault_trip; //���ࡢ��糡ǿ��ͻ�����������ͻ�������������ֵԽ��
//extern stuComplex fftdata[3][2][12];
extern short fault_wave_data[128 * 2 *12 + 2];
extern short signal_wave_data[4][128*2*12 + 2]; //��ͨ�������ڵ�ѹ�������� 128�� 2ͨ�� 12�ܲ� 18432Byte
extern U8 signal_wave_flag;
extern U8 fault_wave_flag;
//extern float value_data[4][2][judge_buff_length_quarter]; //0������ֵ; 1����г����2: ����г��; 3: 5��г��; 4: 7��г��; 5��ͻ����;
extern float current_value[judge_buff_length_quarter]; //������Чֵ
extern float voltage_value[judge_buff_length_quarter]; //��ѹ��Чֵ
extern float group_value_data[4][2][judge_buff_length]; //����ֵ
extern U8 isLinePower;
extern U8 zero_counts;
extern U8 recorde_flag; //¼��������־ 1:¼��������0��¼��δ������¼������
extern U8 math_counts;
extern U8 fault_trip_num;


//void check_unitparam(U8 group, U8 phase);
bool zero_judge(U8 group, U16 channel, stuSystime event_timese);
//void record_wave_fix(void);
stuComplex fft_math_32(I16 *src);
void judge_param_init(void);
U8 trip_judge(U8 group, U8 phase, U16 count, stuSystime *time, float *value_data);
//void handle_fault_event(stuSystime event_time);
void computeEffectiveValue(stuFFt *src_data,stuAnalog *ptrParam);
void signal_phase_judge(stuFFt *fft_data);
extern U8 waveform_analysis(stufaultinfo *faultinfo);
extern void toUTCtime(stuSystime systime, stuUtctime *utctime);

static U8 math_vol_zero_peak(void);
extern void diableFaultJudge(stuSystime *time,U32 timeout);

extern void enableFaultJudge();
#endif

