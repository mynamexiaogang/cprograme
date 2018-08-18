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
#define phase_z 3 //零序

#define PAHSE_A 0x01
#define PAHSE_B 0x02
#define PAHSE_C 0x04

#define  system_ungrounded              0
#define  system_small_current_grounded  1
#define  system_arc_grounded            2

#define  halfarray_64k      1280    //6.4k采样时uint16_t型ADCValues的数组一半的大小
#define  UP_enable_flag     0X0001
#define  DOWN_enable_flag   0x0002
#define  BRK_enable_flag    0x0004
#define  LOAD_enable_flag   0x0008

#define  RECORD_TAP         220 //录波间隔
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

//定义事件类型
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
  U8		counts;     //计数器
  U8		flag;       //故障标志
  U8		fault_delay_start_flag; //故障延时启动标志
  U8    lost_delay_start_flag; //失压延时启动标志
  U8    vol_status;
  U8    trans_fault_flag; //发生瞬时性故障
  U8    per_fault_flag; //发生永久性故障
  //U8    fault_trip_flag; //线路是否发生故障标志
  //U8    fault_snd_flag; //触发信息发送标志
  stuevent_t event_data; //故障事件
  stuUtctime fault_start_time; //故障启动时间
  stuUtctime lost_start_time; //失压启动时间
}stujudge_t;

typedef struct
{
  U8		counts;     //计数器
  U8		flag;       //故障标志
  U8		limit_flag; //越限标志
  U8		direction;  //越限方向，1：up,2:down;
}stutrip_t;

typedef struct
{
  float real;   //实部
  float imagin; //虚部
}stuComplex;

typedef struct
{
  U8 limit_flag; //越限标志
  U8 direction; //越限方向，1：up,2:down;
  float ratio;
}stuvolstatus;

typedef struct
{
  U8 aPhaBreak; //1 突变,0 不突变
  U8 bPhaBreak; //1 突变,0 不突变
  U8 cPhaBreak; //1 突变,0 不突变
}PhaseBreakResult;

typedef struct
{
  U8 group; //组
  U8 phase; //相
  U8 channel; //通道序号
  U8 index; //周期索引
  U8 buff_len; //缓冲区长度
  U8 base_flag; //基准电压标志,电压通道有效
  U8 counts; //计算次数
  U8 fault_flag;
  stuSystime ffttime; //周期波形时间
  U16 offset; //偏移
  short *ADC_sample; //采样数据指针
}stuFFt;

typedef struct
{
  U8 group;
  U8 phase;
  U8 channel_type;
  U8 judgetype; //故障类型
  U16 cycyle_serial;
  unsigned long eventTimeMs; //8byte
  stuevent_t event;
}stufaultinfo;

typedef struct
{
  U8 group; //组
  U8 phase; //相
  U8 channel; //通道序号
  U8 base_flag; //基准电压标志,电压通道有效
  U8 counts; //计算次数
  U8 fault_flag; //故障标志
  stuAnalog *p;
  stuSystime *event_time;
  float *value_data;
}stuWave;

typedef struct
{
  U8 group;  //组
  U8 phase;  //相
  U8 counts; //计算次数
  stuSystime *event_time;
  float *value_data;
}stutrip;

extern stujudge_t fault_trip; //三相、相电场强度突变量，相电流突变量，相电流幅值越限
//extern stuComplex fftdata[3][2][12];
extern short fault_wave_data[128 * 2 *12 + 2];
extern short signal_wave_data[4][128*2*12 + 2]; //三通道单周期电压电流数据 128点 2通道 12周波 18432Byte
extern U8 signal_wave_flag;
extern U8 fault_wave_flag;
//extern float value_data[4][2][judge_buff_length_quarter]; //0：基波值; 1：总谐波；2: 三次谐波; 3: 5次谐波; 4: 7次谐波; 5：突变量;
extern float current_value[judge_buff_length_quarter]; //电流有效值
extern float voltage_value[judge_buff_length_quarter]; //电压有效值
extern float group_value_data[4][2][judge_buff_length]; //零序值
extern U8 isLinePower;
extern U8 zero_counts;
extern U8 recorde_flag; //录波启动标志 1:录波启动，0：录波未启动或录波结束
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

