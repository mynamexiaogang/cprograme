/**采集单元判断故障，汇集判断故障类型*/

#include "Public.h"
#include "fault.h"
#include "math.h"
#include "msg.h"
#include "log_file.h"
#include "zigbee_cmd.h"
#include "IEC104.h"
#ifndef WIN32
#include "time_deal.h"
#else
//#include <Windows.h>
#include <string>
#include <file.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <cstringt.h>
#include <WinUser.h>
#endif

short  fault_wave_data[128 * 2 *12 + 2];
//short  v_fault_wave_data[128 *12 + 1];
//short  c_fault_wave_data[128 *12 + 1];

#ifndef WIN32
//short va_wave_data[128 * 12 + 1]@"sram2";
//short vb_wave_data[128 * 12 + 1]@"sram2";
//short vc_wave_data[128 * 12 + 1]@"sram2";
//short v0_wave_data[128 * 12 + 1]@"sram2";
//short ca_wave_data[128 * 12 + 1]@"sram2";
//short cb_wave_data[128 * 12 + 1]@"sram2";
//short cc_wave_data[128 * 12 + 1]@"sram2";
//short c0_wave_data[128 * 12 + 1]@"sram2";

short  signal_wave_data[4][128 * 2 * 12 + 2]@"sram2"; //三通道单周期电压电流数据 18432Byte,
#else

//short va_wave_data[128 * 12 + 1];
//short vb_wave_data[128 * 12 + 1];
//short vc_wave_data[128 * 12 + 1];
//short v0_wave_data[128 * 12 + 1];
//short ca_wave_data[128 * 12 + 1];
//short cb_wave_data[128 * 12 + 1];
//short cc_wave_data[128 * 12 + 1];
//short c0_wave_data[128 * 12 + 1];

short  signal_wave_data[4][128 * 2 * 12+2]; //三通道单周期电压电流数据 18432Byte,

#endif
stujudge_t fault_trip; //三相、相电场强度突变量，相电流突变量，相电流幅值越限
U8 isLinePower = false;
U8 signal_wave_flag = false;
U8 fault_wave_flag = false;
static U8 event_cur_over_flag = false; //相电流越上限
static U8 event_vol_over_flag = false; //相电压越上限
static U8 event_vol_under_flag = false; //相电压越下限
U8 event_cur_up_break_flag = false;  //电流上突变
U8 event_vol_down_break_flag = false;//电压下突变
U8 fault_trip_num = 0;

#define vol_unknown 0 //0. unknown 1.上电 2.//失电 3.//电压正常 4.//线路无电
#define vol_up      1 //上电
#define vol_down    2 //失电
#define vol_normal  3 //电压正常·
#define vol_none    4 //线路无电
#define vol_fault   5 //线路处于故障
#define only_voltage 6  //线路只有电压
#define only_current 7  //线路只有电流
#define only_current_up 8 //线路只有电流,并且上电

#define INDICATOR_ON              0
#define INDICATOR_OFF             1
#define ARC_PEAK_THREADSHOLD_NUM  5

#define  judge_type_over      0  //越上限
#define  judge_type_low       1  //越下限
#define  judge_type_brk       2  //突变
#define  judge_type_ph_brk    2  //相角突变

#define CURRENT_CRITICAL_DOWN 15.0f
#define FAULT_LINGT_MASK 0x0800

U8 transperflag = 0;

U8 math_counts = 0;

float current_value[judge_buff_length_quarter]; //电流有效值
float voltage_value[judge_buff_length_quarter]; //电压有效值
float group_value_data[4][2][judge_buff_length]; //A/B/C/0 电压 电流值

//float va_effective_value_data[judge_buff_length]; 
//float vb_effective_value_data[judge_buff_length]; 
//float vc_effective_value_data[judge_buff_length]; 
//float v0_effective_value_data[judge_buff_length]; 
//float ca_effective_value_data[judge_buff_length]; 
//float cb_effective_value_data[judge_buff_length];
//float cc_effective_value_data[judge_buff_length];
//float c0_effective_value_data[judge_buff_length]; 

U8 recorde_flag = 0; //录波启动标志 1:录波启动，0：录波未启动或录波结束

//[2]:0:电压，1：电流；【10】：0：上限，1：下限，2：突变；3：重载4：总谐波，5：三次谐波；6:5次谐波，7:7次谐波；8:9次谐波；9:11次谐波;10:零序电压，11：零序电流
const float  fft_coe_sin[9] = { 0, 0.19509, 0.38268, 0.55557, 0.70711, 0.83147, 0.92388, 0.98079, 1 };
const float  fft_coe_cos[9] = { 1, 0.98079, 0.92388, 0.83147, 0.70711, 0.55557, 0.38268, 0.19509, 0 };

float const phase_t[60] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
  32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60 };
float const phase_tan[60] = { 0.017455065, 0.034920769, 0.052407778, 0.069926811, 0.087488662, 0.105104233, 0.122784559, 0.140540832, 0.158384438, 0.176326978, 0.194380306, 0.212556558, 0.230868187,
  0.249327998, 0.267949188, 0.286745381, 0.305730676, 0.32491969, 0.344327607, 0.363970228, 0.383864028, 0.404026218, 0.424474808, 0.445228677, 0.466307649, 0.487732579, 0.509525439, 0.531709421, 0.55430904,
  0.577350257, 0.600860606, 0.624869339, 0.649407579, 0.674508502, 0.700207523, 0.726542512, 0.753554033, 0.781285608, 0.809784014, 0.839099611, 0.869286716, 0.900404022, 0.932515062, 0.965688749, 0.999999973,
  1.035530285, 1.07236868, 1.110612483, 1.150368373, 1.191753557, 1.234897118, 1.279941591, 1.327044778, 1.376381874, 1.428147957, 1.482560915, 1.539864907, 1.600334468, 1.664279416, 1.732050736 };

float const c_table_sin[128] = { 0.000000, 0.049068, 0.098017, 0.146730, 0.195090, 0.242980, 0.290285, 0.336890, 0.382683, 0.427555, 0.471397,
  0.514103, 0.555570, 0.595699, 0.634393, 0.671559, 0.707107, 0.740951, 0.773010, 0.803208, 0.831470, 0.857729, 0.881921, 0.903989, 0.923880, 0.941544, 0.956940, 0.970031, 0.980785, 0.989177, 0.995185, 0.998795, 1.000000, 0.998795, 0.995185, 0.989177, 0.980785, 0.970031, 0.956940, 0.941544, 0.923880, 0.903989, 0.881921, 0.857729, 0.831470, 0.803208, 0.773010, 0.740951, 0.707107, 0.671559, 0.634393, 0.595699, 0.555570, 0.514103, 0.471397,
  0.427555, 0.382683, 0.336890, 0.290285, 0.242980, 0.195090, 0.146730, 0.098017, 0.049068, 0.000000, -0.049068, -0.098017, -0.146730, -0.195090,
  -0.242980, -0.290285, -0.336890, -0.382683, -0.427555, -0.471397, -0.514103, -0.555570, -0.595699, -0.634393, -0.671559, -0.707107,
  -0.740951, -0.773010, -0.803208, -0.831470, -0.857729, -0.881921, -0.903989, -0.923880, -0.941544, -0.956940, -0.970031, -0.980785,
  -0.989177, -0.995185, -0.998795, -1.000000, -0.998795, -0.995185, -0.989177, -0.980785, -0.970031, -0.956940, -0.941544, -0.923880,
  -0.903989, -0.881921, -0.857729, -0.831470, -0.803208, -0.773010, -0.740951, -0.707107, -0.671559, -0.634393, -0.595699, -0.555570,
  -0.514103, -0.471397, -0.427555, -0.382683, -0.336890, -0.290285, -0.242980, -0.195090, -0.146730, -0.098017, -0.049068
};

stuComplex fft_math_32(I16 *src);

void add_event_list(stuevent_t *pedata);
void get_utc_time(stuUtctime *utctime);
//static U8 phase_brk_judge_Quarter(stuFFt *fft_data, float phaseBreak, U16 breakNum);
static U8 doRealFaultPush(stufaultinfo *faultinfo);

//static U8 Phase_brk_quarter(I16 *src, I16 *lstsrc, I16 group, I16 phase, float phaseBreak, U16 breakNum);
//static U8 Phase_brk_quarter2(I16 *src, I16 *lstsrc, I16 group, I16 phase, float phaseBreak, U16 breakNum);

static void judgeLinePowerStatus(float voltageCriticalValue, float currentCriticalValue, U8 count);
static U8 doFaultJudage(U8 count, stuSystime *time_t);
static void judgeInstantOrPermanent(U8 count, stuSystime *time_t,U32 sec,U16 ms);

stuComplex fftdata[MAX_PHASE_NUM][2][12];
//PhaseBreakResult phaseBreakResult;
I16 current_swap_adc_value[128] = { 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048 };
I16 voltage_swap_adc_value[128] = { 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048
  , 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048 };

#define IS_SHOW_PRINT 1

#define computeCurrentEffective(amplitude,slope,basis)   (amplitude *cur_enlarge * 0.707f*slope+basis)

#define computeVoltageEffective(amplitude,slope,basis)   (amplitude *vol_enlarge*0.707f*slope+basis)

float faultVoltage=0.1f;

/*
计算虚部
src:数据源
thd:谐波次数，1：基波
*/
float getDFTnRX(short *src, short THD)
{
  float fft_Rx = 0.0;
  int i = 0;
  for (i = 1; i < 128; i++)
  {
    fft_Rx += src[i] * c_table_sin[i * THD % 128];
  }
  return fft_Rx / 64;
}
/************************************************************************/
/*

*/
/************************************************************************/
float getDFTnTX(short *src, short THD)
{
  float fft_Tx = 0.0;
  int i = 0;
  for (i = 1; i < 128; i++)
  {
    fft_Tx += src[i] * c_table_sin[(i * THD + 32) % 128];
  }
  fft_Tx /= 64;
  fft_Tx += ((src[0] + src[127]) / 128);
  return fft_Tx;
}
/************************************************************************/
/*

*/
/************************************************************************/
float getDFTnRX_half(short *src, short THD)
{
  float fft_Rx = 0.0;
  int i = 0;
  for (i = 1; i < 63; i++)
  {
    fft_Rx += src[i] * c_table_sin[i * THD % 128];
  }
  return fft_Rx / 32;
}
/************************************************************************/
/*

*/
/************************************************************************/
float getDFTnTX_half(short *src, short THD)
{
  float fft_Tx = 0.0;
  int i = 0;
  for (i = 1; i < 63; i++)
  {
    fft_Tx += src[i] * c_table_sin[(i * THD + 32) % 128];
  }
  fft_Tx /= 32;
  fft_Tx += ((src[0] + src[63]) / 64);
  return fft_Tx;
}
/************************************************************************/
/*

*/
/************************************************************************/
stuComplex mul_compux(stuComplex x, stuComplex y)
{
  stuComplex p;
  p.real = x.real * y.real - x.imagin * y.imagin;
  p.imagin = x.real * y.imagin + x.imagin * y.real;
  return p;
}
/************************************************************************/
/*

*/
/************************************************************************/
stuComplex add_compux(stuComplex x, stuComplex y)
{
  stuComplex p;
  p.real = x.real + y.real;
  p.imagin = y.imagin + x.imagin;
  return p;
}

stuComplex sub_compux(stuComplex x, stuComplex y)
{
  stuComplex p;
  p.real = x.real - y.real;
  p.imagin = x.imagin - y.imagin;
  return p;
}

/************************************************************************/
/*

*/
/************************************************************************/
stuComplex a;
stuComplex math_pos_xu(stuComplex x, stuComplex y, stuComplex z)
{
  stuComplex p;
  stuComplex a2;
  a.imagin = 0.866;
  a.real = -0.5;
  a2 = mul_compux(a, a);
  p = add_compux(mul_compux(a, y), mul_compux(a2, z));
  p = add_compux(p, x);
  p.imagin = p.imagin / 3;
  p.real = p.real / 3;
  return p;
}

/************************************************************************/
/*


*/
/************************************************************************/
stuComplex math_neg_xu(stuComplex x, stuComplex y, stuComplex z)
{
  stuComplex p;
  stuComplex a2;
  a.imagin = 0.866;
  a.real = -0.5;
  a2 = mul_compux(a, a);
  p = add_compux(mul_compux(a, z), mul_compux(a2, y));
  p = add_compux(p, x);
  p.imagin = p.imagin / 3;
  p.real = p.real / 3;
  return p;
}
/************************************************************************/
/*


*/
/************************************************************************/
stuComplex math_zero_xu(stuComplex x, stuComplex y, stuComplex z)
{
  stuComplex p;
 // stuComplex a2;
  p = add_compux(x, y);
  p = add_compux(p, z);
  p.imagin = p.imagin / 3;
  p.real = p.real / 3;
  return p;
}
/************************************************************************/
/*


*/
/************************************************************************/
stuComplex fft_math_128(I16 *src, short THD)
{
  float Rex = 0.0;
  float Zex = 0.0;
  stuComplex data;
  Rex = getDFTnRX(src, THD);
  Zex = getDFTnTX(src, THD);
  data.imagin = Zex;
  data.real = Rex;
  return data;
}

float fft_base_value_128(I16 *src)
{
  float fft_rx;
  int i = 0;
  for (i = 0; i < 128; i++)
  {
    fft_rx += src[i];
  }
  return fft_rx / 128;
}

float fft_harmonic2_128(I16 *src)
{
  float rex = 0.0;
  float zex = 0.0;

  rex = getDFTnRX(src, 2);
  zex = getDFTnTX(src, 2);

  return sqrtf(rex* rex + zex * zex);
}

float fft_harmonic3_128(I16 *src)
{
  float rex = 0.0;
  float zex = 0.0;

  rex = getDFTnRX(src, 3);
  zex = getDFTnTX(src, 3);

  return sqrtf(rex* rex + zex * zex);
}


float fft_harmonic4_128(I16 *src)
{
  float rex = 0.0;
  float zex = 0.0;

  rex = getDFTnRX(src, 4);
  zex = getDFTnTX(src, 4);

  return sqrtf(rex* rex + zex * zex);
}


float fft_harmonic5_128(I16 *src)
{
  float rex = 0.0;
  float zex = 0.0;

  rex = getDFTnRX(src, 5);
  zex = getDFTnTX(src, 5);

  return sqrtf(rex* rex + zex * zex);
}

float fft_harmonic6_128(I16 *src)
{
  float rex = 0.0;
  float zex = 0.0;

  rex = getDFTnRX(src, 6);
  zex = getDFTnTX(src, 6);

  return sqrtf(rex* rex + zex * zex);
}

float fft_harmonic7_128(I16 *src)
{
  float rex = 0.0;
  float zex = 0.0;

  rex = getDFTnRX(src, 7);
  zex = getDFTnTX(src, 7);

  return sqrtf(rex* rex + zex * zex);
}

/************************************************************************/
/*


*/
/************************************************************************/
stuComplex fft_math_half(I16 *src, short THD)
{
  float Rex = 0.0;
  float Zex = 0.0;
  stuComplex data;
  Rex = getDFTnRX_half(src, THD);
  Zex = getDFTnTX_half(src, THD);
  data.imagin = Zex;
  data.real = Rex;
  return data;
}
/************************************************************************/
/*
基波对称分量计算
A/B/C为对应的基波分量
*/
/************************************************************************/
stuComplex fun_component_F0(stuComplex A, stuComplex B, stuComplex  C)
{
  //stuComplex compex;
  stuComplex p;
  p = add_compux(B, C);
  p = add_compux(p, A);
  p.imagin = p.imagin / 3;
  p.real = p.real / 3;
  return p;
}
/************************************************************************/
/*


*/
/************************************************************************/
stuComplex fun_component_F1(U8 phase, stuComplex A, stuComplex B, stuComplex  C)
{
  //stuComplex compex;
  stuComplex p;
  stuComplex a2;
  stuComplex a;
  a.imagin = 0.866;
  a.real = -0.5;
  a2 = mul_compux(a, a);
  switch (phase)
  {
  case 0:
    p = add_compux(mul_compux(a, B), mul_compux(a2, C));
    p = add_compux(p, A);
    p.imagin = p.imagin / 3;
    p.real = p.real / 3;
    break;
  case 1:
    p = add_compux(mul_compux(a, C), mul_compux(a2, A));
    p = add_compux(p, B);
    p.imagin = p.imagin / 3;
    p.real = p.real / 3;
    break;
  case 2:
    p = add_compux(mul_compux(a, A), mul_compux(a2, B));
    p = add_compux(p, C);
    p.imagin = p.imagin / 3;
    p.real = p.real / 3;
    break;
  default:
    break;
  }
  return p;
}
stuComplex fun_component_F2(U8 phase, stuComplex A, stuComplex B, stuComplex  C)
{
 // stuComplex compex;
  stuComplex p;
  stuComplex a2;
  stuComplex a;
  a.imagin = 0.866;
  a.real = -0.5;
  a2 = mul_compux(a, a);
  switch (phase)
  {
  case 0:
    p = add_compux(mul_compux(a, C), mul_compux(a2, B));
    p = add_compux(p, A);
    p.imagin = p.imagin / 3;
    p.real = p.real / 3;
    break;
  case 1:
    p = add_compux(mul_compux(a, A), mul_compux(a2, C));
    p = add_compux(p, B);
    p.imagin = p.imagin / 3;
    p.real = p.real / 3;
    break;
  case 2:
    p = add_compux(mul_compux(a, B), mul_compux(a2, A));
    p = add_compux(p, C);
    p.imagin = p.imagin / 3;
    p.real = p.real / 3;
    break;
  default:
    break;
  }
  return p;
}

void computeEffectiveValue(stuFFt *fft_data, stuAnalog *ptrParam)
{
  uint8_t i;
  //float basewave_value;
  float data = 0;
  U16 counts = fft_data->counts * 4;
  stuComplex FFT_datamag; //FFT计算出的基波值
#ifndef FFT_128_H
  I16 FFT_datain[32];
  memset(FFT_datain, 0, sizeof(FFT_datain));
  for (i = 0; i < 32; i++)
  {
    FFT_datain[i] = fft_data->ADC_sample[(8 * i + fft_data->channel) + fft_data->index * 128 * 2 + fft_data->offset];
  }
  FFT_datamag = fft_math_32(FFT_datain);
#else
  I16 FFT_datain[128];
  I16 FFT_datain1[128];
  int offsetPoint = 0;

  for (i = 0; i < 128; i++)
  {
    FFT_datain[i] = fft_data->ADC_sample[(2 * i + fft_data->channel) + fft_data->index * 128 * 2 + fft_data->offset];
  }

#endif

  if (fft_data->channel == vol_channel)
  {
    for (i = 0; i < 4; i++)
    {
      offsetPoint = 32 * (i + 1);
      memcpy(FFT_datain1, voltage_swap_adc_value +  offsetPoint, (128 - offsetPoint) * 2);
      memcpy(FFT_datain1 + 128 - offsetPoint, FFT_datain, offsetPoint * 2);

      FFT_datamag = fft_math_128(FFT_datain1, 1);

      if (ptrParam->amp_ptr == 0)
      {
        ptrParam->amp_ptr = 1;
      }
      data = sqrtf(FFT_datamag.real * FFT_datamag.real + FFT_datamag.imagin * FFT_datamag.imagin);

      voltage_value[counts]=  computeVoltageEffective(data, ptrParam->amp_ptr,ptrParam->zero_chk);
      
      counts++;
    }
    memcpy(voltage_swap_adc_value, FFT_datain, 256);
  }

  if (fft_data->channel == cur_channel)
  {
    for (i = 0; i < 4; i++)
    {
      offsetPoint = 32 * (i + 1);
      memcpy(FFT_datain1, current_swap_adc_value +  offsetPoint, (128 - offsetPoint) * 2);
      memcpy(FFT_datain1 + 128 - offsetPoint, FFT_datain, offsetPoint * 2);
      FFT_datamag = fft_math_128(FFT_datain1, 1);
      if (ptrParam->amp_ptr == 0)
      {
        ptrParam->amp_ptr = 1;
      }
      data = sqrtf(FFT_datamag.real * FFT_datamag.real + FFT_datamag.imagin * FFT_datamag.imagin);
     
      current_value[counts]=computeCurrentEffective(data,ptrParam->amp_ptr, ptrParam->zero_chk);
    
      counts++;
    }
    memcpy(current_swap_adc_value, FFT_datain, 256);
  }
}

/************************************************************************/
/* 判断线路状态                                                           */
/************************************************************************/
//const U8 lineState[64]={vol_none,vol_up};

static void judgeLinePowerStatus(float voltageCriticalValue, float currentCriticalValue, U8 count)
{
  float currentf1 =  current_value[((count + 11 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  float current   =  current_value[((count + 7 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  float currentb1 =  current_value[((count + 3 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  float voltagef1 =  voltage_value[((count + 11 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  float voltage   =  voltage_value[((count + 7 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  float voltageb1 =  voltage_value[((count+3 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  

  
  
  if ((voltagef1 <= voltageCriticalValue) && (voltage <= voltageCriticalValue) && (voltageb1 <= voltageCriticalValue))
  {
    if(currentf1 < currentCriticalValue && current < currentCriticalValue && currentb1 <currentCriticalValue)
    {
      fault_trip.vol_status = vol_none; //失电状态
      isLinePower = false;
    }
    /*else if (currentf1<currentCriticalValue && currentb1>currentCriticalValue)
    {
      fault_trip.vol_status = vol_down; //掉电状态，
      sal_dbg_printf(DEBUG_FAULT, "line_status: down :cf1:%f,c:%f,cb1:%f,vf1:%f,v:%f,vb1:%f\r\n", currentf1, current, currentb1, voltagef1, voltage, voltageb1);
    }*/
    else if(currentf1>currentCriticalValue && currentb1<currentCriticalValue)
    {
      fault_trip.vol_status = only_current_up; //只有电流的状态，并且上电
    }
    else
    {
       fault_trip.vol_status = only_current; //只有电流的状态
        isLinePower = true;

    }
  }
  else if ((voltagef1 > voltageCriticalValue) && (voltageb1 < voltageCriticalValue))
  {
    //无需判断电流，不管电流什么状态都认为是上电状态
    fault_trip.vol_status = vol_up; //上电状态
      isLinePower = true;
    sal_dbg_printf(DEBUG_FAULT, "line_status: up :cf1:%f,c:%f,cb1:%f,vf1:%f,v:%f,vb1:%f\r\n", currentf1, current, currentb1, voltagef1, voltage, voltageb1);
  }
  else if ((voltagef1 > voltageCriticalValue) && (voltage > voltageCriticalValue) && (voltageb1 > voltageCriticalValue))
  {
    if (currentf1 < currentCriticalValue && current < currentCriticalValue && currentb1 < currentCriticalValue)
    {
      fault_trip.vol_status = vol_none; //失电状态
      isLinePower = false;
    }
    else if(currentf1 < currentCriticalValue && currentb1 > currentCriticalValue)
    {
      fault_trip.vol_status = vol_down; //掉电状态
      sal_dbg_printf(DEBUG_FAULT, "line_status: down :cf1:%f,c:%f,cb1:%f,vf1:%f,v:%f,vb1:%f\r\n", currentf1, current, currentb1, voltagef1, voltage, voltageb1);
    }
    else if(currentf1 > currentCriticalValue && current > currentCriticalValue && currentb1 > currentCriticalValue)
    {
      fault_trip.vol_status = vol_normal; //有电状态
      isLinePower = true;
    }
    else
    {
       fault_trip.vol_status = vol_fault; //可能处于故障状态
       sal_dbg_printf(DEBUG_FAULT, "line_status: fault :cf1:%f,c:%f,cb1:%f,vf1:%f,v:%f,vb1:%f\r\n", currentf1, current, currentb1, voltagef1, voltage, voltageb1);
    }
  }
  else if ((voltagef1 < voltageCriticalValue) && (voltageb1 > voltageCriticalValue))
  {
    if (currentf1 < currentCriticalValue)
    {
      fault_trip.vol_status = vol_down; //掉电状态
      sal_dbg_printf(DEBUG_FAULT, "line_status: down :cf1:%f,c:%f,cb1:%f,vf1:%f,v:%f,vb1:%f\r\n", currentf1, current, currentb1, voltagef1, voltage, voltageb1);
    }
    else
    {
      //fault_trip.vol_status = vol_normal; //可能处于故障状态
      fault_trip.vol_status = vol_fault; //可能处于故障状态
      sal_dbg_printf(DEBUG_FAULT, "line_status: fault :cf1:%f,c:%f,cb1:%f,vf1:%f,v:%f,vb1:%f\r\n", currentf1, current, currentb1, voltagef1, voltage, voltageb1);
    }
  }
  else
  {
    fault_trip.vol_status = vol_unknown; //未知状态
    sal_dbg_printf(DEBUG_FAULT, "line_status: unkown :cf1:%f,c:%f,cb1:%f,vf1:%f,v:%f,vb1:%f\r\n", currentf1, current, currentb1, voltagef1, voltage, voltageb1);
  }
}





U8 doRealFaultPush(stufaultinfo *faultinfo)
{
   
  fault_msg_gloable.msg_len = sizeof(stuevent_t) + 4;
  fault_msg_gloable.msg_type = event_fault_id;
  fault_msg_gloable.msg_data[0] = self_group_id;
  fault_msg_gloable.msg_data[1] = self_phase_id;
  if (faultinfo->judgetype == cur_brk_serial)
  {
    fault_msg_gloable.msg_data[2] = 5; //信息类型
  }
  else if (faultinfo->judgetype == phase_brk_serial)
  {
    fault_msg_gloable.msg_data[2] = 3; //信息类型
  }
  else if (faultinfo->judgetype == vol_base_over_serial)
  {
    fault_msg_gloable.msg_data[2] = 3; //信息类型
  }
  else if (faultinfo->judgetype == vol_base_lower_serial)
  {
    fault_msg_gloable.msg_data[2] = 3; //信息类型
  }
  else
  {
    fault_msg_gloable.msg_data[2] = 5; //信息类型
  }
  fault_msg_gloable.msg_data[3] = 1; //数据
  memcpy(&fault_msg_gloable.msg_data[4], &faultinfo->event, sizeof(stuevent_t));

     // add_event_list(&faultinfo->event);
#ifndef WIN32
      msg_send(msg_manage, fault_msg_gloable);
      //msg_send(msg_zigbee, fault_msg_gloable);
      zigbee_add_event(fault_msg_gloable.msg_type, fault_msg_gloable.msg_data, fault_msg_gloable.msg_len);
      sal_dbg_printf(DEBUG_FAULT, "fault trip:group:%d,phase:%d,counts:%d,event_code:%d,value:%d,time:%d--%d--%d--%d--%d--%d--%d--%d\r\n",faultinfo->group,faultinfo->phase,
                     faultinfo->cycyle_serial, faultinfo->event.event_code, faultinfo->event.event_data, 
                     faultinfo->event.event_time.years, faultinfo->event.event_time.month, faultinfo->event.event_time.days, 
                     faultinfo->event.event_time.hours, faultinfo->event.event_time.mins, faultinfo->event.event_time.sec, faultinfo->event.event_time.MS, faultinfo->event.event_time.US);
#else
      printf("fault trip:group:%d,phase:%d,channel:%d,counts:%d,event_code:%d,value:%d,time:%d--%d--%d--%d--%d--%d--%d--%d\r\n", faultinfo->group, faultinfo->phase,
             faultinfo->channel_type, faultinfo->cycyle_serial, faultinfo->event.event_code, faultinfo->event.event_data, 
             faultinfo->event.event_time.years, faultinfo->event.event_time.month, faultinfo->event.event_time.days, 
             faultinfo->event.event_time.hours, faultinfo->event.event_time.mins, faultinfo->event.event_time.sec, faultinfo->event.event_time.MS, faultinfo->event.event_time.US);
#endif
      return true;
   


}

/*void record_wave_fix(void)
{
  stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
  U32 fix_time;
  U32 fix_time_t;
  msg_t msg;
  stuevent_t event_data;
  fix_time = p->record_hour * 3600;
  fix_time += p->record_min * 60;
  fix_time += p->record_sec;
  stuSystime time;
  get_sys_time(&time);
  fix_time_t = time.hours * 3600;
  fix_time_t += time.mins * 60;
  fix_time_t += time.sec;
  if (fix_time == fix_time_t || (fix_time + 1) == fix_time_t)
  {
    event_data.event_code = event_definetime_id;
    msg.msg_type = event_definetime_id;
    event_data.event_time.hours = p->record_hour;
    event_data.event_time.mins = p->record_min;
    event_data.event_time.sec = p->record_sec;
    event_data.event_time.years = time.years;
    event_data.event_time.month = time.month;
    event_data.event_time.days = time.days;
    event_data.event_time.MS = 0;
    memcpy(msg.msg_data, &event_data, sizeof(stuevent_t));
    msg.msg_len = sizeof(stuevent_t);
#ifndef WIN32
    if (msg_manage == NULL)
    {
      msg_manage = msg_creat(msg_depth, sizeof(msg_t));
    }
    msg_send(msg_manage, msg);
    sal_delay_ms(2000);
#endif
  }
}*/

stuComplex fft_math_32(I16 *src)
{
  float Rex = 0.0;
  float Zex = 0.0;
  stuComplex data;
  Rex = (src[1] + src[15] - src[17] - src[31]) * fft_coe_sin[1];
  Rex += (src[2] + src[14] - src[18] - src[30]) * fft_coe_sin[2];
  Rex += (src[3] + src[13] - src[19] - src[29]) * fft_coe_sin[3];
  Rex += (src[4] + src[12] - src[20] - src[28]) * fft_coe_sin[4];

  Rex += (src[5] + src[11] - src[21] - src[27]) * fft_coe_sin[5];
  Rex += (src[6] + src[10] - src[22] - src[26]) * fft_coe_sin[6];
  Rex += (src[7] + src[9] - src[23] - src[25]) * fft_coe_sin[7];
  Rex += (src[8] - src[24]) * fft_coe_sin[8];
  Rex = Rex / 16;
  Zex = (src[1] + src[31] - src[15] - src[17]) * fft_coe_cos[1];
  Zex += (src[2] + src[30] - src[14] - src[18]) * fft_coe_cos[2];
  Zex += (src[3] + src[29] - src[13] - src[19]) * fft_coe_cos[3];
  Zex += (src[4] + src[28] - src[12] - src[20]) * fft_coe_cos[4];

  Zex += (src[5] + src[27] - src[11] - src[21]) * fft_coe_cos[5];
  Zex += (src[6] + src[26] - src[10] - src[22]) * fft_coe_cos[6];
  Zex += (src[7] + src[25] - src[9] - src[23]) * fft_coe_cos[7];
  Zex += (src[0] - src[16]) * fft_coe_cos[0];
  Zex = Zex / 16;
  data.imagin = Zex;
  data.real = Rex;
  // Rex = Rex * Rex + Zex * Zex;
  return data;
}

#define CURRENT_NORMAL                0
#define CURRENT_WEAK_BREAK            1
#define CURRENT_SMALL_BREAK           2
#define CURRENT_LARGE_BREAK           3
#define CURRENT_OVERLIMIT             4


static U8 currentBreakOrOverlimitFlg=false;
static U8 currentFaultType=CURRENT_NORMAL;

stujudge_t reclose_trip;



static U8 voltageDownBeakJudge( U8 counts)
{
 
  float trip_data = 0;
  float trip_datalast = 0;
  
 
 
  stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
  trip_data = voltage_value[(counts + 1 + judge_buff_length_quarter) % judge_buff_length_quarter];
  trip_datalast = voltage_value[((counts - 3 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  if (trip_datalast == 0) 
  {
    trip_datalast = (float)(p->analog_vol.critical_Value);
  }
  trip_data = trip_data - trip_datalast;
  trip_data /= trip_datalast;
  trip_data *= 100;
  if (fabs(trip_data) > p->analog_vol.param_break  && trip_data < 0) //相电场强度突变
  {
    trip_data = voltage_value[((counts + judge_buff_length_quarter) % judge_buff_length_quarter)];
    trip_datalast = voltage_value[((counts - 4 + judge_buff_length_quarter) % judge_buff_length_quarter)];
    
    if (trip_datalast == 0) 
    {
      trip_datalast = (float)(p->analog_vol.critical_Value);
    }
    trip_data = trip_data - trip_datalast;
    trip_data /= trip_datalast;
    trip_data *= 100;
    if (fabs(trip_data) > p->analog_vol.param_break && trip_data < 0) //前一周期相电场强度突变
    {
       return true;
      //sal_dbg_printf(DEBUG_FAULT, "judge info: voltage down break! base value:%f break value:%f count:%d param_break:%d\r\n", trip_datalast, trip_data, counts, p->analog_vol.param_break);
    }
    else
    {
      return false;
    }
  }
 return false;
}

static U8  currentWeakUpBreakJudge(U8 counts)
{
	
	float trip_data = 0;
	float trip_data1 = 0;
	trip_data = current_value[(counts + 6) % judge_buff_length_quarter];
	trip_data1 =  current_value[counts];
	trip_data = (trip_data - trip_data1);
	if (trip_data >= 50) //相电流突变上
	{
		trip_data = current_value[((counts +7+ judge_buff_length_quarter) % judge_buff_length_quarter)];
		trip_data1 = current_value[((counts +1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
		trip_data = (trip_data - trip_data1);
		if (trip_data >= 50)
		{
			return true;
			//sal_dbg_printf(DEBUG_FAULT, "current up break,base value:%f,break value:%f,count:%d\r\n", trip_data1, trip_data, counts);
		}
		else
		{
			return false;
		}
	}
	return false;
}
static U8  currentSmallUpBreakJudge( U8 counts);
static U8 currentUpBreakJudge2(U8 counts)
{
  
  float trip_data = 0;
  float trip_data1 = 0;
  stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
  
  float baseValue=0;
 
  baseValue =  current_value[counts];
  trip_data = current_value[((counts + 1 + judge_buff_length_quarter) % judge_buff_length_quarter)] - baseValue;
  trip_data1=baseValue-current_value[((counts - 1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  if(trip_data>5&&fabs(trip_data1)<5)
  {
    
        trip_data = current_value[((counts + 5 + judge_buff_length_quarter) % judge_buff_length_quarter)]-baseValue;
        trip_data1 =  current_value[((counts + 6 + judge_buff_length_quarter) % judge_buff_length_quarter)]-baseValue;
        
            if(trip_data>p->analog_cur.param_overload&&trip_data1>p->analog_cur.param_overload) 
            {
                 return CURRENT_LARGE_BREAK;
            
            }
            if(trip_data>p->analog_cur.param_break&&trip_data1>p->analog_cur.param_break) 
            {
                 return CURRENT_SMALL_BREAK;
            
            }
    
     
     
  }
  return CURRENT_NORMAL;

}

static U8 currentUpBreakJudge(U8 counts)
{
  
  float trip_data = 0;
  float trip_data1 = 0;
  stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
  
  U8 i=0;
  float baseValue=0;
 
  baseValue =  current_value[counts];
  trip_data = current_value[((counts + 1 + judge_buff_length_quarter) % judge_buff_length_quarter)] - baseValue;
  trip_data1=baseValue-current_value[((counts - 1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  if(trip_data>5&&fabs(trip_data1)<5)
  {

     for(i=2;i<=8;i++)
     {
        trip_data = current_value[((counts + i + judge_buff_length_quarter) % judge_buff_length_quarter)];
        trip_data1 =  current_value[((counts + i-1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
        trip_data = (trip_data - trip_data1);
        if(fabs(trip_data)<5)
        {
          
          
            
           trip_data =trip_data1-baseValue;
            
            if(trip_data>p->analog_cur.param_overload) 
            {
                 return CURRENT_LARGE_BREAK;
            
            }
            if(trip_data>p->analog_cur.param_break) 
            {
                 return CURRENT_SMALL_BREAK;
            
            }
        }
     
     
     }
  }
  return CURRENT_NORMAL;

}

static U8  currentLargeUpBreakJudge(U8 counts)
{
 
  float trip_data = 0;
  float trip_data1 = 0;
  
    stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
    


  trip_data = current_value[((counts + 6 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  trip_data1 =  current_value[counts];
  trip_data = (trip_data - trip_data1);
  
  
  if(trip_data > p->analog_cur.param_overload)
  {
     trip_data = current_value[((counts+7 + judge_buff_length_quarter) % judge_buff_length_quarter)];
    trip_data1 = current_value[((counts +1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
    trip_data = (trip_data - trip_data1);
    if (trip_data > p->analog_cur.param_overload)
    {
      return true;
    }
  }

  return false;
}



static U8  currentLargeDownBreakJudge(U8 counts)
{
 
  float trip_data = 0;
  float trip_data1 = 0;
  stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
  trip_data = current_value[(counts + 6) % judge_buff_length_quarter];
  trip_data1 =  current_value[counts];
  trip_data = (trip_data - trip_data1);
  if (fabs(trip_data) > p->analog_cur.param_overload && trip_data<0) //相电流突变上
  {
    trip_data = current_value[((counts + 7+judge_buff_length_quarter) % judge_buff_length_quarter)];
    trip_data1 = current_value[((counts +1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
    trip_data = (trip_data - trip_data1);
    if (fabs(trip_data) > p->analog_cur.param_overload && trip_data<0)
    {
      return true;
      //sal_dbg_printf(DEBUG_FAULT, "current up break,base value:%f,break value:%f,count:%d\r\n", trip_data1, trip_data, counts);
    }
    else
    {
      return false;
    }
  }
  return false;
}

static U8  currentSmallUpBreakJudge( U8 counts)
{
  
  float trip_data = 0;
  float trip_data1 = 0;
  stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];

  
  trip_data = current_value[((counts+6 + judge_buff_length_quarter) % judge_buff_length_quarter)];
  trip_data1 =  current_value[counts];
  trip_data = (trip_data - trip_data1);
  if (trip_data > p->analog_cur.param_break) //相电流突变上
  {
    trip_data = current_value[((counts+7 + judge_buff_length_quarter) % judge_buff_length_quarter)];
    trip_data1 = current_value[((counts +1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
    trip_data = (trip_data - trip_data1);
    if (trip_data > p->analog_cur.param_break)
    {  
          return true;
      //sal_dbg_printf(DEBUG_FAULT, "current up break,base value:%f,break value:%f,count:%d\r\n", trip_data1, trip_data, counts);
    }
   
  }
  return false;
}

static U8  currentSmallDownBreakJudge(U8 counts)
{
  
  float trip_data = 0;
  float trip_data1 = 0;

  stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
  trip_data = current_value[(counts + 6) % judge_buff_length_quarter];
  trip_data1 =  current_value[counts ];
  trip_data = (trip_data - trip_data1);
  if (fabs(trip_data) > p->analog_cur.param_break && trip_data<0) //相电流突变上
  {
    trip_data = current_value[((counts +7+ judge_buff_length_quarter) % judge_buff_length_quarter)];
    trip_data1 = current_value[((counts +1+ judge_buff_length_quarter) % judge_buff_length_quarter)];
    trip_data = (trip_data - trip_data1);
    if (fabs(trip_data) > p->analog_cur.param_break&&trip_data<0)
    {
      return true;
      //sal_dbg_printf(DEBUG_FAULT, "current up break,base value:%f,break value:%f,count:%d\r\n", trip_data1, trip_data, counts);
    }
    else
    {
      return false;
    }
  }
  return false;
}

static U8 currentOverLimitJudge(U8 counts)
{

  float trip_data = 0;
  stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
  trip_data = current_value[counts ];
    if (trip_data > p->analog_cur.param_fault_up) //相电流幅值越限
    {
      trip_data = current_value[((counts+ 1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
      if (trip_data > p->analog_cur.param_fault_up)
      {
        //sal_dbg_printf(DEBUG_FAULT, "cur up,value:%f,count:%d\r\n", trip_data, counts);
       return true; //相电流越限
      }
    
    }
   
  return false;
}

static U8 currentOverLimitRecoveryJudge(U8 counts)
{

  float trip_data = 0;
  stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
  trip_data = current_value[counts];
    if (trip_data < p->analog_cur.param_fault_up* p->analog_cur.up_back_slope) //相电流幅值越限
    {
      trip_data = current_value[((counts + 1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
      if (trip_data < p->analog_cur.param_fault_up* p->analog_cur.up_back_slope)
      {
        //sal_dbg_printf(DEBUG_FAULT, "cur up,value:%f,count:%d\r\n", trip_data, counts);
       return true; //相电流越限
      }
     
    }
   
  return false;
}




static U8 voltageOverLimitJudge(U8 counts)
{
    float trip_data = 0;
    stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
    trip_data = voltage_value[counts % judge_buff_length_quarter];
    
    if (trip_data > p->analog_vol.param_fault_up) //相电压幅值越上限
    {
      trip_data = voltage_value[((counts +1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
      if (trip_data > p->analog_vol.param_fault_up)
      {
        //sal_dbg_printf(DEBUG_FAULT, "vol up,value:%f,count:%d\r\n", trip_data, counts);
         return true; //相电压幅值越上限
      }
   
    }
    return false;
}

static U8 voltageOverLimitRecoveryJudge(U8 counts)
{
    float trip_data = 0;
    stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
    if (trip_data < (float)(p->analog_vol.param_fault_up * p->analog_vol.up_back_slope)) //相电压幅值上越限恢复
    {
      trip_data = voltage_value[((counts + 1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
      if (trip_data < (float)(p->analog_vol.param_fault_up * p->analog_vol.up_back_slope))
      {
         //sal_dbg_printf(DEBUG_FAULT, "vol up recover,value:%f,count:%d\r\n", trip_data, counts);
         return true; //相电压越限恢复
      }
    }
    return  false;
}



static U8 voltageUnderLimitJudge(U8 counts)
{
    float trip_data = 0;
    stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
    trip_data = voltage_value[counts % judge_buff_length_quarter];
    if (trip_data <= p->analog_vol.param_fault_low) //相电流幅值越限
    {
      trip_data = voltage_value[((counts + 1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
      if (trip_data <= p->analog_vol.param_fault_low)
      {
        //sal_dbg_printf(DEBUG_FAULT, "vol low,value:%f,count:%d\r\n", trip_data, counts);
        return true; //相电压越下限
      }
     
    }
    return false;
}

static U8 voltageUnderLimitRecoveryJudge(U8 counts)
{
    float trip_data = 0;
    stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
    trip_data = voltage_value[counts % judge_buff_length_quarter];
    if (trip_data > (float)(p->analog_vol.param_fault_low * p->analog_vol.low_back_slope)) //相电压幅值下越限恢复
    {
      trip_data = voltage_value[((counts + 1 + judge_buff_length_quarter) % judge_buff_length_quarter)];
      if (trip_data > (float)(p->analog_vol.param_fault_low * p->analog_vol.low_back_slope))
      {
        //sal_dbg_printf(DEBUG_FAULT, "vol low recover,value:%f,count:%d\r\n", trip_data, counts);
        return true; //相电压下越限恢复
      }
    }
    return  false;
}


static void pushInstantaneousFault(stuSystime *time_t)
{
    stuSystime pcptime;
    msg_t trip_msg;
#ifndef WIN32
          
          sal_dbg_printf(DEBUG_FAULT, "unit happen transient faults,fault current:%d group:%d phase:%d %04u/%02u/%02u %02u:%02u:%02u:%03u:%03u\r\n", 
                         fault_trip.event_data.event_data, self_group_id, self_phase_id,
                         time_t->years + 2000, time_t->month, time_t->days, 
                         time_t->hours, time_t->mins, time_t->sec, time_t->MS, time_t->US);
          trip_msg.msg_type = event_fault_id;
          trip_msg.msg_len = 2;
          trip_msg.msg_data[0] = 1; //故障
          trip_msg.msg_data[1] = 1; //瞬时性故障
          trip_msg.msg_data[2] = 0x00;
          msg_send(msg_ctrl, trip_msg);
          trip_msg.msg_type = event_fault_transient_id;
          trip_msg.msg_len = 10;
          trip_msg.msg_data[0] = self_group_id; //故障组
          trip_msg.msg_data[1] = self_phase_id; //故障相
          trip_msg.msg_data[2] = 1; //发生或者解除故障
          UtcTosystime(fault_trip.fault_start_time, &pcptime);
          memcpy(&trip_msg.msg_data[3], &pcptime, sizeof(stuSystime));
          if (coll_flag==true)
          {
            msg_send(msg_gprs, trip_msg);
          }
          else
          {
            //msg_send(msg_zigbee, trip_msg);
            zigbee_add_event(trip_msg.msg_type, trip_msg.msg_data, trip_msg.msg_len);
          }
#else
          cout << "瞬时性故障，故障值:" << fault_trip.event_data.event_data << endl;
#endif
}
static void pushPermanentFault(stuSystime *time_t)
{
  stuSystime pcptime;
  msg_t trip_msg;
#ifndef WIN32           
           
            sal_dbg_printf(DEBUG_FAULT, "unit happen permanent faults,fault current:%d group:%d phase:%d %04u/%02u/%02u %02u:%02u:%02u:%03u:%03u\r\n", 
                           fault_trip.event_data.event_data, self_group_id, self_phase_id,
                           time_t->years + 2000, time_t->month, time_t->days, 
                           time_t->hours, time_t->mins, time_t->sec, time_t->MS, time_t->US);
            trip_msg.msg_type = event_fault_id;
            trip_msg.msg_len = 2;
            trip_msg.msg_data[0] = 1; //故障
            trip_msg.msg_data[1] = 2; //永久性故障
            trip_msg.msg_data[2] = 0x00;
            msg_send(msg_ctrl, trip_msg);
            trip_msg.msg_type = event_fault_permanent_id;
            trip_msg.msg_len = 10;
            trip_msg.msg_data[0] = self_group_id; //故障组
            trip_msg.msg_data[1] = self_phase_id; //故障相
            trip_msg.msg_data[2] = 1; //发生或者解除故障
            UtcTosystime(fault_trip.fault_start_time, &pcptime);
            memcpy(&trip_msg.msg_data[3], &pcptime, sizeof(stuSystime));
            if (coll_flag==true)
            {
              msg_send(msg_gprs, trip_msg);
            }
            else
            {
              //msg_send(msg_zigbee, trip_msg);
              zigbee_add_event(trip_msg.msg_type, trip_msg.msg_data, trip_msg.msg_len);
            }
#else
            cout << "永久性故障，故障值:" << fault_trip.event_data.event_data << endl;
#endif

}
static void judgeInstantOrPermanent(U8 count, stuSystime *time_t,U32 sec,U16 ms)
{
  U32 trip_time = 0;

  if (false == fault_trip.fault_delay_start_flag) //故障延时启动标志
  {
    fault_trip.fault_start_time.sec =sec;
    fault_trip.fault_start_time.ms = ms;
    fault_trip.fault_delay_start_flag = true;
    fault_trip.event_data.event_data = (U16)current_value[count];
  }
  else
  {
    trip_time = (sec - fault_trip.fault_start_time.sec) * 1000 + ms - fault_trip.fault_start_time.ms;
    if (trip_time >= collparam.act_delay) //故障延时时间到,判断是临时性故障or永久性故障 ms
    {
      //if (fault_trip.vol_status == vol_normal||fault_trip.vol_status == vol_up||fault_trip.vol_status==only_current) //有压
      if ( isLinePower==true) //有电或者有电流
      {
         trip_start_flag[self_phase_id] = false;     
         fault_trip.fault_delay_start_flag = false;
         fault_trip.trans_fault_flag = true;
         pushInstantaneousFault(time_t);
        
      }
      else //if (fault_trip.vol_status == vol_none||fault_trip.vol_status == vol_fault) //延时时间到，线路无压
      {

          trip_start_flag[self_phase_id] = false;
          fault_trip.fault_delay_start_flag = false;
           fault_trip.per_fault_flag = true;
           pushPermanentFault(time_t);
         
       
      }
    }
    else //故障延时时间未到
    {}
  }
}


static stufaultinfo globalFaultInfo;
static U8 doCurrentLargeUpBreakRecord(U8 count, stuSystime *time_t)
{

         stuevent_t event;


          event.dev_id = unitparam.dev_id;
          event.event_data = (U16)current_value[(count + 2) % judge_buff_length_quarter];         
          event.event_code = CUR_BREAK;
          
          
          globalFaultInfo.group = self_group_id;
          globalFaultInfo.phase = self_phase_id;
          globalFaultInfo.judgetype = cur_brk_serial;
           globalFaultInfo.eventTimeMs=LocalTime_to_Utc(time_t)*1000+time_t->MS;       
          globalFaultInfo.cycyle_serial = (count + 2) % judge_buff_length_quarter;
         //memcpy(&event.event_time.years, &time_t->years, sizeof(stuSystime));
         time_add_ms(time_t, &event.event_time, 8);
        // time_add_ms(time_t, &event.event_time, 18);
           
          memcpy(&globalFaultInfo.event.dev_id, &event.dev_id, sizeof(stuevent_t));
        
      return true;
}
static U8 doCurrentSmallUpBreakRecord(U8 count, stuSystime *time_t)
{
             stuevent_t event;


              event.dev_id = unitparam.dev_id;         
              event.event_code = CUR_BREAK;
              event.event_data = (U16)current_value[(count + 2) % judge_buff_length_quarter];
         
              globalFaultInfo.group = self_group_id;
              globalFaultInfo.phase = self_phase_id;
              globalFaultInfo.eventTimeMs=LocalTime_to_Utc(time_t)*1000+time_t->MS;
              globalFaultInfo.judgetype = cur_brk_serial;
              globalFaultInfo.cycyle_serial = (count + 2) % judge_buff_length_quarter;
            //  memcpy(&event.event_time.years, &time_t->years, sizeof(stuSystime));
               time_add_ms(time_t, &event.event_time, 8);
            // time_add_ms(time_t, &event.event_time, 18);
              memcpy(&globalFaultInfo.event.dev_id, &event.dev_id, sizeof(stuevent_t));

    return true;
}


static U8 doCurrentOverLimitRecord(U8 count, stuSystime *time_t)
{
 
      stuevent_t event;

      event.dev_id = unitparam.dev_id;  
      event.event_code = CUR_OVER;  
      event.event_data = (U16)current_value[count];
      
      globalFaultInfo.judgetype = cur_base_over_serial;
      globalFaultInfo.group = self_group_id;
      globalFaultInfo.phase = self_phase_id; 
      globalFaultInfo.eventTimeMs=LocalTime_to_Utc(time_t)*1000+time_t->MS;
      globalFaultInfo.cycyle_serial = count;
      memcpy(&event.event_time.years, &time_t->years, sizeof(stuSystime));
      memcpy(&globalFaultInfo.event.dev_id, &event.dev_id, sizeof(stuevent_t));
    
      return true;
}


static U8 pushFault()
{
  
     stuUnitparam *p;  
     
     msg_t msg; 
      
    if (coll_event.unit_event[self_group_id][self_phase_id].event_num >= ZIGBEE_EVENT_NUM && READ_BIT(collstatus.Unit_dev_info[self_group_id][self_phase_id].Comm_status, 0x01)==0x00) //汇集兼采集单元装置一个轮询周期内最多只能触发3次
    {
       return false;
    }
    else
    {
         fault_trip_num++;
         doRealFaultPush(&globalFaultInfo);
      
         p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
         trip_start_flag[self_phase_id] = true; //故障启动标志，手动或自动复位
        // sal_dbg_printf(DEBUG_FAULT, "fault: sleep current over 1! group:%d phase:%d vol:%f cur:%f\r\n", group, phase, voltage_value[count], current_value[count]);
        /*  sal_dbg_printf(DEBUG_FAULT, "fault: voltage down break! group:%d phase:%d vol:%f cur:%f trip_data:%f trip_datalast:%f\r\n", 
                       group, phase, voltage_value[count], current_value[count], voltage_value[(count + 1 + judge_buff_length_quarter) % judge_buff_length_quarter], 
                       voltage_value[((count - 3 + judge_buff_length_quarter) % judge_buff_length_quarter)]);*/
         
         if ((p->analog_vol.param_enable & FAULT_LINGT_MASK))
         {
          msg.msg_data[0] = self_group_id;
          msg.msg_data[1] = self_phase_id;
          msg.msg_data[2] = 0;
          msg.msg_data[3] = 2;
          msg.msg_type = event_remote_id; //event_remote_id, //远程控制
          msg_send(msg_ctrl, msg);
        }
      
      return true;
    }
    
       

}

/*********************************************************
电流判断基本思想：电流从上突变开始到下突变，如果时间未到设
定时间，就认为是故障；如果没有发生突变，且时间超过了设定时间
，那么认为是非故障。
**********************************************************/

static U8 doCurrentFaultJudge(U8 count, stuSystime *time_t)
{
  
 
 
    U8 result=false;
    result= currentUpBreakJudge(count);
    if(result==CURRENT_LARGE_BREAK)
    {
      doCurrentLargeUpBreakRecord(count,time_t);
      currentBreakOrOverlimitFlg=true;
      currentFaultType=CURRENT_LARGE_BREAK;
      return true;
    }
    
    if(result==CURRENT_SMALL_BREAK)
    {
      doCurrentSmallUpBreakRecord(count,time_t);      
      currentBreakOrOverlimitFlg=true;
      currentFaultType=CURRENT_SMALL_BREAK;
      return true;
    }


    /*result=currentOverLimitJudge(count); //相电流越限
    if(result)
    {
       doCurrentOverLimitRecord(count, time_t);
       currentBreakOrOverlimitFlg=true;
       currentFaultType=CURRENT_OVERLIMIT;
       return true;
    }*/
   currentBreakOrOverlimitFlg=false;
   currentFaultType=CURRENT_NORMAL;
   return false;
}

static U8 doHavePowerFaultJudge(U8 count, stuSystime *time_t)
{
  stuevent_t event;

    U8 result = false;
 
  
    result = voltageDownBeakJudge(count); //相电压突变
    if (result==true) //电压负荷值向下突变
    {
      
      event.dev_id = unitparam.dev_id;     
      event.event_code = VOL_BREAK;
      event.event_data = (U16)current_value[count];
      faultVoltage=voltage_value[count];
            
      globalFaultInfo.judgetype = vol_base_lower_serial;
      globalFaultInfo.group = self_group_id;
      globalFaultInfo.phase = self_phase_id; 
      globalFaultInfo.eventTimeMs=LocalTime_to_Utc(time_t)*1000+time_t->MS;
      globalFaultInfo.cycyle_serial = count;
      memcpy(&event.event_time.years, &time_t->years, sizeof(stuSystime));
      memcpy(&globalFaultInfo.event.dev_id, &event.dev_id, sizeof(stuevent_t));

      
       result= pushFault();
       if(result==true)
         return true;
      
    
    }


   return false;
}


static U8 doCurrentBreakHandle(U8 count,stuSystime *time_t)
{
   stuUnitparam *p = &collparam.param_group[self_group_id].unit_param[self_phase_id];
    U8 result=false;
     unsigned long nowTimeMs=0;

     
         nowTimeMs=LocalTime_to_Utc(time_t)*1000+time_t->MS;
         if(currentFaultType==CURRENT_LARGE_BREAK)
         {
             result=currentLargeDownBreakJudge(count);
              if(result)
              {
                 if((nowTimeMs-globalFaultInfo.eventTimeMs)<p->largeFaultCurrentConitunsTime)
                 {
                    result= pushFault(); 
                    if(result)
                    {
                        currentBreakOrOverlimitFlg=false;
                        currentFaultType=CURRENT_NORMAL;
                        return true;
                    
                    }
                    else
                    {
                        currentBreakOrOverlimitFlg=false;
                         currentFaultType=CURRENT_NORMAL;
                        return false;
                    }
                 }
              }
              if((nowTimeMs-globalFaultInfo.eventTimeMs)>=p->largeFaultCurrentConitunsTime)
              {
                 currentBreakOrOverlimitFlg=false;
                  currentFaultType=CURRENT_NORMAL;
                 return false;
              }
         }
         else if(currentFaultType==CURRENT_SMALL_BREAK)
         {
              result=currentSmallDownBreakJudge(count);
              if(result)
              {
                 if((nowTimeMs-globalFaultInfo.eventTimeMs)<p->smallFaultCurrentConitunsTime)
                 {
                    result= pushFault(); 
                    if(result)
                    {
                        currentBreakOrOverlimitFlg=false;
                        currentFaultType=CURRENT_NORMAL;
                        return true;
                    
                    }
                    else
                    {
                        currentBreakOrOverlimitFlg=false;
                        currentFaultType=CURRENT_NORMAL;
                        return false;
                    }
                 }
              }
              if((nowTimeMs-globalFaultInfo.eventTimeMs)>=p->smallFaultCurrentConitunsTime)
              {
                  currentBreakOrOverlimitFlg=false;
                 currentFaultType=CURRENT_NORMAL;
                 return false;
              }
                  
           
         }
 
        
        return false;
}

static U8 doFaultJudage( U8 count, stuSystime *time_t)
{
  
    U8 result=false;
   static U8 isCurrentUpDelay=false;  
   static U8 currentUpDelayCycleNum=0;
   if(isCurrentUpDelay==false)
   {
      if(fault_trip.vol_status == only_current_up||fault_trip.vol_status == vol_up) 
      {
        isCurrentUpDelay=true;
        currentUpDelayCycleNum=24;//32;//可以延迟6个周波
        return false;
      }
   
   }
   else
   {
    
       if(currentUpDelayCycleNum>0)
       {
           currentUpDelayCycleNum--;
           return false;
     
       }
       isCurrentUpDelay=false;
   }
   
   


    if(currentBreakOrOverlimitFlg==false)
    {
      if(fault_trip.vol_status == only_current)
        doCurrentFaultJudge(count,time_t);
    }
    
   if(currentBreakOrOverlimitFlg==true)
   {
        /* if(currentFaultType==CURRENT_OVERLIMIT)//如果是越限类型，就立刻触发
         {
              result= pushFault();//推送故障成功
           
              if(result)
              {
                 currentBreakOrOverlimitFlg=false;//恢复故障状态
                 currentFaultType=CURRENT_NORMAL;
                 return true; //发送成功了，返回true，停止故障判断
              }
              else
              {
                  currentBreakOrOverlimitFlg=false;
                  currentFaultType=CURRENT_NORMAL;
                  return false;//缓冲区满，发送失败，返回false，继续执行故障判断
              }
         }*/
         
         result= doCurrentBreakHandle(count,time_t); 
         if(result)
           return true;
         return false;
   }
  

         

     
 // if (fault_trip.vol_status == vol_up) //线路上电过程，注意装置正好放到有故障的线路上
 // {
//	  return false;
 // }
  
  if (fault_trip.vol_status == vol_unknown) //未知过程
  {
	  return false;
  }
  
  if (fault_trip.vol_status == vol_down) //线路掉电过程
  {
	  result=currentWeakUpBreakJudge(count);//用于抓取单个周波
	  if(result!=true)		  
	  return  false;
  }

  if (fault_trip.vol_status == vol_normal||fault_trip.vol_status == vol_down||
      fault_trip.vol_status==vol_fault) //线路电压正常
  {
        result= doHavePowerFaultJudge(count, time_t);
        if(result)
          return true;
  
  }
 

 
  return false;
}



static U8 exceedLimitNotify(U8 count, stuSystime *time_t,U8 type,U8 state)
{

      msg_t msg;
      msg.msg_data[0] = self_group_id;
      msg.msg_data[1] = self_phase_id;
      msg.msg_data[2] = state; //状态数据//带品质描述词的单/双点信息BS1[1]<0..1> 0:=开 1:=合
      memcpy((U8 *)&msg.msg_data[3], &time_t->years, sizeof(stuSystime));
      msg.msg_len = sizeof(stuSystime) + 3;
      msg.msg_type = type; //超限类型
      if (coll_flag==true)
      {
        msg_send(msg_gprs, msg);
      }
      else
      {
        //msg_send(msg_zigbee, msg);
        zigbee_add_event(msg.msg_type, msg.msg_data, msg.msg_len);
      }
      return true;

}
#define STATE_CURRENT_OVER_LIMIT                   1
#define STATE_CURRENT_OVER_LIMIT_RECOVERY          0
#define STATE_VOLTAGE_OVER_LIMIT                   1
#define STATE_VOLTAGE_OVER_LIMIT_RECOVERY          0
#define STATE_VOLTAGE_UNDER_LIMIT                  1
#define STATE_VOLTAGE_UNDER_LIMIT_RECOVERY         0


#define OVER_LIMIT_JUDGE_INTERVAL 60

static void doCurrentOverlimitJudge(U8 count, stuSystime *time_t,U32 nowSeconds)
{
  U8 result;
  static U32 lastCurrentOverlimitOrRecoveryTimeSec=0;
  
    if(event_cur_over_flag)//相电流越上限通知
    {
      
      if((nowSeconds-lastCurrentOverlimitOrRecoveryTimeSec)>OVER_LIMIT_JUDGE_INTERVAL)
      {
         result= currentOverLimitRecoveryJudge(count);
         if(result)
         {
           event_cur_over_flag=false;
           exceedLimitNotify(count, time_t,event_fault_cur_over_id,STATE_CURRENT_OVER_LIMIT_RECOVERY);
           lastCurrentOverlimitOrRecoveryTimeSec=nowSeconds;
         }
      }
 
    }
    else
    {
      if((nowSeconds-lastCurrentOverlimitOrRecoveryTimeSec)>OVER_LIMIT_JUDGE_INTERVAL)
      {
          result= currentOverLimitJudge(count);
          if(result)
          {
             event_cur_over_flag=true;
             exceedLimitNotify(count, time_t,event_fault_cur_over_id,STATE_CURRENT_OVER_LIMIT); 
             lastCurrentOverlimitOrRecoveryTimeSec=nowSeconds;
          }
       
      }
    }

}

static void doVoltageOverlimitJudge(U8 count, stuSystime *time_t,U32 nowSeconds)
{ 
  U8 result;
   static U32 lastVoltageOverlimitOrRecoveryTimeSec=0;
  if(event_vol_over_flag)//相电压越上限通知
    {
      
      if((nowSeconds-lastVoltageOverlimitOrRecoveryTimeSec)>OVER_LIMIT_JUDGE_INTERVAL)
      {
         result= voltageOverLimitRecoveryJudge(count);
         if(result)
         {
           event_vol_over_flag=false;
           exceedLimitNotify(count, time_t,event_fault_vol_over_id,STATE_VOLTAGE_OVER_LIMIT_RECOVERY);
           lastVoltageOverlimitOrRecoveryTimeSec=nowSeconds;
         }
      }
 
    }
    else
    {
       if((nowSeconds-lastVoltageOverlimitOrRecoveryTimeSec)>OVER_LIMIT_JUDGE_INTERVAL)
       {
          result= voltageOverLimitJudge(count);
          if(result)
          {
             event_vol_over_flag=true;
             exceedLimitNotify(count, time_t,event_fault_vol_over_id,STATE_VOLTAGE_OVER_LIMIT);
             lastVoltageOverlimitOrRecoveryTimeSec=nowSeconds;
          }
       }

    }
}
static void doVoltageUnderlimitJudge(U8 count, stuSystime *time_t,U32 nowSeconds)
{ 
  U8 result;
  static U32 lastVoltageUnderlimitOrRecoveryTimeSec=0;
  if(event_vol_under_flag)//相电压越下限通知
    {
       if((nowSeconds-lastVoltageUnderlimitOrRecoveryTimeSec)>OVER_LIMIT_JUDGE_INTERVAL)
      {
         result= voltageUnderLimitRecoveryJudge(count);
         if(result)
         {
           event_vol_under_flag=false;
           exceedLimitNotify(count, time_t,event_fault_vol_under_id,STATE_VOLTAGE_UNDER_LIMIT_RECOVERY);
           lastVoltageUnderlimitOrRecoveryTimeSec=nowSeconds;
         }
      }
 
    }
    else
    {
      if((nowSeconds-lastVoltageUnderlimitOrRecoveryTimeSec)>OVER_LIMIT_JUDGE_INTERVAL)
      {
         result= voltageUnderLimitJudge(count);
         if(result)
         {
             event_vol_under_flag=true;
             exceedLimitNotify(count, time_t,event_fault_vol_under_id,STATE_VOLTAGE_UNDER_LIMIT);
             lastVoltageUnderlimitOrRecoveryTimeSec=nowSeconds;
         }
      }

    }
}


/*void handle_fault_event(stuSystime event_time)
{
  msg_t msg_data;
  msg_data.msg_type = event_handle_id;
  msg_data.msg_len = sizeof(stuSystime) + 3;
  msg_data.msg_data[0] = self_group_id;
  msg_data.msg_data[1] = self_phase_id;
  msg_data.msg_data[2] = Signal_curr_over_addr;
  msg_data.msg_data[3] = true;
  if (coll_flag)
  {
    memcpy(&msg_data.msg_data[4], &event_time, sizeof(stuSystime));
    //msg_send(msg_gprs, msg_data);
  }
}*/













/*********************************************
采集单元触发故障判断
fftdata,判断相位的数据源，
*********************************************/


static bool enableFaultJudgeFlag=true;
static unsigned long  begintimeMs;
static U32 faultJudgeTimeout;

void diableFaultJudge(stuSystime *time,U32 timeout)
{
  enableFaultJudgeFlag=false;
  begintimeMs=LocalTime_to_Utc(time)  * 1000 + time->MS;
  faultJudgeTimeout=timeout;
}

void enableFaultJudge()
{
  enableFaultJudgeFlag=true;
}
 
void signal_phase_judge(stuFFt *fft_data)
{
  stuSystime time;
  static unsigned long lastFaultTimeMs=0;
    unsigned  long nowTimeMs = 0;
   

  static U8 isAllowFaultJudage=true;
   U8 resultFault=false;
  U32 nowSeconds=0;
  U16 nowMs=0; 
  
  U8 counts = fft_data->counts * 4;
  uint8_t i;
  stuAnalog *pv = &collparam.param_group[fft_data->group].unit_param[fft_data->phase].analog_vol;
  stuAnalog *pc = &collparam.param_group[fft_data->group].unit_param[fft_data->phase].analog_cur;
  judgeLinePowerStatus((float)(pv->critical_Value), (float)(pc->critical_Value), counts); //电压临界值 电流临界值

  
  if(GPRS_file_status.file_status == 5)
    return;
  
  
  

  for (i = 0; i < 4; i++)
  { 
      //time_add_ms(time_t, &time, wave_period/4);
      time_add_ms(&fft_data->ffttime, &time, i * 5);
      nowTimeMs = LocalTime_to_Utc(&time)  * 1000 + time.MS ;
      
    if(enableFaultJudgeFlag==false)
    {
        if((nowTimeMs-begintimeMs)>faultJudgeTimeout)
        {
           enableFaultJudgeFlag==true;
        }
    
       return;
    }
        //故障判断
      if (isAllowFaultJudage == true)
      {        
         resultFault= doFaultJudage(counts, &time);
         if(resultFault==true)
         {
           
            isAllowFaultJudage=false;        
            lastFaultTimeMs=nowTimeMs;
         }
       }
       else
       {
         if ((nowTimeMs-lastFaultTimeMs) >= collparam.recorder_wave_time)
         {
            
            isAllowFaultJudage = true;          
         }
         else if ((nowTimeMs-lastFaultTimeMs )< 0)
         {
             isAllowFaultJudage = true;
         }
       }
     nowSeconds=nowTimeMs/1000; 
     nowMs=nowTimeMs%1000;
     //永久性瞬时性判断             
    if (trip_start_flag[fft_data->phase] == true)
    {
     /* if(fault_trip.vol_status == vol_normal){
          if(fabs(voltage_value[counts]/faultVoltage)<1.2)//表示故障未恢复，
		      {
			       fault_trip.vol_status=vol_fault;//这里没有增加障未恢复状态，就用vol_unknown表示
		      }
      }*/
      judgeInstantOrPermanent(counts, &time,nowSeconds,nowMs);
    }

    counts++;
  }
  
  
  /***************************************************************
   越限判断， 每个周波判断一次
  ***************************************************************/
    counts--;
 

     if(resultFault==false)
     {
             
           doCurrentOverlimitJudge(counts,&time,nowSeconds);
  
           doVoltageOverlimitJudge(counts, &time,nowSeconds);

           doVoltageUnderlimitJudge( counts, &time,nowSeconds);
     }
    //  resultFault=false;

  
  
}








/******************************************************************************************/
/*
 后面为汇集故障判断部分，后期分开到另外文件

*/

/********************************************************************************************/
static float averageFront4CyclesVoltage(U8 phase)
{
  float sum = 0;
  U8 i = 0;
  U8 count = 3;
  for (i = 0; i < count; i++) //算前三个是否有电压，第四个周期接近故障周期，不算
  {
    sum += group_value_data[phase][vol_channel][i];
  }
  sum /= count;
  return sum;
}

static float averageBack8CyclesVoltage(U8 phase)
{
  float sum = 0;
  U8 i = 0;
  U8 count = 8;
  U8 start = 12 - count;
  for (i = 0; i < count; i++)
  {
    sum += group_value_data[phase][vol_channel][i + start];
  }
  sum /= count;
  return sum;
}

/*
故障波形分析，电压波形
*/
stuvolstatus waveform_vol(U8 group, U8 phase, U8 serial)
{
  float trip_data = 0;
  U8 i = 0;
  stuvolstatus vol_status;
  vol_status.direction = 0;
  vol_status.limit_flag = 0;
  memset(&vol_status, 0, sizeof(stuvolstatus));
  stuUnitparam *p = &collparam.param_group[group].unit_param[phase];
  float baseValue = averageFront4CyclesVoltage(phase);
  //float back8Value=averageBack8CyclesVoltage(phase);

  for (i = serial; i < judge_buff_length; i++)
  {
    trip_data = (group_value_data[phase][vol_channel][i] - baseValue) * 100 / baseValue;
    if (fabs(trip_data) > p->analog_vol.param_break && trip_data > 0) //当前周期相电场强度突变
    {
      trip_data = (group_value_data[phase][vol_channel][(i - 1 + judge_buff_length) % judge_buff_length] - baseValue) * 100 / baseValue;
      if (fabs(trip_data) > p->analog_vol.param_break && trip_data > 0) //前一周期相电场强度突变
      {
        vol_status.limit_flag = true; //波形存在相电场强度突变
        vol_status.direction = 1; //电压上升
        //vol_status.ratio=fabs(trip_data);
        vol_status.ratio = fabs(((group_value_data[phase][vol_channel][6]+group_value_data[phase][vol_channel][7])/2-baseValue)*100/baseValue);
        break;
      }
    }
    else if (fabs(trip_data) > p->analog_vol.param_break && trip_data < 0)
    {
      trip_data = (group_value_data[phase][vol_channel][(i - 1 + judge_buff_length) % judge_buff_length] - baseValue) * 100 / baseValue;
      if (fabs(trip_data) > p->analog_vol.param_break && trip_data < 0) //前一周期相电场强度突变
      {
        vol_status.limit_flag = true; //波形存在相电场强度突变
        vol_status.direction = 2; //电压下降
        //vol_status.ratio=fabs(trip_data);
        vol_status.ratio = fabs(((group_value_data[phase][vol_channel][6]+group_value_data[phase][vol_channel][7])/2-baseValue)*100/baseValue);
        break;
      }
    }
  }
  return vol_status;
}

stuvolstatus waveform_cur_over(U8 group, U8 phase, U8 serial)
{
  float trip_data = 0;
//  U8 result = 0;
  U8 i = 0;
  stuvolstatus cur_status;
  memset(&cur_status, 0, sizeof(stuvolstatus));
  stuUnitparam *p = &collparam.param_group[group].unit_param[phase];
  for (i = serial; i < judge_buff_length; i++)
  {
    trip_data = group_value_data[phase][cur_channel][i];
    if (trip_data > p->analog_cur.param_fault_up) //相电流越限
    {
      trip_data = group_value_data[phase][cur_channel][(i - 1 + judge_buff_length) % judge_buff_length];
      if (trip_data > p->analog_cur.param_fault_up) //前一周期相电流越限
      {
        cur_status.limit_flag = true; //波形存在电流越限
        break;
      }
    }
  }
  for (i = serial; i < judge_buff_length; i++)
  {
    if (group_value_data[phase][cur_channel][(i - 2 + judge_buff_length) % judge_buff_length] > 5)
    {
      trip_data = group_value_data[phase][cur_channel][i] / group_value_data[phase][cur_channel][(i - 2 + judge_buff_length) % judge_buff_length];
      if (trip_data > 1.5)
      {
        if (group_value_data[phase][cur_channel][(i - 3 + judge_buff_length) % judge_buff_length])
        {
          trip_data = group_value_data[phase][cur_channel][(i - 1 + judge_buff_length) % judge_buff_length] / group_value_data[phase][cur_channel][(i - 3 + judge_buff_length) % judge_buff_length];
          if (trip_data > 1.5)
          {
            cur_status.direction = 1; //上升
            break;
          }
        }
      }
      else
      {
        if (trip_data < 0.5)
        {
          if (group_value_data[phase][cur_channel][(i - 3 + judge_buff_length) % judge_buff_length] > 5)
          {

            trip_data = group_value_data[phase][cur_channel][(i - 1 + judge_buff_length) % judge_buff_length] / group_value_data[phase][cur_channel][(i - 3 + judge_buff_length) % judge_buff_length];
            if (trip_data < 0.5)
            {
              cur_status.direction = 2; //跌落
              break;
            }
          }
        }
      }
    }
  }
  return cur_status;
}



U8 waveform_cur_brk(U8 group, U8 phase, U8 serial)
{
  float trip_data = 0;
  U8 result = 0;
  U8 i = 0;
  if (serial < 4)
  {
    return false;
  }
  stuUnitparam *p = &collparam.param_group[group].unit_param[phase];
  for (i = serial; i < judge_buff_length; i++)
  {
    trip_data = group_value_data[phase][cur_channel][i] - group_value_data[phase][cur_channel][(i - 3 + judge_buff_length) % judge_buff_length];
    if (trip_data > p->phaseShortCurrentBreakValue) //相电流突变
    {
      trip_data = group_value_data[phase][cur_channel][(i - 1 + judge_buff_length) % judge_buff_length] - group_value_data[phase][cur_channel][(i - 4 + judge_buff_length) % judge_buff_length];
      if (trip_data > p->phaseShortCurrentBreakValue) //前一周期相相电流突变
      {
        result = true; //波形存在相电流突变
        break;
      }
    }
  }
  return result;
}

/*static U8 math_vol_zero_peak(void) //检测电压峰值个数，xjj
{
  U8 result = 0;
  float fir_data = 0;
  float mod_data = 0;
  float end_data = 0;
  float condata = 0;
  for (U16 i = 130; i < 128 * 12; i++)
  {
    condata = group_value_data[3][0][i / 128];
    if (group_value_data[3][0][i / 128 - 1] <= 0.5)
    {
      i += 128;
      continue;
    }
    condata *= sin_const;
    if (condata > sin_const)
    {
      fir_data = signal_wave_data[3][(i - 2) * 2];
      mod_data = signal_wave_data[3][(i - 1) * 2];
      end_data = signal_wave_data[3][(i)*2];
      if (fabs(mod_data - fir_data) >= 3 * condata && fabs(mod_data - end_data) > 3 * condata)
      {
        result++;
      }
    }
  }
  return result;
}*/
static U8 math_vol_zero_peak(void) //检测电压峰值个数，xjj
{
  U8 result = 0;
  float fir_data = 0;
  float mod_data = 0;
  float end_data = 0;
  float condata = 0;
  float f1=0;
  float f2=0;
  U16 lastUpPeak=0;
  for(U16 i=128*4;i<128*12-3;i++){
    condata = group_value_data[3][0][0];//基波有效值		
      fir_data = signal_wave_data[3][(i -1) * 2];
      mod_data = signal_wave_data[3][(i ) * 2];
      end_data = signal_wave_data[3][(i+1)*2];
	  f1=signal_wave_data[3][(i - 3) * 2 ];
	  f2=signal_wave_data[3][(i + 3) * 2 ];
            if(mod_data>0
         &&mod_data>fir_data
         &&mod_data>end_data
		 &&mod_data>(3*condata)){
           if((i-lastUpPeak>64)||(lastUpPeak==0)){
             if((fabs(f1)+fabs(f2))/(mod_data*2.0)<0.9){
				 lastUpPeak = i;
				 result++;
             }
           }
      }
  }
  return result;
}
/*
U8 math_cur_zero_peak(void) //检测电流峰值个数
{
  U8 result = 0;
  float fir_data = 0;
  float mod_data = 0;
  float end_data = 0;
  float condata = 0;
  float condata1 = 0;
  for (U16 i = 130; i < 128 * 12; i++)
  {
    condata = group_value_data[3][1][i / 128];//有效值
    if (group_value_data[3][1][i / 128 - 1] <= 0.5) //零票电流,xjj 0.5需要提取成可配置参数
    {
      i += 128;
      continue;
    }
    condata *= sin_const;
    if (condata > sin_const)
    {
      fir_data = signal_wave_data[3][(i - 2) * 2 + 1];//零序瞬时值
      mod_data = signal_wave_data[3][(i - 1) * 2 + 1];
      end_data = signal_wave_data[3][(i)*2 + 1];
      if (fabs(mod_data - fir_data) >= 10 * condata && fabs(mod_data - end_data) > 10 * condata)
      {
        result++;
      }
    }
  }
  return result;
}*/
U8 math_cur_zero_peak(void) //检测电流峰值个数
{
  U8 result = 0;
  U16 lastUpPeak =0;
  float fir_data = 0;
  float mod_data = 0;
  float end_data = 0;
  float condata = 0;
  float f1=0;
  float f2=0;
  for(U16 i=128*4;i<128*12-3;i++){
      condata = group_value_data[3][1][0];//基波有效值
      fir_data = signal_wave_data[3][(i - 1) * 2 + 1];//零序瞬时值
      mod_data = signal_wave_data[3][(i) * 2 + 1];
      end_data = signal_wave_data[3][(i+1)*2 + 1];
      f1=signal_wave_data[3][(i - 3) * 2 + 1];
      f2=signal_wave_data[3][(i + 3) * 2 + 1];
      if(mod_data>0
         &&mod_data>fir_data
         &&mod_data>end_data
		 &&mod_data>(5*condata)){
           if((i-lastUpPeak>64)||(lastUpPeak==0)){
             if(((fabs(f1)+fabs(f2))/(mod_data*2.0))<0.5){
             lastUpPeak = i;
             result++;
             }
           }
      }
  }
return result;
}
U8 math_cur_phase_peak(int phase) //检测电流峰值个数
{
  U8 result = 0;
  U16 lastUpPeak =0;
  float fir_data = 0;
  float mod_data = 0;
  float end_data = 0;
  float condata = 0;
  float f1=0;
  float f2=0;
  for(U16 i=128*4;i<128*12-3;i++){
      condata = group_value_data[phase][1][0];//基波有效值
      fir_data = signal_wave_data[phase][(i - 1) * 2 + 1];//零序瞬时值
      mod_data = signal_wave_data[phase][(i) * 2 + 1];
      end_data = signal_wave_data[phase][(i+1)*2 + 1];
      f1=signal_wave_data[phase][(i - 3) * 2 + 1];
      f2=signal_wave_data[phase][(i + 3) * 2 + 1];
      if(mod_data>0
         &&mod_data>fir_data
         &&mod_data>end_data
		 &&mod_data>(4*condata)){
           if((i-lastUpPeak>64)||(lastUpPeak==0)){
             if(((fabs(f1)+fabs(f2))/(mod_data*2.0))<0.5){
             lastUpPeak = i;
             result++;
             }
           }
      }
  }
return result;
}
U8 math_cur_zero_shake(void) //xjj-未用
{
  U8 result = 0;
  float max_data;
  float min_data = 0;
  float avg_data = 0;
  //float p = group_value_data[3][1][0];
  //max_data = group_value_data[3][1][0];
  //min_data = group_value_data[3][1][0];
  for (U16 i = 1; i < 12; i++)
  {
    if (group_value_data[3][1][i] > 0.5) //fei 0
    {
      avg_data += group_value_data[3][1][i];
      min_data++;
    }
  }
  if (min_data)
  {
    avg_data /= min_data;
  }
  for (U16 i = 1; i < 12; i++)
  {
    max_data = fabs(group_value_data[3][1][i] - avg_data) / avg_data * 100;
    if (max_data > 10.0)
    {
      result++;
    }
  }
  return result;
}

static void computeVolataAndCurrentValue(U8 group, U8 phase, U8 *ptrPhaseBreak)
{
  stuComplex *point = NULL;
  //U8 result = false;
  stuAnalog *p = NULL;
  I16 fft_array[128];
//  I16 fft_array1[128];
  F32 FFT_datamag;  //FFT计算后存放数据的结果

  U16 i, j;
  float vol_base_value;
  float cur_base_value;
  memset(fft_array, 0, sizeof(fft_array));
  for (j = 0; j < 12; j++)
  {
    p = &collparam.param_group[group].unit_param[phase].analog_vol;
    for (i = 0; i < 128; i++)
    {
      fft_array[i] = signal_wave_data[phase][(2 * i) + j * 128 * 2]; //电压
    }
    point = &fftdata[phase][vol_channel][j];
    *point = fft_math_128(fft_array, 1);
    FFT_datamag = sqrtf(point->imagin * point->imagin + point->real * point->real);
   // FFT_datamag *= vol_enlarge;
   // group_value_data[phase][vol_channel][j] =  FFT_datamag;
    
     group_value_data[phase][vol_channel][j] = computeVoltageEffective(FFT_datamag,p->amp_ptr,p->zero_chk);
    
    if (j == 0) vol_base_value = fft_base_value_128(fft_array);
    //电压减去基准
    for (i = 0; i < 128; i++)
    {
      signal_wave_data[phase][(2 * i) + j * 128 * 2] = (short)((signal_wave_data[phase][(2 * i) + j * 128 * 2] - vol_base_value)*p->amp_ptr);
    }


    p = &collparam.param_group[group].unit_param[phase].analog_cur;
    for (i = 0; i < 128; i++)
    {
      fft_array[i] = signal_wave_data[phase][(2 * i + 1) + j * 128 * 2]; //电流
    }
    point = &fftdata[phase][cur_channel][j];
    *point = fft_math_128(fft_array, 1);
    FFT_datamag = sqrtf(point->imagin * point->imagin + point->real * point->real);
    
   /* FFT_datamag *= (cur_enlarge * 0.707 / 64.0f); //计算幅值

    p = &collparam.param_group[group].unit_param[phase].analog_cur;
    group_value_data[phase][cur_channel][j] = (FFT_datamag * p->amp_ptr + p->zero_chk);*/
    
    group_value_data[phase][cur_channel][j]=computeCurrentEffective(FFT_datamag, p->amp_ptr, p->zero_chk);
    
    if (j == 0) cur_base_value = fft_base_value_128(fft_array);

    for (i = 0; i < 128; i++)
    {
      signal_wave_data[phase][(2 * i + 1) + j * 128 * 2] = (short)((signal_wave_data[phase][(2 * i + 1) + j * 128 * 2] - cur_base_value) * p->amp_ptr);
    }
  }
}

//float  cur_data[3][12];//ia-ib,ib-ic,ic-ia
static void computeEffective(U8 group)
{
 
  stuComplex *point = NULL;
 
  stuAnalog *p = NULL;
  I16 fft_array[128];
 // I16 fft_array1[128];
 // float FFT_datamag1;
  float ac_base_value;
  float bc_base_value;
  float cc_base_value;
  float av_base_value;
  float bv_base_value;
  float cv_base_value;
  U16 i, j;
  F32 FFT_datamag;  //FFT计算后存放数据的结果
 // float basewave_value; //FFT计算出的基波值
  memset(fft_array, 0, sizeof(fft_array));
 
  //computeVolataAndCurrentValue(0,phase_a);
  //computeVolataAndCurrentValue(0,phase_b);
  //computeVolataAndCurrentValue(0,phase_c);
  for (j = 0; j < 12; j++)
  {
    p = &collparam.param_group[group].unit_param[0].analog_vol;
    for (i = 0; i < 128; i++)
    {
      fft_array[i] = signal_wave_data[0][(2 * i) + j * 128 * 2]; //A相电压
    }
    point = &fftdata[0][0][j];
    *point = fft_math_128(fft_array, 1);
    FFT_datamag = sqrtf(point->imagin * point->imagin + point->real * point->real);
    // FFT_datamag *= vol_enlarge;
    // group_value_data[phase_a][vol_channel][j] =  FFT_datamag;
    group_value_data[phase_a][vol_channel][j]= computeVoltageEffective(FFT_datamag,p->amp_ptr,p->zero_chk);
    if (j == 0) av_base_value = fft_base_value_128(fft_array);
    //A相电压减去基准
    for (i = 0; i < 128; i++)
    {
      signal_wave_data[0][(2 * i) + j * 128 * 2] = (short)((signal_wave_data[0][(2 * i) + j * 128 * 2] - av_base_value)*p->amp_ptr); //A相电压
    }
    /*if (j >= 1)
    {
      for (i = 0; i < 128; i++)
      {
        fft_array1[i] = signal_wave_data[0][(2 * i) + (j - 1) * 128 * 2]; //A相电压
      }
      //判断是否相位突变
      result = Phase_brk_quarter(fft_array, fft_array1, group, 0, (float)(p->phase_break), p->phase_break_num);
      if (result)
      {
        phaseBreakResult.aPhaBreak = true;
      }
    }*/
    p = &collparam.param_group[group].unit_param[0].analog_cur;
    for (i = 0; i < 128; i++)
    {
      fft_array[i] = signal_wave_data[0][(2 * i + 1) + j * 128 * 2]; //A相电流
    }
    point = &fftdata[0][1][j];
    *point = fft_math_128(fft_array, 1);
    FFT_datamag = sqrtf(point->imagin * point->imagin + point->real * point->real);
 
    group_value_data[phase_a][cur_channel][j]=computeCurrentEffective(FFT_datamag, p->amp_ptr, p->zero_chk);
    if (j == 0) 
    {
      ac_base_value = fft_base_value_128(fft_array);
    }
    for (i = 0; i < 128; i++)
    {
      if((collparam.wave_adjust&0x0001) == 1){
      signal_wave_data[0][(2 * i + 1) + j * 128 * 2] = (short)((signal_wave_data[0][(2 * i + 1) + j * 128 * 2] - ac_base_value) * p->amp_ptr*(-1));
      }else{
       signal_wave_data[0][(2 * i + 1) + j * 128 * 2] = (short)((signal_wave_data[0][(2 * i + 1) + j * 128 * 2] - ac_base_value) * p->amp_ptr); 
      }
    }
    /*************************************/
    p = &collparam.param_group[group].unit_param[1].analog_vol;
    for (i = 0; i < 128; i++)
    {
      fft_array[i] = signal_wave_data[1][(2 * i) + j * 128 * 2]; //B相电压
    }
    point = &fftdata[1][0][j];
    *point = fft_math_128(fft_array, 1);
    FFT_datamag = sqrtf(point->imagin * point->imagin + point->real * point->real);
    // FFT_datamag *= vol_enlarge;
    // group_value_data[phase_b][vol_channel][j] = FFT_datamag;
    group_value_data[phase_b][vol_channel][j]= computeVoltageEffective(FFT_datamag,p->amp_ptr,p->zero_chk);
    if (j == 0) 
    {
      bv_base_value = fft_base_value_128(fft_array);
    }
    for (i = 0; i < 128; i++)
    {
      signal_wave_data[1][(2 * i) + j * 128 * 2] = (short)((signal_wave_data[1][(2 * i) + j * 128 * 2] - bv_base_value)*p->amp_ptr); //B相电压     
    }
    /*if (j >= 1)
    {
      for (i = 0; i < 128; i++)
      {
        fft_array1[i] = signal_wave_data[1][(2 * i) + (j - 1) * 128 * 2]; //B相电压
      }
      result = Phase_brk_quarter(fft_array, fft_array1, group, 1, (float)(p->phase_break), p->phase_break_num);
      if (result)
      {
        phaseBreakResult.bPhaBreak = true;
      }
    }*/
    p = &collparam.param_group[group].unit_param[1].analog_cur;
    for (i = 0; i < 128; i++)
    {
      fft_array[i] = signal_wave_data[1][(2 * i + 1) + j * 128 * 2]; //B相电流
    }
    point = &fftdata[1][1][j];
    *point = fft_math_128(fft_array, 1);
    FFT_datamag = sqrtf(point->imagin * point->imagin + point->real * point->real);
  
    group_value_data[phase_b][cur_channel][j]=computeCurrentEffective(FFT_datamag, p->amp_ptr, p->zero_chk);
    if (j == 0) 
    {
      bc_base_value = fft_base_value_128(fft_array);
    }
    for (i = 0; i < 128; i++)
    {
      if((collparam.wave_adjust& 0x0002) == 2){
      signal_wave_data[1][(2 * i + 1) + j * 128 * 2] = (short)((signal_wave_data[1][(2 * i + 1) + j * 128 * 2] - bc_base_value) * p->amp_ptr*(-1));
      }else{
      signal_wave_data[1][(2 * i + 1) + j * 128 * 2] = (short)((signal_wave_data[1][(2 * i + 1) + j * 128 * 2] - bc_base_value) * p->amp_ptr);
      }
    }
    /*************************************/
    p = &collparam.param_group[group].unit_param[2].analog_vol;
    for (i = 0; i < 128; i++)
    {
      fft_array[i] = signal_wave_data[2][(2 * i) + j * 128 * 2]; //C相电压
    }
    point = &fftdata[2][0][j];
    *point = fft_math_128(fft_array, 1);
    FFT_datamag = sqrtf(point->imagin * point->imagin + point->real * point->real);
    //FFT_datamag *= vol_enlarge;
    //group_value_data[phase_c][vol_channel][j] = FFT_datamag;
    group_value_data[phase_c][vol_channel][j]= computeVoltageEffective(FFT_datamag,p->amp_ptr,p->zero_chk);
    if (j == 0)
    {
      cv_base_value = fft_base_value_128(fft_array);
    }
    for (i = 0; i < 128; i++)
    {
      signal_wave_data[2][(2 * i) + j * 128 * 2] = (short)((signal_wave_data[2][(2 * i) + j * 128 * 2] - cv_base_value)*p->amp_ptr); //c相电压  
    }
    /*if (j >= 1)
    {
      for (i = 0; i < 128; i++)
      {
        fft_array1[i] = signal_wave_data[2][(2 * i) + (j - 1) * 128 * 2]; //C相电压
      }
      result = Phase_brk_quarter(fft_array, fft_array1, group, 2, (float)(p->phase_break), p->phase_break_num);
      if (result)
      {
        phaseBreakResult.cPhaBreak = true;
      }
    }*/
    p = &collparam.param_group[group].unit_param[2].analog_cur;
    for (i = 0; i < 128; i++)
    {
      fft_array[i] = signal_wave_data[2][(2 * i + 1) + j * 128 * 2]; //C相电流
    }
    point = &fftdata[2][1][j];
    *point = fft_math_128(fft_array, 1);
    FFT_datamag = sqrtf(point->imagin * point->imagin + point->real * point->real);
  
    group_value_data[phase_c][cur_channel][j]=computeCurrentEffective(FFT_datamag, p->amp_ptr, p->zero_chk);
    if (j == 0)
    {
      cc_base_value = fft_base_value_128(fft_array);
    }
    for (i = 0; i < 128; i++)
    {
      if((collparam.wave_adjust&0x0004) == 4){
      signal_wave_data[2][(2 * i + 1) + j * 128 * 2] = (short)((signal_wave_data[2][(2 * i + 1) + j * 128 * 2] - cc_base_value) * p->amp_ptr*(-1));
      }else{
      signal_wave_data[2][(2 * i + 1) + j * 128 * 2] = (short)((signal_wave_data[2][(2 * i + 1) + j * 128 * 2] - cc_base_value) * p->amp_ptr);
      }
    }
    /*********************计算零序电压**************/
    stuComplex complex = fun_component_F0(fftdata[phase_a][vol_channel][j], fftdata[phase_b][vol_channel][j], fftdata[phase_c][vol_channel][j]); //计算零序电压
    FFT_datamag = sqrtf(complex.imagin * complex.imagin + complex.real * complex.real);
    /// FFT_datamag *= vol_enlarge;
    //group_value_data[phase_z][vol_channel][j] = FFT_datamag;
    group_value_data[phase_z][vol_channel][j]= computeVoltageEffective(FFT_datamag,p->amp_ptr,p->zero_chk);
    /****************************************************/
    /*********************计算零序电流**************/
    complex = fun_component_F0(fftdata[phase_a][cur_channel][j], fftdata[phase_b][cur_channel][j], fftdata[phase_c][cur_channel][j]); //计算零序电流
    FFT_datamag = sqrtf(complex.imagin * complex.imagin + complex.real * complex.real);
    
    group_value_data[phase_z][cur_channel][j] = computeCurrentEffective(FFT_datamag, p->amp_ptr, p->zero_chk);
    /****************************************************/
  }
  for (j = 0; j < (128 * 12 * 2); j++) //计算零序
  {
    signal_wave_data[phase_z][j] = (signal_wave_data[phase_a][j] + signal_wave_data[phase_b][j] + signal_wave_data[phase_c][j]) / 3;
  }
}

static U8 judgeFront4CyclesVoltageIsPower(U8 phase, float voltageCritical)
{
  float sum = 0;
  U8 i = 0;
  U8 count = 3;
  for (i = 0; i < count; i++) //算前三个是否有电压，第四个周期接近故障周期，不算
  {
    sum += group_value_data[phase][vol_channel][i];
  }
  sum /= count;
  if (sum > voltageCritical) return true;
  else return false;
}
static U8 judgeFront4CyclesCurIsPower(U8 phase, float curCritical)
{
  float sum = 0;
  U8 i = 0;
  U8 count = 3;
  for (i = 0; i < count; i++) //算前三个是否有电流，第四个周期接近故障周期，不算
  {
    sum += group_value_data[phase][cur_channel][i];
  }
  sum /= count;
  if (sum > curCritical) return true;
  else return false;
}
static U8 judgeBack8CyclesVoltageIsPower(U8 phase, float voltageCritical)
{
  float sum = 0;
//  U8 i = 0;
//  U8 count = 6;
//  U8 start = 12 - count;
//
//  for (i = 0; i < count; i++)
//  {
//    sum += group_value_data[phase][vol_channel][i + start];
//  }
//  sum /= count;

  sum=(group_value_data[phase][vol_channel][4]+group_value_data[phase][vol_channel][5])/2;
  if (sum > voltageCritical) return true;
  else return false;
}
static U8 judgeBack8CyclesCurIsPower(U8 phase, float curCritical)
{
  float sum = 0;
//  U8 i = 0;
//  U8 count = 6;
//  U8 start = 12 - count;
//
//  for (i = 0; i < count; i++)
//  {
//    sum += group_value_data[phase][vol_channel][i + start];
//  }
//  sum /= count;

  sum=(group_value_data[phase][cur_channel][4]+group_value_data[phase][cur_channel][5])/2;
  if (sum > curCritical) return true;
  else return false;
}
/*******************************************************
波形类型判断
上电过程，失电过程，故障过程
在上电过程或失电过程中都会触发录波；因此，在分析故障时需要确定录波的类型：
在上电过程中，前4个波形：电压幅值0，电流幅值为0；
上电过程包括：本装置的安装，线路上电或安装在非故障相上的因故障相重合闸造成的上电过程；
在失电判断时需要判断在失电之前是否发生故障，根据故障前点的前4个波形分析；
在失电过程中，后8个波形电压幅值、电流幅值趋于0；也有可能在后8个波形录波为录波完成时，线路跳闸，导致最后波形幅值数据趋于0；
*********************************************************/
//判断上电过程
static U8 judgeVoltageStatus(float voltageCritical,float curCritical)
{
  U8 aFront4IsPower = judgeFront4CyclesVoltageIsPower(phase_a, voltageCritical);
  U8 aBack8IsPower = judgeBack8CyclesVoltageIsPower(phase_a, voltageCritical);

  U8 bFront4IsPower = judgeFront4CyclesVoltageIsPower(phase_b, voltageCritical);
  U8 bBack8IsPower = judgeBack8CyclesVoltageIsPower(phase_b, voltageCritical);

  U8 cFront4IsPower = judgeFront4CyclesVoltageIsPower(phase_c, voltageCritical);
  U8 cBack8IsPower = judgeBack8CyclesVoltageIsPower(phase_c, voltageCritical);
  
  
  U8 aFront4IsPowerCur = judgeFront4CyclesCurIsPower(phase_a, curCritical);
  U8 aBack8IsPowerCur = judgeBack8CyclesCurIsPower(phase_a, curCritical);

  U8 bFront4IsPowerCur = judgeFront4CyclesCurIsPower(phase_b, curCritical);
  U8 bBack8IsPowerCur = judgeBack8CyclesCurIsPower(phase_b, curCritical);

  U8 cFront4IsPowerCur = judgeFront4CyclesCurIsPower(phase_c, curCritical);
  U8 cBack8IsPowerCur = judgeBack8CyclesCurIsPower(phase_c, curCritical);

  sal_dbg_printf(DEBUG_FAULT, "collect unit line status: a4:%d,a8:%d,b4:%d,b8:%d,c4:%d,c8:%d\r\n",
                 aFront4IsPower, aBack8IsPower, bFront4IsPower, bBack8IsPower, cFront4IsPower, cBack8IsPower);
  //有一相上电，即为上电过程
  if ((aFront4IsPower == false && aBack8IsPower == true&&aFront4IsPowerCur == false && aBack8IsPowerCur == true) 
      || (bFront4IsPower == false && bBack8IsPower == true&&bFront4IsPowerCur == false && bBack8IsPowerCur == true) 
      || (cFront4IsPower == false && cBack8IsPower == true&&cFront4IsPowerCur == false && cBack8IsPowerCur == true))
  {
    return vol_up; //当前录的波形属于线路上电过程
  }
  if (aFront4IsPower == true && aBack8IsPower == false)
  {
    if (bFront4IsPower == true && bBack8IsPower == false)
    {
      if (cFront4IsPower == true && cBack8IsPower == false)
      {
        if(aFront4IsPowerCur == true && aBack8IsPowerCur == false&&bFront4IsPowerCur == true && bBack8IsPowerCur == false&&cFront4IsPowerCur == true && cBack8IsPowerCur == false){
                return vol_down; //当前录的波形属于线路掉电过程
        }

      }
    }
  }
  if (aFront4IsPower == false && aBack8IsPower == false)
  {
    if (bFront4IsPower == false && bBack8IsPower == false)
    {
      if (cFront4IsPower == false && cBack8IsPower == false)
      {
        return vol_none; //前4后8的电压波形都为0，当前录的波形处于掉电状态，应用于休眠情况下，发生相间短路唤醒
      }
    }
  }
  return vol_normal;
}

#define  TYPE_A_SHORT_CIRCUIT 0 //A短路
#define  TYPE_B_SHORT_CIRCUIT 1//B短路
#define  TYPE_C_SHORT_CIRCUIT 2//C短路
#define  TYPE_AB_SHORT_CIRCUIT 3//AB短路
#define  TYPE_AC_SHORT_CIRCUIT 4//AC短路
#define  TYPE_BC_SHORT_CIRCUIT 5//BC短路
#define  TYPE_ABC_SHORT_CIRCUIT 6//ABC短路
/*
显示接地故障信息
*/
static void outEarthFaultMsg(stufaultinfo *faultinfo,U8 type,U8 phase){// 参数3故障相 0A 1B 2C ；参数2 故障类别 0弧光 1高阻  2金属  3小电阻
    U8 group;
    U16 fault_code;
    msg_t fault_msg;
    group = faultinfo->group ;
    fault_msg.msg_len = 0;
    memset(&fault_msg, 0, sizeof(msg_t)); 
    if(type ==0){//弧光
      collstatus.Unit_dev_info[group][phase_a].Dev_status|=0x0200;
      fault_msg.msg_type = event_fault_arc_gnd_id;
      if(phase == 0){
        fault_code = line1_arc_gnd_a_id + group*SIGLE_PASHE_SIZE;
      }else if(phase==1){
        fault_code = line1_arc_gnd_b_id + group*SIGLE_PASHE_SIZE;
      }else{
        fault_code = line1_arc_gnd_c_id + group*SIGLE_PASHE_SIZE;
      }     
    }
    else if(type ==1){//高阻
     collstatus.Unit_dev_info[group][phase_a].Dev_status|=0x0400;
     fault_msg.msg_type = event_fault_big_resistor_gnd_id;
      if(phase == 0){
        fault_code = line1_big_resistor_gnd_a_id + group*SIGLE_PASHE_SIZE;
      }else if(phase==1){
        fault_code = line1_big_resistor_gnd_b_id + group*SIGLE_PASHE_SIZE;
      }else{
        fault_code = line1_big_resistor_gnd_c_id + group*SIGLE_PASHE_SIZE;
      } 
    }
    else if(type ==2){//金属
     collstatus.Unit_dev_info[group][phase_a].Dev_status|=0x0800;
     fault_msg.msg_type = event_fault_metal_gnd_id;
      if(phase == 0){
        fault_code = line1_small_resistor_gnd_a_id + group*SIGLE_PASHE_SIZE;
      }else if(phase==1){
        fault_code = line1_small_resistor_gnd_b_id + group*SIGLE_PASHE_SIZE;
      }else{
        fault_code = line1_small_resistor_gnd_c_id + group*SIGLE_PASHE_SIZE;
      } 
    }
    else if(type ==3){//小电阻
     collstatus.Unit_dev_info[group][phase_a].Dev_status|=0x0080;
     fault_msg.msg_type = event_fault_little_resistor_gnd_id;
      if(phase == 0){
        fault_code = line1_little_resistor_gnd_a_id + group*SIGLE_PASHE_SIZE;
      }else if(phase==1){
        fault_code = line1_little_resistor_gnd_b_id + group*SIGLE_PASHE_SIZE;
      }else{
        fault_code = line1_little_resistor_gnd_c_id + group*SIGLE_PASHE_SIZE;
      } 
    }
          faultinfo->judgetype = fault_code;
          fault_info.event_code = fault_code;
          msg_send(msg_gprs, fault_msg); 
          
          if(phase == 0){//a相故障
            if(((transperflag&1)==1)||((transperflag&2)==2)){
              transperflag = transperflag & (1+2);
            }else if(((transperflag&4)==4)||((transperflag&8)==8)){  
            transperflag = (transperflag & (4+8))>>2;
            }else if(((transperflag&16)==16)||((transperflag&32)==32)){
            transperflag = (transperflag & (16+32))>>4;
            }
            //ctrl_indicate_cmd3(group,0,INDICATOR_ON);
            ctrl_traper_cmd(group,0, (transperflag-1));//参数3 0瞬时，1永久；
            ctrl_indicate_cmd3(group,1,INDICATOR_OFF);
            ctrl_indicate_cmd3(group,2,INDICATOR_OFF);// 0A相  1B相  2C相

          }else if(phase ==1){//b相故障
            if(((transperflag&1)==1)||((transperflag&2)==2)){
            transperflag = (transperflag&(1+2))<<2;
            }else if(((transperflag&4)==4)||((transperflag&8)==8)){
            transperflag = transperflag & (4+8);
            }else if(((transperflag&16)==16)||((transperflag&32)==32)){
            transperflag = (transperflag & (16+32))>>2;
            } 
            ctrl_indicate_cmd3(group,0,INDICATOR_OFF);
            //ctrl_indicate_cmd3(group,1,INDICATOR_ON);
            ctrl_traper_cmd(group,1, ((transperflag>>2)-1));//参数3 0瞬时，1永久；
            ctrl_indicate_cmd3(group,2,INDICATOR_OFF);// 0A相  1B相  2C相 
          }else{//c相故障
            if(((transperflag&1)==1)||((transperflag&2)==2)){
              transperflag = (transperflag&(1+2))<<4;
            }else if(((transperflag&4)==4)||((transperflag&8)==8)){ 
            transperflag = (transperflag&(4+8))<<2;
            }else if(((transperflag&16)==16)||((transperflag&32)==32)){
            transperflag = transperflag&(16+32);
            }
            ctrl_indicate_cmd3(group,0,INDICATOR_OFF);
            ctrl_indicate_cmd3(group,1,INDICATOR_OFF);
            //ctrl_indicate_cmd3(group,2,INDICATOR_ON);// 0A相  1B相  2C相 
            ctrl_traper_cmd(group,2, ((transperflag>>4)-1));//参数3 0瞬时，1永久；


          }          

}
/*
相间短路时操作指示灯
*/
static void  openCloseIndicatorLightShortCircuitFailure(U8 group,U8 type){//参数2  0A短路   1B墩路 2C短路   3AB短路  4AC短路 5BC短路  6 ABc短路  参数3线路状态
  if(type ==TYPE_A_SHORT_CIRCUIT){              //a短路
     //ctrl_indicate_cmd3(group,0,INDICATOR_ON);
     ctrl_traper_cmd(group,0, (transperflag-1));//参数3 0瞬时，1永久；
     ctrl_indicate_cmd3(group,1,INDICATOR_OFF);
     ctrl_indicate_cmd3(group,2,INDICATOR_OFF);// 0A相  1B相  2C相     
  }else if(type ==TYPE_B_SHORT_CIRCUIT){
     ctrl_indicate_cmd3(group,0,INDICATOR_OFF);
     //ctrl_indicate_cmd3(group,1,INDICATOR_ON);
     ctrl_traper_cmd(group,1, ((transperflag>>2)-1));//参数3 0瞬时，1永久；
     ctrl_indicate_cmd3(group,2,INDICATOR_OFF);  
  }else if(type ==TYPE_C_SHORT_CIRCUIT){
     ctrl_indicate_cmd3(group,0,INDICATOR_OFF);
     ctrl_indicate_cmd3(group,1,INDICATOR_OFF);
     //ctrl_indicate_cmd3(group,2,INDICATOR_ON);
     ctrl_traper_cmd(group,2, ((transperflag>>4)-1));//参数3 0瞬时，1永久；
  }else if(type ==TYPE_AB_SHORT_CIRCUIT){
     //ctrl_indicate_cmd3(group,0,INDICATOR_ON);
     //ctrl_indicate_cmd3(group,1,INDICATOR_ON);
     ctrl_traper_cmd(group,0, (transperflag&(1+2)-1));//参数3 0瞬时，1永久；
     ctrl_traper_cmd(group,1, (((transperflag&(4+8))>>2)-1));//参数3 0瞬时，1永久；
     ctrl_indicate_cmd3(group,2,INDICATOR_OFF);  
  }else if(type ==TYPE_AC_SHORT_CIRCUIT){
     //ctrl_indicate_cmd3(group,0,INDICATOR_ON);
     ctrl_traper_cmd(group,0, (transperflag&(1+2)-1));//参数3 0瞬时，1永久；
     ctrl_indicate_cmd3(group,1,INDICATOR_OFF);
     //ctrl_indicate_cmd3(group,2,INDICATOR_ON); 
     ctrl_traper_cmd(group,2, (((transperflag&(16+32))>>4)-1));//参数3 0瞬时，1永久；
  }else if(type ==TYPE_BC_SHORT_CIRCUIT){
     ctrl_indicate_cmd3(group,0,INDICATOR_OFF);
     //ctrl_indicate_cmd3(group,1,INDICATOR_ON);
     ctrl_traper_cmd(group,1, (((transperflag&(4+8))>>2)-1));//参数3 0瞬时，1永久；
     ctrl_traper_cmd(group,2, (((transperflag&(16+32))>>4)-1));//参数3 0瞬时，1永久；
     //ctrl_indicate_cmd3(group,2,INDICATOR_ON);  
  }else if(type ==TYPE_ABC_SHORT_CIRCUIT){
     //ctrl_indicate_cmd3(group,0,INDICATOR_ON);
     //ctrl_indicate_cmd3(group,1,INDICATOR_ON);
     //ctrl_indicate_cmd3(group,2,INDICATOR_ON); 
    ctrl_traper_cmd(group,0, (transperflag&(1+2)-1));//参数3 0瞬时，1永久；
    ctrl_traper_cmd(group,1, (((transperflag&(4+8))>>2)-1));//参数3 0瞬时，1永久；
    ctrl_traper_cmd(group,2, (((transperflag&(16+32))>>4)-1));//参数3 0瞬时，1永久；
  }

}
/*
相间短路时操作瞬时永久性标志位，//1A瞬时  2A永久 4B瞬时  8B永久 16B瞬时  32B永久,可叠加
思路：首先记录触发相瞬时或永久故障，1、判断出的故障相有瞬时或永久标志则保持不变，没有则与触发相一致 2.触发相，但不在判断结果中，该相的瞬时或永久标志至0
*/
static void OperatingInstantaneousPermanenceFlag(U8 phase,U8 type){//参数2  0A短路   1B墩路 2C短路   3AB短路  4AC短路 5BC短路  6 ABc短路  参数3线路状态
  U8 flag;//记录触发相的瞬时或永久
  U8 temp;//临时变量
  if(phase==0){
    flag = (transperflag&(1+2));
  }else if(phase ==1){
    flag = (transperflag&(4+8))>>2;
  }else {
  flag = (transperflag&(16+32))>>4;
  } 
  if(type ==TYPE_A_SHORT_CIRCUIT){//a短路
    if(((transperflag&1)==1)||((transperflag&2)==2)){
     transperflag =  transperflag &(1+2);
    }else{
    transperflag = flag;
    }
  }else if(type ==TYPE_B_SHORT_CIRCUIT){//b短路
    if(((transperflag&4)==4)||((transperflag&8)==8)){
     transperflag =  transperflag &(4+8);
    }else{
    transperflag = flag<<2;
    }    
  }else if(type ==TYPE_C_SHORT_CIRCUIT){//c短路
    if(((transperflag&16)==16)||((transperflag&32)==32)){
     transperflag =  transperflag &(16+32);
    }else{
    transperflag = flag<<4;
    }    
  }else if(type ==TYPE_AB_SHORT_CIRCUIT){//ab短路
    temp =0;
    if(((transperflag&1)==1)||((transperflag&2)==2)){
     temp +=  transperflag &(1+2);
    }else{
    temp += flag;
    }
    if(((transperflag&4)==4)||((transperflag&8)==8)){
     temp +=  transperflag &(4+8);
    }else{
    temp += flag<<2;
    }
    transperflag = temp;
  }else if(type ==TYPE_AC_SHORT_CIRCUIT){//ac短路
    temp =0;
    if(((transperflag&1)==1)||((transperflag&2)==2)){
     temp +=  transperflag &(1+2);
    }else{
    temp += flag;
    }
    if(((transperflag&16)==16)||((transperflag&32)==32)){
     temp +=  transperflag &(16+32);
    }else{
    temp += flag<<4;
    }
    transperflag = temp;    
  }else if(type ==TYPE_BC_SHORT_CIRCUIT){//bc短路
    temp =0;
    if(((transperflag&4)==4)||((transperflag&8)==8)){
     temp +=  transperflag &(4+8);
    }else{
    temp += flag<<2;
    }
    if(((transperflag&16)==16)||((transperflag&32)==32)){
     temp +=  transperflag &(16+32);
    }else{
    temp += flag<<4;
    }
    transperflag = temp;    
  }else if(type ==TYPE_BC_SHORT_CIRCUIT){//abc短路
    temp =0;
    if(((transperflag&1)==1)||((transperflag&2)==2)){
     temp +=  transperflag &(1+2);
    }else{
    temp += flag;
    }    
    if(((transperflag&4)==4)||((transperflag&8)==8)){
     temp +=  transperflag &(4+8);
    }else{
    temp += flag<<2;
    }
    if(((transperflag&16)==16)||((transperflag&32)==32)){
     temp +=  transperflag &(16+32);
    }else{
    temp += flag<<4;
    }
    transperflag = temp;    
  }

}
/*
相间短路相关
*/
static void outShortCircuitFailureMsg(stufaultinfo *faultinfo ,U8 type,U8 line_status){//参数2  0A短路   1B墩路 2C短路   3AB短路  4AC短路 5BC短路  6 ABc短路  参数3线路状态
  
    U8 group;
    U8 phase;
    U16 fault_code;
    msg_t fault_msg;
    group = faultinfo->group;
    phase = faultinfo->phase;
    if(type>6)
    return;   
    memset(&fault_msg, 0, sizeof(msg_t));     
    if(type ==TYPE_A_SHORT_CIRCUIT){      //a短路
      fault_code = line1_phase_cur_break_a + group*SIGLE_PASHE_SIZE;
      fault_msg.msg_type = event_fault_sigle_phase_short_id;
      fault_msg.msg_data[0] = group;  
      fault_msg.msg_data[1] = 0;   
    }else if(type ==TYPE_B_SHORT_CIRCUIT){//b短路
      fault_code = line1_phase_cur_break_b + group*SIGLE_PASHE_SIZE;
      fault_msg.msg_type = event_fault_sigle_phase_short_id;
      fault_msg.msg_data[0] = group;  
      fault_msg.msg_data[1] = 1;      
    }else if(type ==TYPE_C_SHORT_CIRCUIT){//c短路
      fault_code = line1_phase_cur_break_c + group*SIGLE_PASHE_SIZE;
      fault_msg.msg_type = event_fault_sigle_phase_short_id;
      fault_msg.msg_data[0] = group;  
      fault_msg.msg_data[1] = 2;       
    }else if(type ==TYPE_AB_SHORT_CIRCUIT){//ab短路
        collstatus.Unit_dev_info[group][phase_a].Dev_status|=0x0100;
        collstatus.Unit_dev_info[group][phase_b].Dev_status|=0x0100;      
        fault_code = line1_phase_short_ab_id + group*SIGLE_PASHE_SIZE;
        fault_msg.msg_type = event_fault_phase_short_id;
        fault_msg.msg_len = 0; 
    }else if(type ==TYPE_AC_SHORT_CIRCUIT){//ac短路
        collstatus.Unit_dev_info[group][phase_a].Dev_status|=0x0100;
        collstatus.Unit_dev_info[group][phase_c].Dev_status|=0x0100;
        fault_code = line1_phase_short_ac_id + group*SIGLE_PASHE_SIZE;
        fault_msg.msg_type = event_fault_phase_short_id;
        fault_msg.msg_len = 0;
    }else if(type ==TYPE_BC_SHORT_CIRCUIT){//bc短路
        collstatus.Unit_dev_info[group][phase_b].Dev_status|=0x0100;
        collstatus.Unit_dev_info[group][phase_c].Dev_status|=0x0100;
        fault_code = line1_phase_short_bc_id + group*SIGLE_PASHE_SIZE;
        fault_msg.msg_type = event_fault_phase_short_id;
        fault_msg.msg_len = 0;
        
    }else if(type ==TYPE_ABC_SHORT_CIRCUIT){//abc短路
        collstatus.Unit_dev_info[group][phase_a].Dev_status|=0x0100;
        collstatus.Unit_dev_info[group][phase_b].Dev_status|=0x0100;
        collstatus.Unit_dev_info[group][phase_c].Dev_status|=0x0100;
        fault_code = line1_phase_short_abc_id + group*SIGLE_PASHE_SIZE;
        fault_msg.msg_type = event_fault_phase_short_id;
        fault_msg.msg_len = 0;
    }
        faultinfo->judgetype = fault_code; 
        fault_info.event_code = fault_code;
        msg_send(msg_gprs, fault_msg);
        OperatingInstantaneousPermanenceFlag(phase,type);//操作瞬时永久性标志位
        openCloseIndicatorLightShortCircuitFailure(group,type);//操作指示灯

}
/*
误动作 相关    
*/
  
  static U8 doErrorAction(U8 group,U8 phase){
  float trip_data = 0;
  U8 i = 0;
  stuUnitparam *p = &collparam.param_group[group].unit_param[phase];
  float baseValue = averageFront4CyclesVoltage(phase);
  //float back8Value=averageBack8CyclesVoltage(phase);

  for (i = 4; i < 12; i++)
  {
    trip_data = (group_value_data[phase][vol_channel][i] - baseValue) * 100 / baseValue;
    if (fabs(trip_data) > 5 ) //当前周期相电场强度突变
    {
      trip_data = (group_value_data[phase][vol_channel][(i - 1 + judge_buff_length) % judge_buff_length] - baseValue) * 100 / baseValue;
      if (fabs(trip_data) > 5) //前一周期相电场强度突变
      {
        return true;
      }
    }
  }
    return false;
  
  }

static U8 doFaultTypeJudge(stufaultinfo* faultinfo,U8  line_status)//参数2：1：上电，2：失电，3：故障波形
{
  U8 result = false;
  stuvolstatus break_result_ua;
  stuvolstatus break_result_ub;
  stuvolstatus break_result_uc;
  stuvolstatus over_result_ia;
  stuvolstatus over_result_ib;
  stuvolstatus over_result_ic;
  U8  result_brk_ia = false;
  U8  result_brk_ib = false;
  U8  result_brk_ic = false;
  U8  break_result_i0 = false;
  U8  break_result_u0 = false;
  U8 group = faultinfo->group; 
  stuUnitparam *p= &collparam.param_group[group].unit_param[phase_b];
  memset(&break_result_ua, 0, sizeof(stuvolstatus));
  memset(&break_result_ub, 0, sizeof(stuvolstatus));
  memset(&break_result_uc, 0, sizeof(stuvolstatus));
  memset(&over_result_ia, 0, sizeof(stuvolstatus));
  memset(&over_result_ib, 0, sizeof(stuvolstatus));
  memset(&over_result_ic, 0, sizeof(stuvolstatus));
  over_result_ia = waveform_cur_over(group, phase_a, 3);//电流越线
  over_result_ib = waveform_cur_over(group, phase_b, 3);
  over_result_ic = waveform_cur_over(group, phase_c, 3);
  sal_dbg_printf(DEBUG_FAULT, "collect unit current limit status: A:%d,orientation:%d B:%d,orientation:%d, C:%d,orientation:%d\r\n",
                 over_result_ia.limit_flag, over_result_ia.direction, over_result_ib.limit_flag, over_result_ib.direction, over_result_ic.limit_flag, over_result_ic.direction);
  result_brk_ia = waveform_cur_brk(group, phase_a, 4);//电流突变
  result_brk_ib = waveform_cur_brk(group, phase_b, 4);
  result_brk_ic = waveform_cur_brk(group, phase_c, 4);
  sal_dbg_printf(DEBUG_FAULT, "collect unit current break status: A:%d B:%d C:%d\r\n", result_brk_ia, result_brk_ib, result_brk_ic);
  if (line_status == vol_normal)
  {
    break_result_ua = waveform_vol(group, phase_a, 0);//电压突变
    break_result_ub = waveform_vol(group, phase_b, 0);
    break_result_uc = waveform_vol(group, phase_c, 0);
    sal_dbg_printf(DEBUG_FAULT, "collect unit, voltage break, A:%d,orientation:%d,B:%d,orientation:%d,C:%d,orientation:%d\r\n",
                   break_result_ua.limit_flag, break_result_ua.direction, break_result_ub.limit_flag, break_result_ub.direction, break_result_uc.limit_flag, break_result_uc.direction);
    sal_dbg_printf(DEBUG_FAULT, "collect unit, current zero break:%d,voltage zero break:%d\r\n",  
                   break_result_i0, break_result_u0);
  }
  /************************************************************************/
  /*相间短路判断
   相间短路现象：相电场强度突变，相电流越限，相电流突变；
   判据：
   短路相的相电场强度跌落；
   短路相的相电流越限；故障点前电流相位差180，故障点后故障相的相电流同相位
  */
  /************************************************************************/

//  CLEAR_BIT(collstatus.Unit_dev_info[group][phase_a].Dev_status, 0x0100 | 0x0020 | 0x0080 | 0x0200 | 0x0400);
//  CLEAR_BIT(collstatus.Unit_dev_info[group][phase_b].Dev_status, 0x0100 | 0x0020 | 0x0080 | 0x0200 | 0x0400);
//  CLEAR_BIT(collstatus.Unit_dev_info[group][phase_c].Dev_status, 0x0100 | 0x0020 | 0x0080 | 0x0200 | 0x0400);
  CLEAR_BIT(collstatus.Unit_dev_info[group][phase_a].Dev_status, 0x0100 | 0x0080 | 0x0200 | 0x0400 |0X800);
  CLEAR_BIT(collstatus.Unit_dev_info[group][phase_b].Dev_status, 0x0100 | 0x0080 | 0x0200 | 0x0400 |0X800);
  CLEAR_BIT(collstatus.Unit_dev_info[group][phase_c].Dev_status, 0x0100 | 0x0080 | 0x0200 | 0x0400 |0X800);
  switch (collparam.Ground_mode)
  {
  case  system_ungrounded:
    if (line_status == vol_none) //无压情况下，相间短路
    {
      if ((over_result_ic.limit_flag && over_result_ib.limit_flag  && over_result_ia.limit_flag) || 
          (result_brk_ic && result_brk_ib && result_brk_ia)) //abc相相间短路
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_ABC_SHORT_CIRCUIT,line_status);//参数2  0A短路   1B墩路 2C短路   3AB短路  4AC短路 5BC短路  6 ABc短路  参数3线路状态
        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! no voltage status! A/B/C phase short! group:%d\r\n", group);
        return true;
      }
      if ((over_result_ia.limit_flag && over_result_ib.limit_flag) || (result_brk_ia && result_brk_ib)) //ab相相间短路
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_AB_SHORT_CIRCUIT,line_status);
        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! no voltage status! A/B phase short! group:%d\r\n", group);
        return true; //返回true 录波
      }
      if ((over_result_ia.limit_flag && over_result_ic.limit_flag) || (result_brk_ia && result_brk_ic)) //ac相相间短路
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_AC_SHORT_CIRCUIT,line_status);
        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! no voltage status! A/C phase short! group:%d\r\n", group);
        return true;
      }
      if ((over_result_ic.limit_flag == true && over_result_ib.limit_flag) || (result_brk_ic && result_brk_ib)) //bc相相间短路
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_BC_SHORT_CIRCUIT,line_status);
        sal_dbg_printf(DEBUG_FAULT,"coll waveform analysis! no voltage status! B/C phase phase short! group:%d\r\n", group);
        return true;
      }
    }
    /*******************无压情况下，判断电流越限或突变*，属于短路*********/
    //if (line_status = vol_none&&break_result_ua.limit_flag == false && break_result_ub.limit_flag == false && break_result_uc.limit_flag == false)
    if (line_status == vol_none)
    {
      if (over_result_ia.limit_flag || result_brk_ia)
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_A_SHORT_CIRCUIT,line_status);//参数2  0A短路   1B墩路 2C短路   3AB短路  4AC短路 5BC短路  6 ABc短路  参数3线路状态
        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! no voltage status! A phase current over or up break! group:%d\r\n", group);
        return true;
      }
      if (over_result_ib.limit_flag || result_brk_ib)
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_B_SHORT_CIRCUIT,line_status);
        sal_dbg_printf(DEBUG_FAULT,"coll waveform analysis! no voltage status! B phase current over or up break! group:%d\r\n", group);
        return true;
      }
      if (over_result_ic.limit_flag || result_brk_ic)
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_C_SHORT_CIRCUIT,line_status);
        sal_dbg_printf(DEBUG_FAULT,"coll waveform analysis! no voltage status! C phase current over or up break! group:%d\r\n", group);
        return true;
      }
    }
    if (line_status == vol_none)
    {
      sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! no voltage status! can not judge fault type! group:%d\r\n", group);
      return true;
    }
    /*******************正常情况下判断相间短路**********/
    if (break_result_uc.limit_flag && break_result_ub.limit_flag && break_result_ua.limit_flag &&
        (break_result_uc.direction == 2) && (break_result_ub.direction == 2) && (break_result_ua.direction == 2)) //ABC相间短路
    {
      if ((over_result_ic.limit_flag == true && over_result_ib.limit_flag && over_result_ia.limit_flag) || (result_brk_ic && result_brk_ib && result_brk_ia)) //abc相相间短路
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_ABC_SHORT_CIRCUIT,line_status);//参数2  0A短路   1B墩路 2C短路   3AB短路  4AC短路 5BC短路  6 ABc短路  参数3线路状态
        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A/B/C phase short! group:%d\r\n", group);
        return true;
      }
    }
    if (break_result_ua.limit_flag && break_result_ub.limit_flag && (break_result_ua.direction == 2) && (break_result_ub.direction == 2)) //AB相间短路 AB两相相电场强度突变,跌落
    {
      if ((over_result_ia.limit_flag && over_result_ib.limit_flag) || (result_brk_ia && result_brk_ib)) //ab相间短路
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_AB_SHORT_CIRCUIT,line_status);
        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A/B phase short! group:%d\r\n", group);
        return true; //返回true 录波
      }
    }
    if (break_result_ua.limit_flag && break_result_uc.limit_flag && (break_result_ua.direction == 2) && (break_result_uc.direction == 2)) //Ac相间短路
    {
      if ((over_result_ia.limit_flag && over_result_ic.limit_flag) || (result_brk_ia && result_brk_ic)) //ac相相间短路
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_AC_SHORT_CIRCUIT,line_status);
        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A/C phase short! group:%d\r\n", group);
        return true;
      }
    }
    if (break_result_uc.limit_flag && break_result_ub.limit_flag && (break_result_uc.direction == 2) && (break_result_ub.direction == 2)) //BC相间短路
    {
      if ((over_result_ic.limit_flag == true && over_result_ib.limit_flag) || (result_brk_ic && result_brk_ib)) //bc相相间短路
      {
        outShortCircuitFailureMsg(faultinfo ,TYPE_BC_SHORT_CIRCUIT,line_status);
        sal_dbg_printf(DEBUG_FAULT,"coll waveform analysis! normal voltage status! B/C phase phase short! group:%d\r\n", group);
        return true;
      }
    }
    /************************************************************************/
    /*弧光接地判断
     相间短路现象：相电场强度突变，相电流越限，相电流突变；
     判据：
     短路相的相电场强度跌落；
     短路相的相电流越限；故障点前电流相位差180，故障点后故障相的相电流同相位
    */
    /************************************************************************/
    if((over_result_ia.limit_flag == false) && (over_result_ib.limit_flag == false) && (over_result_ic.limit_flag == false)){
             result = math_cur_zero_peak();
         if(result >= p->analog_vol.arc_peak_num)
          {
          result= math_cur_phase_peak(0);
          U8 result1= math_cur_phase_peak(1);
          U8 result2= math_cur_phase_peak(2); 
            if((result>result1)&&(result>result2)){
              outEarthFaultMsg(faultinfo,0,0);// 参数3故障相 0A 1B 2C ；参数2 故障类别 0弧光 1高阻  2金属  3小电阻
              sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A arc ground! group:%d\r\n", group);
          return true;          
            }else if((result1>result)&&(result1>result2)){//b相
         outEarthFaultMsg(faultinfo,0,1); 
         sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! B arc ground! group:%d\r\n", group);
          return true;             
            }else{//C相
          outEarthFaultMsg(faultinfo,0,2);
          sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! C arc ground! group:%d\r\n", group);
          return true;            
            }
          } 
    }
    /*
     判断接地故障
     接地故障：金属性接地、小电阻接地，如果两相都下降该怎么判？
     判据：
     金属性接地、小电阻接地：故障相相电场强度跌落，非故障相相电场强度上升，故障相相电流突变；
   */
    {
		float curCoefficienta =  fabs((group_value_data[phase_a][cur_channel][6]+group_value_data[phase_a][cur_channel][7])/2)/group_value_data[phase_a][cur_channel][0];
		float curCoefficientb =  fabs((group_value_data[phase_b][cur_channel][6]+group_value_data[phase_b][cur_channel][7])/2)/group_value_data[phase_b][cur_channel][0];
		float curCoefficientc =  fabs((group_value_data[phase_c][cur_channel][6]+group_value_data[phase_c][cur_channel][7])/2)/group_value_data[phase_c][cur_channel][0];
      if (break_result_ua.limit_flag && break_result_ua.direction == 2) //A相相电场突变，跌落
      {
        if(break_result_ua.ratio<=p->analog_vol.low_ratio &&break_result_ua.ratio>0){//a高阻
			if(break_result_ub.limit_flag && break_result_ub.direction == 2&&break_result_ub.ratio<=p->analog_vol.low_ratio&&break_result_ub.ratio>0){
				if(break_result_uc.limit_flag && break_result_uc.direction == 2&&break_result_uc.ratio<=p->analog_vol.low_ratio&&break_result_uc.ratio>0){//abc都符合
					if((break_result_ua.ratio>break_result_ub.ratio)&&(break_result_ua.ratio>break_result_uc.ratio)){
                        outEarthFaultMsg(faultinfo,1,0);// 参数3故障相 0A 1B 2C ；参数2 故障类别 0弧光 1高阻  2金属  3小电阻
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase big resist ground! group:%d\r\n", group);
			return true;
					}
				}else{//ab符合  c不符合
					if(break_result_ua.ratio>break_result_ub.ratio){
                        outEarthFaultMsg(faultinfo,1,0); 
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase big resist ground! group:%d\r\n", group);
			return true;					
					
					}
				}
			
			}else{
				if(break_result_uc.limit_flag && break_result_uc.direction == 2&&break_result_uc.ratio<=p->analog_vol.low_ratio&&break_result_uc.ratio>0){//ac符合  b不符合
					if(break_result_ua.ratio>break_result_uc.ratio){
                        outEarthFaultMsg(faultinfo,1,0);
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase big resist ground! group:%d\r\n", group);
			return true;
					}
				}else{//a 符合 bc不符合
                        outEarthFaultMsg(faultinfo,1,0);
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase big resist ground! group:%d\r\n", group);
			return true;
				}
			}

        }else if(break_result_ua.ratio>=p->analog_vol.low_ratio&&!(curCoefficienta>1.2)){ //A相金属性接地
			if(break_result_ub.limit_flag && break_result_ub.direction == 2&&break_result_ub.ratio>=p->analog_vol.low_ratio&&!(curCoefficientb>1.2)){//b 金属接地
				if(break_result_uc.limit_flag && break_result_uc.direction == 2&&break_result_uc.ratio>=p->analog_vol.low_ratio&&!(curCoefficientc>1.2)){//abc  都符合
					if((break_result_ua.ratio>break_result_ub.ratio)&&(break_result_ua.ratio>break_result_uc.ratio)){
                        outEarthFaultMsg(faultinfo,2,0);// 参数3故障相 0A 1B 2C ；参数2 故障类别 0弧光 1高阻  2金属  3小电阻 
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase metal ground! group:%d\r\n", group);
			return true;
					}
				}else{//ab符合 c不符合
					if(break_result_ua.ratio>break_result_ub.ratio){
                        outEarthFaultMsg(faultinfo,2,0);
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase metal ground! group:%d\r\n", group);
			return true;					
					}
				}
			}else{
				if(break_result_uc.limit_flag && break_result_uc.direction == 2&&break_result_uc.ratio>=p->analog_vol.low_ratio&&!(curCoefficientc>1.2)){//ac符合  b不符合
					if(break_result_ua.ratio>break_result_uc.ratio){
                        outEarthFaultMsg(faultinfo,2,0);
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase metal ground! group:%d\r\n", group);
			return true;
					}
				}else{//a符合  bc不符合
                        outEarthFaultMsg(faultinfo,2,0);
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase metal ground! group:%d\r\n", group);
			return true;
				}
			}
        }else if(break_result_ua.ratio>p->analog_vol.low_ratio&&(curCoefficienta>1.2)){ //A相小电阻接地

			if(break_result_ub.limit_flag && break_result_ub.direction == 2&&break_result_ub.ratio>p->analog_vol.low_ratio&&(curCoefficientb>1.2)){//b 小电阻
				if(break_result_uc.limit_flag && break_result_uc.direction == 2&&break_result_uc.ratio>p->analog_vol.low_ratio&&(curCoefficientc>1.2)){//abc 都 符合
					if((break_result_ua.ratio>break_result_ub.ratio)&&(break_result_ua.ratio>break_result_uc.ratio)){
                        outEarthFaultMsg(faultinfo,3,0);// 参数3故障相 0A 1B 2C ；参数2 故障类别 0弧光 1高阻  2金属  3小电阻
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase little resist ground! group:%d\r\n", group);
			return true;
					}
				}else{//ab符合  c不符合
					if(break_result_ua.ratio>break_result_ub.ratio){
                        outEarthFaultMsg(faultinfo,3,0);
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase little resist ground! group:%d\r\n", group);
			return true;
					}
				}
			
			}else{
				if(break_result_uc.limit_flag && break_result_uc.direction == 2&&break_result_uc.ratio>p->analog_vol.low_ratio&&(curCoefficientc>1.2)){//ac符合  b不符合
					if(break_result_ua.ratio>break_result_uc.ratio){
                        outEarthFaultMsg(faultinfo,3,0); 
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase little resist ground! group:%d\r\n", group);
			return true;
					}
				}else{//a符合  bc不符合
                        outEarthFaultMsg(faultinfo,3,0);
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! A phase little resist ground! group:%d\r\n", group);
			return true;
				}
			}
        }
      }
      if (break_result_ub.limit_flag && break_result_ub.direction == 2) //B相相电场突变，跌落
      {

        if(break_result_ub.ratio<=p->analog_vol.low_ratio&&break_result_ub.ratio>0){ //B相高阻接地

			if(break_result_uc.limit_flag && break_result_uc.direction == 2&&break_result_uc.ratio<=p->analog_vol.low_ratio&&break_result_uc.ratio>0){//bc符合
				if(break_result_ub.ratio>break_result_uc.ratio){
                        outEarthFaultMsg(faultinfo,1,1);// 参数3故障相 0A 1B 2C ；参数2 故障类别 0弧光 1高阻  2金属  3小电阻
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! B phase big resist ground! group:%d\r\n", group);
			return true;
				}
			
			}else{
                        outEarthFaultMsg(faultinfo,1,1);
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! B phase big resist ground! group:%d\r\n", group);
			return true;
			}    
        }
        else if(break_result_ub.ratio>=p->analog_vol.low_ratio&&!(curCoefficientb>1.2)){ //B相金属性接地
			if(break_result_uc.limit_flag && break_result_uc.direction == 2&&break_result_uc.ratio>=p->analog_vol.low_ratio&&!(curCoefficientc>1.2)){//bc符合
                        outEarthFaultMsg(faultinfo,2,1);// 参数3故障相 0A 1B 2C ；参数2 故障类别 0弧光 1高阻  2金属  3小电阻
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! B phase metal ground! group:%d\r\n", group);
			return true;
			}else{
                        outEarthFaultMsg(faultinfo,2,1);
                        sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! B phase metal ground! group:%d\r\n", group);
			return true;			
			}
        }
            else if(break_result_ub.ratio>p->analog_vol.low_ratio&&(curCoefficientb>1.2)){ //B相小电阻接地
				if(break_result_uc.limit_flag && break_result_uc.direction == 2&&break_result_uc.ratio>=p->analog_vol.low_ratio&&(curCoefficientc>1.2)){//bc符合
					if(break_result_ub.ratio>break_result_uc.ratio){
                                outEarthFaultMsg(faultinfo,3,1);// 参数3故障相 0A 1B 2C ；参数2 故障类别 0弧光 1高阻  2金属  3小电阻
                                sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! B phase little resist ground! group:%d\r\n", group);
				return true;					
					}
				}else{
                                outEarthFaultMsg(faultinfo,3,1);
                                sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! B phase little resist ground! group:%d\r\n", group);
				return true;				
				}
        }
      }
      if (break_result_uc.limit_flag && break_result_uc.direction == 2) //C相相电场突变，跌落
      {
        if(break_result_uc.ratio<=p->analog_vol.low_ratio&&break_result_uc.ratio>0){//C相高阻接地
          outEarthFaultMsg(faultinfo,1,2);// 参数3故障相 0A 1B 2C ；参数2 故障类别 0弧光 1高阻  2金属  3小电阻
          sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! C phase big resist ground! group:%d\r\n", group);
        }
        else if(break_result_uc.ratio>=p->analog_vol.low_ratio&&!(curCoefficientc>1.2)){ //C相金属性接地
          outEarthFaultMsg(faultinfo,2,2);
          sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! C phase metal ground! group:%d\r\n", group);
       }
       else if(break_result_uc.ratio>=p->analog_vol.low_ratio&&(curCoefficientc>1.2)){ //C相小电阻接地
         outEarthFaultMsg(faultinfo,3,2);
         sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! C phase little resist ground! group:%d\r\n", group);
        }
            return true;
      }
      
      /*
        防误动作//试用
      */
      if(collparam.common_enable_switch&0x0004){//启用防误触发//常用开关第三位
      if(!doErrorAction(0,0)&&!doErrorAction(0,1)&&!doErrorAction(0,2)){//三相电压相邻两周波突变是否超过5%
        return false;
      }
    }        
      }

    
    
    sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! coll unit can not judge fault type!\r\n");
    break;
  case system_small_current_grounded:
    sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! coll unit can not judge fault type!\r\n");
    break;
  case system_arc_grounded:
    sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! coll unit can not judge fault type!\r\n");
    break;
  default:
    sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! normal voltage status! coll unit can not judge fault type!\r\n");
    break;
  }
  return true;
}
/************************************************************************
数据源：A\B\C三相电压，三相电流
判断相间短路故障：
输入：A\B\C三相电压电流波形数据
零序电流
相间短路故障：两相电压突然跌落，相电流突然增加，

输入：group,组号
event_t 产生故障波形时的事件
判断出故障后发送消息到GPRS模块

返回值：true 表示录波，false 不录波
************************************************************************/


U8 waveform_analysis(stufaultinfo *faultinfo)
{
  static unsigned long lastTimems=0;
  static  U8 firstJudge=true;
  unsigned long nowTimems;
  

  stuSystime *time_t;

  U8  line_status; //1：上电，2：失电，3：故障波形
  stuUnitparam *p;
  U8 group;
  U8 phase;
  U16 fault_code;
  stuevent_t event_t;
  group = faultinfo->group;
  phase = faultinfo->phase;
  U8 fff = transperflag; 
  event_t = faultinfo->event;
  time_t = &event_t.event_time;
  nowTimems=LocalTime_to_Utc(time_t) *1000+time_t->MS;
  if(firstJudge)
  {
    firstJudge=false;
    lastTimems=nowTimems;
    
  }
  else
  {
    if(abs(nowTimems-lastTimems)<100)
      return false;
    else
      lastTimems=nowTimems;
  }
  
  p = &collparam.param_group[group].unit_param[phase_b];
  computeEffective(group);
  fault_info.faultdata[0].vol_data =group_value_data[phase_a][vol_channel][5];
  fault_info.faultdata[1].vol_data =group_value_data[phase_b][vol_channel][5];
  fault_info.faultdata[2].vol_data =group_value_data[phase_c][vol_channel][5];
  fault_info.faultdata[0].curr_data = group_value_data[phase_a][cur_channel][5];
  fault_info.faultdata[1].curr_data = group_value_data[phase_b][cur_channel][5];
  fault_info.faultdata[2].curr_data = group_value_data[phase_c][cur_channel][5];  
  fault_info.fault_time = *time_t;
  fault_code = line1_unkown_fault;              
  //faultinfo->group = group; //故障组
  //faultinfo->phase = 0;
  faultinfo->judgetype = fault_code; 
  //手动录波
  fault_info.event_code = fault_code;
  fault_info.group = group;
  fault_info.phase = phase;
 // fault_info.fault_code = faultinfo->event.event_code;
  memcpy(&fault_info.fault_code,&faultinfo->event.event_code,1);
/*  transperflag = 0;
  if(collstatus.Unit_dev_info[group][0].Dev_status & 0x00000004)
  {
   transperflag = 1;
  }
 else if(collstatus.Unit_dev_info[group][0].Dev_status & 0x00000008)
 {
   transperflag |= (1<<1);
  }
  if(collstatus.Unit_dev_info[group][1].Dev_status & 0x00000004)
  {
   transperflag |= (1<<2);
  }
  else if(collstatus.Unit_dev_info[group][1].Dev_status & 0x00000008)
  {
   transperflag |= (1<<3);
  }
  if(collstatus.Unit_dev_info[group][2].Dev_status & 0x00000004)
  {
   transperflag |= (1<<4);
  }
  else if(collstatus.Unit_dev_info[group][2].Dev_status & 0x00000008)
 {
  transperflag |= (1<<5);
  }
*/
  memcpy(&fault_info.transperflag,&transperflag,1); 
  if (event_t.event_type==TYP_HANDLE_EVENT)
  {
    fault_code = line_hand_wave_id;
    //faultinfo->group = group; //故障组
    //faultinfo->phase = 0;
    faultinfo->judgetype = fault_code; 
    //手动录波
    fault_info.event_code = fault_code;
    sal_dbg_printf(DEBUG_FAULT, "coll waveform analysis! hand event! group:%d vol_a:%f vol_b:%f vol_c:%f cur_a:%f cur_b:%f cur_c:%f\r\n", 
                   group, fault_info.faultdata[0].vol_data, fault_info.faultdata[1].vol_data, fault_info.faultdata[2].vol_data, fault_info.faultdata[0].curr_data, fault_info.faultdata[1].curr_data, fault_info.faultdata[2].curr_data);
    return true;
  }
  line_status = judgeVoltageStatus((float)(p->analog_vol.critical_Value),(float)(p->analog_cur.critical_Value));
  if (line_status == vol_up)
  {
    sal_dbg_printf(DEBUG_FAULT, "collect unit line voltage up, do not judge fault!\r\n");
    return false; //返回false不录波
  }
  if (line_status == vol_down)
  {
    sal_dbg_printf(DEBUG_FAULT, "collect unit line voltage down, do not judge fault!\r\n");
    return false;  //返回false不录波
  }
  sal_dbg_printf(DEBUG_FAULT, "collect unit judge line status:%d group:%d phase:%d\r\n", line_status, group, phase);
  
 /* U8 peakValue = math_cur_zero_peak();//0序电流峰值数
  if(peakValue<1)
    return false;//返回false不录波*/
  
  return doFaultTypeJudge(faultinfo,line_status);
}
/*
弧光接地、高祖接地判断
高祖接地时，故障相存在相位突变，零序电压上升，零序电流比较小；
弧光接地：
故障相电压跌落，零序电流存在放电尖峰；
U8 Hi_reigster_ground(I16 *src,U8 group,U8 phase)
*/
/************************************************************************/


void toUTCtime(stuSystime systime, stuUtctime *utctime)
{
  utctime->sec = LocalTime_to_Utc(&systime);
  utctime->ms = systime.MS;
}


void add_event_list(stuevent_t *pedata)
{
  stuEvent_list *pevent = NULL;
  U16 event_index;
  pevent = &Event_list;
  if (pevent->event_new_num >= 64)
  {
    pevent->event_new_num = 64;
  }
  else
  {
    pevent->event_new_num++;
  }

  if (pevent->event_total_num >= 64)
  {
    pevent->event_total_num = 64;
  }
  else
  {
    pevent->event_total_num++;
  }
#ifdef WIN32
  FILE *file = NULL;
  string log_file = "log.txt";
  event_index = pevent->event_new_index; //U8 event_new_index; //新事件引号
  memcpy(&pevent->Log_listitem[event_index], pedata, sizeof(stuevent_t));
  pevent->event_new_index = (pevent->event_new_index + 1) % 64;
  file = fopen((char *)&log_file, "w+");
  if (file == NULL)
  {
    return;
  }
  fwrite(pedata, 1, sizeof(stuevent_t), file);
#else
  event_index = pevent->event_new_index; //U8 event_new_index; //新事件引号
  memcpy(&pevent->Log_listitem[event_index], pedata, sizeof(stuevent_t));
  pevent->event_new_index = (pevent->event_new_index + 1)%64;
  //EventListWrite(pevent);
#endif
}

