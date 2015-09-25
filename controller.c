/**
 *@file  controller.c
 *@brief Implements the Control Law
*/

#include "common.h"
#include "controller.h"
#include "mathutil.h"
#include "propagator.h"
#include "quest.h"
#include "timer.h"
#include "peripherals.h"
#include "frame.h"
#include "gps.h"

static vector v_B;
static vector v_w = { 0.0, 0.0, 0.0 };
static quaternion q_o;
static uint8_t first_B = 1, light, w_ctrl = 1;


static matrix m_Kp = { { 2.31974732995976e-07, -9.68862003823701e-10, -8.16250915587234e-10 },
                       { -9.68862003823701e-10, 2.34054275693572e-07, -3.27879997319606e-09 },
                       { -8.16250915587234e-10, -3.27879997319606e-09, 2.16304237871903e-07 } };
                         
static matrix m_Ki = { { 4.40842904803014e-12, -1.84121750934969e-14, -1.55119663261705e-14 },
                       { -1.84121750934969e-14, 4.44794850912104e-12, -6.23100492792408e-14 },
                       { -1.55119663261705e-14, -6.23100492792408e-14, 4.1106282271831e-12 } };
                         
static matrix m_Kd = { { 0.000451402853158012, -1.88532202276527e-06, -1.58835398765308e-06 },
                       { -1.88532202276527e-06, 0.00045544946416101, -6.38026238340704e-06 },
                       { -1.58835398765308e-06, -6.38026238340704e-06, 0.000420909419161786 } };

void detumbling(vector v_m_D)
{
  static vector v_B_old;
  if(first_B)
  {
    copy_vector(v_B, v_B_old);
    first_B = 0;
    return;
  }
  
  vector v_B_avg, v_dB;
  float factor;
  uint8_t i;
  
  for(i = 0; i < 3; i++)
  {
    v_dB[i] = (v_B[i] - v_B_old[i]) / FRAME_TIME;
    v_B_avg[i] = (v_B[i] + v_B_old[i]) / 2;
  }
  factor = (-1 * K_DETUMBLING * MAG_B) / vector_norm(v_B_avg);
  
  for(i = 0; i < 3; i++)
    v_m_D[i] = factor * v_dB[i];
  
  copy_vector(v_B, v_B_old);
}

void nominal(vector v_m_N)
{
  static vector v_ieu = { 0.0, 0.0, 0.0 };
  vector v_eu, v_m_temp;
  uint8_t i, j;
  float norm_B = vector_norm(v_B);
    
  for(i = 0; i < 3; i++)
  {
    v_eu[i] = 2 * q_o[i] * q_o[3];
    v_ieu[i] += v_eu[i] * FRAME_TIME;
  }
    
  for(i = 0; i < 3; i++)
  {
    v_m_temp[i] = 0;
    for(j = 0; j < 3; j++)
      v_m_temp[i] += v_eu[j] * m_Kp[i][j] + v_ieu[j] * m_Ki[i][j] + v_w[j] * m_Kd[i][j];
    v_m_temp[i] /= norm_B * norm_B;
  }
    
  vector_cross_product(v_m_temp, v_B, v_m_N);
  scalar_into_vector(v_m_N, light);//scalar_into_vector(v_m_N, light);//values are very small as compared to matlab check
   
}

void apply_torque(vector v_m)
{
	uint8_t i;
	/*uint8_t sen;
	for(int i=0;i<3;i=i+1)
	{
		sen = (v_m[i]*255);
		transmit_UART0(sen);
	}*/
    
	
	for(i = 0; i < 3; i++)
	{
		if(fabs(v_m[i]) > M_MAX)
		{
			if (fabs(v_m[i])/v_m[i] == -1)
			v_m[i] = -1*M_MAX;
			else
			v_m[i] = M_MAX;
		}
		
	}
	
	
	
	scalar_into_vector(v_m, 1.0 / (N_TURNS * AREA));
	
	
	for(i = 0; i < 3; i++)
	{
		if(fabs(v_m[i]) > I_MAX)
		{
			if (fabs(v_m[i])/v_m[i] == -1)
			v_m[i] = -1*I_MAX;
			else
			v_m[i] = I_MAX;
		}
	}
/*uint8_t sen;
for (int i=0;i<3;i++)
{
	
	sen = ((uint8_t)abs(v_m[i]))*100000;
	transmit_UART0(sen);
	
}*/

//Current_state.pwm.x = abs(v_m[0]*65535); //fabs((v_m_D[0] * PWM_RES) / I_MAX)+10000;
// int16_t x1 = Current_state.pwm.x/100;
// if (x1 == 0)
//     Current_state.pwm.x = Current_state.pwm.x + 100;
if(v_m[0]>=0)
Current_state.pwm.x=v_m[0]*65535;
else
Current_state.pwm.x=(-1)*v_m[0]*65535;

/*uint8_t sen=0xf0;transmit_UART0(sen);
for(int i=0;i<3;i=i+1)
{
	if(Current_state.pwm.x==0)
	{ sen=0x0F;}
	else
	sen= 0xf0;   //v_r_ecef[i]
	transmit_UART0(sen);
}*/

Current_state.pwm.y = abs(v_m[1]*65535); //fabs((v_m_D[1] * PWM_RES) / I_MAX)+10000;
// x1 = Current_state.pwm.y/100;
// if (x1 == 0)
//     Current_state.pwm.y = Current_state.pwm.y + 100;

Current_state.pwm.z = abs(v_m[2]*65535); //fabs((v_m_D[2] * PWM_RES) / I_MAX)+10000;
// x1 = Current_state.pwm.z/100;
// if (x1 == 0)
//     Current_state.pwm.z = Current_state.pwm.z + 100;

if(v_m[0] < 0)
Current_state.pwm.x_dir = 1;
else
Current_state.pwm.x_dir = 0;

if(v_m[1] < 0)
Current_state.pwm.y_dir = 1;
else
Current_state.pwm.y_dir = 0;

if(v_m[2] < 0)
Current_state.pwm.z_dir = 1;
else
Current_state.pwm.z_dir = 0;
PORTA=0xA0;
}
/*
void apply_torque(vector v_m)
{
  uint8_t i;
  
  for(i = 0; i < 3; i++)
  {
    if(v_m[i] > M_MAX)
      v_m[i] = M_MAX;
  }
  
  scalar_into_vector(v_m, 1.0 / (N_TURNS * AREA));
  
  for(i = 0; i < 3; i++)
  {
    if(v_m[i] > I_MAX)
      v_m[i] = I_MAX;
  }
  
  Current_state.pwm.x = fabs((v_m[0] * PWM_RES) / I_MAX);
  Current_state.pwm.y = fabs((v_m[1] * PWM_RES) / I_MAX);
  Current_state.pwm.z = fabs((v_m[2] * PWM_RES) / I_MAX);
  
  if(v_m[0] < 0)
    Current_state.pwm.x_dir = 1;
  if(v_m[1] < 0)
    Current_state.pwm.y_dir = 1;
  if(v_m[2] < 0)
    Current_state.pwm.z_dir = 1;
}
*/
void control(void){
  
  vector v_m_D, v_m_N, v_sun_o, v_B_o;
  static uint64_t nominal_end = 0;
  
  ///Watchdog enabled for the control law
  //watch_dog(T_CONTROL);
  
  /// Take B readings from Magnetometer, clear torquer first
  //reset_PWM();
  
  //_delay_us(100);
  
  read_MM();
  
  ///Set the torquer values calculated in the last frame
  //set_PWM();
  uint8_t c1,c2,d;
  c1= (uint8_t)Current_state.pwm.x;
  c2= (uint8_t)((Current_state.pwm.x)>>8);
  d= Current_state.pwm.x_dir;
  transmit_UART0(c1);
  transmit_UART0(c2);
  transmit_UART0(d);
  
  c1= (uint8_t)Current_state.pwm.y;
  c2= (uint8_t)((Current_state.pwm.y)>>8);
  d= Current_state.pwm.y_dir;
  transmit_UART0(c1);
  transmit_UART0(c2);
  transmit_UART0(d);
  
  c1= (uint8_t)Current_state.pwm.z;
  c2= (uint8_t)((Current_state.pwm.z)>>8);
  d= Current_state.pwm.z_dir;
  transmit_UART0(c1);
  transmit_UART0(c2);
  transmit_UART0(d);
  
  v_B[0] = Current_state.mm.B_x;
  v_B[1] = Current_state.mm.B_y;
  v_B[2] = Current_state.mm.B_z;
  
  
 // detumbling(v_m_D);
  
  //if(((GPS_done == 0) && (Time % 600 == 0)) || ((GPS_done < 0) && (!first_B) && (vector_norm(v_m_D) < 2 )))
    ///* Switch on the GPS First
   uint8_t q= 90;
    transmit_UART0(q);
    read_GPS();
    while(UCSR0B & _BV(RXCIE0));
	uint8_t a,b,c;
	
	/*for (int i=1;i<5;i=i+1)
	{
		uint8_t a1= receive_UART0();
	}
	*/
	/*
	a = (uint8_t)Current_state.gps.x;
	transmit_UART0(a);
	//Current_state.gps.x=Current_state.gps.x>>8;
	b = (uint8_t)(Current_state.gps.x>>8);
	transmit_UART0(b);
	c = (uint8_t)(Current_state.gps.x>>16);
	transmit_UART0(c);
	d = (uint8_t)(Current_state.gps.x>>24);
	transmit_UART0(d);
	
	a = (uint8_t)Current_state.gps.y;
	transmit_UART0(a);
	//Current_state.gps.x=Current_state.gps.x>>8;
	b = (uint8_t)(Current_state.gps.y>>8);
	transmit_UART0(b);
	c = (uint8_t)(Current_state.gps.y>>16);
	transmit_UART0(c);
	d = (uint8_t)(Current_state.gps.y>>24);
	transmit_UART0(d);
	
	a = (uint8_t)Current_state.gps.z;
	transmit_UART0(a);
	//Current_state.gps.x=Current_state.gps.x>>8;
	b = (uint8_t)(Current_state.gps.z>>8);
	transmit_UART0(b);
	c = (uint8_t)(Current_state.gps.z>>16);
	transmit_UART0(c);
	d = (uint8_t)(Current_state.gps.z>>24);
	transmit_UART0(d);*/
	//receiving faltu last 4 bytes of gps to be looked later
  
  
  /// Check if in Nominal or Detumbling Mode
  //if(GPS_done >= 0)
  //{
	   
    if(GPS_done == 1)
      copy_gps_reading();  //check
	  
    uint8_t e=100;
    transmit_UART0(e);
    read_SS();
	/*uint8_t sen;
	for(int i=0;i<6;i=i+1)
	{
		sen = (Current_state.ss.read[i]*255/3.3);
		transmit_UART0(sen);
	}*/
	/*
	uint8_t cs,ds;
	int i=0;
	while(i<6)
	{
		//PORTA=0x00;
		cs = (uint8_t) (Current_state.ss.reading[i]>>8);
		transmit_UART0(cs);
		PORTA=0x00;
		//Current_state.gps.x=Current_state.gps.x>>8;
		ds = (uint8_t)(Current_state.ss.reading[i]);
		transmit_UART0(ds);
		i=i+1;
	}
	Current_state.pwm.x ^= 0xffff;
	Current_state.pwm.y ^= 0xffff;
	Current_state.pwm.z ^= 0xffff;*/
    ///////////////////////////////////////////////////////////////////////////////////
    sgp_orbit_propagator();
	
    sun_vector_estimator(v_sun_o);
	
    magnetic_field_estimator(v_B_o);
    
	
    light = quest(v_B_o, v_sun_o, q_o, &w_ctrl); // to be checked
    
    omega_estimation(q_o, v_w);
	
    scalar_into_vector(v_w, light*w_ctrl); //light * w_ctrl
 
     
    convert_unit_quaternion(q_o);
	int8_t sen;
	for (int i=0;i<4;i=i+1)
	{
		sen = ((uint8_t)(q_o[i]*100));
		transmit_UART0(sen);
	}
	
	/* 
    nominal(v_m_N);
    //uint8_t sen;
    /*for (int i=0;i<3;i=i+1)
    {
	    sen = ((uint8_t)abs(v_sun_o[i]))*255;
	    transmit_UART0(sen);
		
		sen = ((uint8_t)abs(v_B_o[i]))*255;
		transmit_UART0(sen);
		
		sen = ((uint8_t)abs(v_m_N[i]))*255;
		transmit_UART0(sen);
    }*/
	
	/*for (int i=0;i<4;i=i+1)
	{
		sen = ((uint8_t)abs(q_o[i]))*100;
		transmit_UART0(sen);
	}*/
	
    /*seconds_since_equinox += FRAME_TIME;
    seconds_since_pivot += FRAME_TIME;
    Current_state.gps.time_since_reading += FRAME_TIME;
    
    float norm_w = vector_norm(v_w);
    
    //Confirm/ Check
    /*if((norm_w < TOLW_D2N) && ((light * w_ctrl) == 1) && Mode == DETUMBLING && Time > D_TIME)
    {
      Mode = NOMINAL;
      nominal_end = Time + N2D_TIME;
    }
    else if((norm_w > TOLW_N2D) && Mode == NOMINAL && Time > nominal_end)
      Mode = DETUMBLING;
  
  
  if(Mode == DETUMBLING)
    apply_torque(v_m_D);
  
  else if(Mode == NOMINAL)*/
    apply_torque(v_m_N);
    
}
