#include "imu_kalman.h"
float Yaw_gyro,Roll_gyro,Pitch_gyro;
float Yaw_mag,Roll_accel,Pitch_accel;
float Yaw,Roll,Pitch,Yaw_Offset,Pitch_Offset,Roll_Offset;
static bool init_flag = 0;
#include "arm_math.h"


void AngleFromAccelMag(const float* accel, const float* mag, float* angles) {
    
    float normFactor = inv_sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    float ax = accel[0] * normFactor;
    float ay = accel[1] * normFactor;
    float az = accel[2] * normFactor;
    
    normFactor = inv_sqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
    float mx = mag[0] * normFactor;
    float my = mag[1] * normFactor;
    float mz = mag[2] * normFactor;
    
    float roll = atan2(-ay, az);        // Roll [az=g not -g]
    float pitch = asin(-ax);            // Pitch
    float yaw = atan2(-my*cos(roll)+mz*sin(roll), 
        mx*cos(pitch)+my*sin(roll)*sin(pitch)+mz*cos(roll)*sin(pitch)); // Yaw
    
    angles[0] = roll;
    angles[1] = pitch;
    angles[2] = yaw;
}

void KalmanFilter(
        Kalman_t* kalman, 
        const float gyro[3], 
        const float accel[3],
        const float mag[3], float dt) 
{
	if(init_flag == 0){
		unsigned char i;
		float ax,ay,az,a;
		ax=ay=az=0.0f;
		for(i=0;i<5;i++){
			ax+=accel[0];
			ay+=accel[1];
			az+=accel[2];
			delay_ms(5);
		}
		a=1.0f/sqrt(ax*ax+ay*ay+az*az);
		ax*=a;
		ay*=a;
		az*=a;
		Roll=atan2(-ay,az);
		Pitch=asin(ax);
		Yaw=0;
		init_flag = 1;
	}
	else{
		float Roll_sin,Roll_cos,Pitch_sin,Pitch_cos;
		float a,gx,gy,gz,dx,dy,dz;
		static const float K=1.0f;  //
		static const float KP=0.5f; //
		Roll_sin=sin(Roll);
		Roll_cos=cos(Roll);
		Pitch_sin=sin(Pitch);
		Pitch_cos=cos(Pitch);
		
		a=inv_sqrt(accel[0]*accel[0]
			+accel[1]*accel[1]
		+accel[2]*accel[2]);

		dx=accel[1]*Pitch_cos*Roll_cos-accel[2]*Pitch_cos*Roll_sin;
		dy=accel[2]*(-Pitch_sin)-accel[0]*Pitch_cos*Roll_cos;
		dz=accel[0]*Pitch_cos*Roll_sin+accel[1]*Pitch_sin;
		
		dx *=a;
		dy *=a;
		dz *=a;

		gx=gyro[0]+KP*dx;
		gy=gyro[1]+KP*dy;
		gz=gyro[2]+KP*dz;
		
		
		Roll=Roll+
		(Pitch_cos*gx+Pitch_sin*Roll_sin*gy+Pitch_sin*Roll_cos*gz)/Pitch_cos*dt;
		
		Pitch=Pitch+(Roll_cos*gy-Roll_sin*gz)*dt;
		
		Yaw_gyro=Yaw+
		(Roll_sin*gy+Roll_cos*gz)/Pitch_cos*dt;
		
		if(Yaw_gyro>PI){	
			Yaw_gyro-=2*PI;
		}else if(Yaw_gyro<-PI){
			Yaw_gyro+=2*PI;
		}
		if(Roll>PI){	
			Roll-=2.0f*PI;
		}else if(Roll<-PI){
			Roll+=2.0f*PI;
		}
		
		if(Pitch>PI/2.0f){	
			Pitch-=PI;
		}else if(Pitch<-PI/2.0f){
			Pitch+=PI;
		}
		
		Yaw=K*Yaw_gyro+(1.0f-K)*mag[2];
		
		kalman->euler.yaw = Yaw;
		kalman->euler.pitch = Pitch;
		kalman->euler.roll = Roll;
		
	}
}
