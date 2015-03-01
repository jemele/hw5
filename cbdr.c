#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

typedef struct {
    // spacial parameters
    double thetaH, thetaH_rate, thetaH_rate_2; //rates are in rad/ms
    double range, range_rate, range_rate_2;  //range is in mm
    
    // wheel speed
    int speed_left; // mm/ms
    int speed_right;
    
    // time data
    int delta_t; // ms
    int external_time; // ms
    
    // flags 1, 0
    int detection; 
    int course_flag;
    int range_rate_flag;
} location_data;

typedef struct {
    // x, y in units of pixels; time_m is ms.
    int x, y, time_m;
} camera_data;

const int max_speed = 300; // mm/s
const int time_m_standard = 200; //ms intervals
const int time_m_refresh = 20;
const double pi =3.1415926535897;

#define true = 1;
#define false = 0; 


int camera_data_update(location_data *iRobot, camera_data *camera);
int thetaH_update(location_data *self, int x);
void reduce_angle (double *angle);
int range (location_data *iRobot, int y);
int initial_course_adjustment (location_data *iRobot, camera_data *camera);

void drive (double left_wheel, double right_wheel);

void adjust_course (location_data *iRobot, location_data *target);

int main(int argc, char **argv) {
    location_data iRobot;
    camera_data camera;
    int i;
    int status = 0;

    iRobot.detection = 0;
    iRobot.course_flag = 0;
 //   for (;;) {
        // detection function call here
        if (iRobot.detection == 0) {
            //detect(&iRobot);
            initial_course_adjustment(&iRobot, &camera);
        } else {
            status = camera_data_update(&iRobot, &camera);
        }
        if (status==0) {
            usleep(time_m_standard);
        } else {
            usleep(time_m_refresh);
        }
        
//    }
    printf("exit.\n");
    return 0;
}

int camera_data_update(location_data *iRobot, camera_data *camera) {
    int status = 0;
    // lock access so there isn't a partial read
    // insert function here
    
    int x = camera->x;
    int y = camera->y;
    int t = camera->time_m;
    //unlock so camera can once again write
    // insert function here
    
    if (t==iRobot->external_time) {
        // allows us to inform the caller that we aren't
        // getting new data
        return 1;
    }
    iRobot->delta_t = t-iRobot->external_time;
    iRobot->external_time = t;
    
    status = range(iRobot, y);
    status |= thetaH_update(iRobot, x);
    if (status == 1) {
        return 1;
    }
    return 0;
}


int thetaH_update(location_data *iRobot, int x) {
    if ((x<0)||(x>320)) {
        return 1;
    }
    x-=160;
    double thetaH = (pi/180)*(x/7.4);
    iRobot->thetaH_rate_2 = iRobot->thetaH_rate;
    iRobot->thetaH_rate = 1000*(thetaH-iRobot->thetaH)/iRobot->delta_t;
    iRobot->thetaH = thetaH;
    
    printf("theta H: %f\n", 180*iRobot->thetaH/pi);
    printf("theta H rate: %f\n", 180*iRobot->thetaH_rate/pi);
    return 0;
}

/*
//After initial course is calculated, makes incremental adjustment to course
void adjust_course (location_data *iRobot, location_data *target) {
    if (iRobot->range_rate < 0) {
        iRobot->range_rate_flag = 0;
    }
    if (iRobot->course_flag == 0) {
        iRobot->course_flag++;
        
        // This will determine if the iRobot and target are going opposite directions
        if ((iRobot->thetaH_rate>pi/180)&&(iRobot->range > iRobot->speed*sin(iRobot->thetaH)/sin(iRobot->thetaH_rate))) { // iRobot->speed*sin(iRobot->thetaH-iRobot->thetaH_rate)/sin(iRobot->thetaH_rate)
            initial_course_adjustment(iRobot, target);
            iRobot->course_flag++;
            printf("                                         Target Zig!\n");
        } else {
        
            // This will determine if the iRobot course is not reducing range
            // Or if bearing rate is going up by greater than 1 degree per unit time
            if (((!iRobot->range_rate_flag)&&(iRobot->range_rate > 0))
                ||((fabs(iRobot->thetaH_rate)-fabs(iRobot->thetaH_rate_2))>pi/180)) { 
                printf("                                         Range Rate!\n");
                //iRobot->course = iRobot->thetaH/2  + iRobot->course/2;
                initial_course_adjustment(iRobot, target);
                iRobot->course_flag++;
            } else {
                // fine course adjustments
                if (180*fabs(iRobot->thetaH_rate)/pi> 2) {
                    iRobot->course += 4*iRobot->thetaH_rate;
                } else {
                    iRobot->course += 2*iRobot->thetaH_rate;
                }
                printf("                                        Course: %f\n", 180*iRobot->course/pi);
            }
        }
    }
    iRobot->course_flag--;
}
*/

//using target range and target bearing rate, calculates iRobot initial course


int initial_course_adjustment (location_data *iRobot, camera_data *camera) {
    
    int status = 0;
    
    
    camera->x = 150;
    camera->y = 100;
    camera->time_m = 0;
    
    status = camera_data_update(iRobot, camera);
    if (status == 1) {
        return 1;
    }
    double R1 = iRobot->range;
    
    usleep(time_m_standard);
    
    
    camera->x = 200;
    camera->y = 105;
    camera->time_m = 200;
    
    status = camera_data_update(iRobot, camera);
    if (status == 1) {
        return 1;
    }
    double R2 = iRobot->range;
    

    // deterimine vector length of target speed
    double target_speed = sqrt(R1*R1+R2*R2-(2*R1*R2*cos(iRobot->thetaH_rate)));
    printf("Target_speed: %f\n", target_speed);
    
    if ((1000*(R2-R1)/iRobot->delta_t) > max_speed) {
        
        if (iRobot->thetaH > 0) {
            //code for operating wheels
            drive(max_speed, -max_speed);
            usleep(1583333*iRobot->thetaH/(2*pi));
            drive(max_speed, max_speed);
        }
        if (iRobot->thetaH < 0) {
            //code for operating wheels
            drive(-max_speed, max_speed);
            usleep(1583333*iRobot->thetaH/(2*pi));
            drive(max_speed, max_speed);
        }
        iRobot->range_rate_flag = 1;
        printf("                                         Range Rate Flag\n");
    } else { 
        iRobot->range_rate_flag = 0;
        //determine vector angle of the target, relative to Line of Site
        double target_angle = asin(R2*sin(iRobot->thetaH_rate)/target_speed);
        printf("Target_angle: %f\n", 180*target_angle/pi);
        //Calculate collission course
        double angle = target_speed*sin(target_angle)/max_speed;
        
        printf("angle: %f\n", angle);
        reduce_angle(&angle);
        printf("angle: %f\n", angle);
        double iRobot_angle = asin(angle);// + iRobot->thetaH;
        printf("irobot_angle: %f\n", 180*iRobot_angle/pi);
        if (iRobot->thetaH_rate>0) {
            drive(max_speed, -max_speed);
            usleep(1583333*iRobot_angle/(2*pi));
            drive(max_speed, max_speed);
        }
        if (iRobot->thetaH_rate<0) {
            drive(-max_speed, max_speed);
            usleep(1583333*iRobot_angle/(2*pi));
            drive(max_speed, max_speed);
        }
        if (iRobot->thetaH_rate == 0) {
            if (iRobot->thetaH > 0) {
                //code for operating wheels
                drive(max_speed, -max_speed);
                usleep(1583333*iRobot->thetaH/(2*pi));
                drive(max_speed, max_speed);
            }
            if (iRobot->thetaH < 0) {
                //code for operating wheels
                drive(-max_speed, max_speed);
                usleep(1583333*iRobot->thetaH/(2*pi));
                drive(max_speed, max_speed);
            }
        }
    }
    iRobot->speed_left = max_speed;
    iRobot->speed_right =max_speed;
    return 0;
}


// used to prevent out of bounds errors due to 
void reduce_angle (double *angle) {
    for(;*angle>1.0;) {
        *angle -=1.0;
    }
    for(;*angle<-1.0;) {
        *angle +=1.0;
    }
} 

// using the calibration data, we conver pixel height y into range
// range is in millimeters.
int range (location_data *iRobot, int y) {
    if ((y<0)||(y>240)) {
        return 1;
    }
    double range =762 / tan (2*pi*(80 - y/5.27)/360);
    iRobot->range_rate_2 = iRobot->range_rate;
    iRobot->range_rate = (range - iRobot->range)/iRobot->delta_t;
    iRobot->range = range;
    printf("Range: %f\n", range);
    return 0;
}

void drive (double left_wheel, double right_wheel) {
    if (left_wheel > right_wheel) {
        printf("turning right\n");
    }
    if (left_wheel < right_wheel) {
        printf("turning left\n");
    }
    
    if ((left_wheel == right_wheel)&&(left_wheel == 0)) {
        printf("stopped\n");
    }
    if ((left_wheel == right_wheel)&&(left_wheel != 0)) {
        printf("driving straight\n");
    }
}
