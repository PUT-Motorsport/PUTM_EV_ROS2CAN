#include "../PUTM_DV_CAN_LIBRARY/Inc/putm_can_interface.hpp"
#include "ros/ros.h"
#include "PUTM_EV_ROS2CAN/APPS1.h"
#include "PUTM_EV_ROS2CAN/AQ_CARD.h"
#include "PUTM_EV_ROS2CAN/BMSHV.h"
#include "PUTM_EV_ROS2CAN/BMSLV.h"
#include "PUTM_EV_ROS2CAN/TC.h"

int main(int argc, char *argv[])
{  
    ros::init(argc, argv, "Telemetry");

    ros::NodeHandle nh;

    ros::Publisher APPS_Publisher       = nh.advertise<PUTM_EV_ROS2CAN::APPS1>   ("Apps_Data", 5);
    ros::Publisher AQ_Card_Publisher    = nh.advertise<PUTM_EV_ROS2CAN::AQ_CARD> ("AQ_Card_Data", 5);
    ros::Publisher BMSHV_Publisher      = nh.advertise<PUTM_EV_ROS2CAN::BMSHV>   ("BmsHv_Data", 5);
    ros::Publisher BMSLV_Publisher      = nh.advertise<PUTM_EV_ROS2CAN::BMSLV>   ("BmsLv_Data", 5);
    ros::Publisher TC_Publisher         = nh.advertise<PUTM_EV_ROS2CAN::TC>      ("Tc_Data", 5);

    PUTM_CAN::CAN can;

    if(can.connect() != PUTM_CAN::CanState::CAN_OK)
    {
        ROS_ERROR("Error while connecting to CAN socket");
        return 0;
    }       
    else
    {
        ROS_INFO("Connected to CAN socket");
    }

    //main loop

    for(;;)
    {
        can_frame frtmp;
        can.structure_receive_random(frtmp, PUTM_CAN::NO_TIMEOUT);

        //switch case.
        switch(frtmp.can_id)
        {
            case PUTM_CAN::APPS_MAIN_CAN_ID:
            {
                PUTM_CAN::Apps_main appstmp;
                memcpy(&appstmp, &frtmp.data, sizeof(frtmp.data));

                PUTM_EV_ROS2CAN::APPS1 apps;

                apps.pedalPosition  = appstmp.pedal_position;
                apps.counter        = appstmp.counter;
                apps.difference     = appstmp.position_diff;

                APPS_Publisher.publish(apps);
            }
            break;

            // case PUTM_CAN::AQ_ACCELERATION_CAN_ID:
            // {
            //     PUTM_CAN::AQ_acceleration aqacctmp;
            //     memcpy(&aqacctmp, &frtmp.data, sizeof(frtmp.data));

            // }
            // break;

            // case PUTM_CAN::AQ_GYROSCOPE_CAN_ID:
            // {
            //     PUTM_CAN::AQ_gyroscope aqgytmp;
            //     memcpy(&aqgytmp, &frtmp.data, sizeof(frtmp.data));

            // }
            // break;

            // case PUTM_CAN::AQ_MAIN_CAN_ID:
            // {
            //     PUTM_CAN::AQ_main aqtmp;
            //     memcpy(&aqtmp, &frtmp.data, sizeof(frtmp.data));

            // }
            // break;

            // case PUTM_CAN::AQ_TS_BUTTON_CAN_ID:
            // {
            //     PUTM_CAN::AQ_main aqtmp;
            //     memcpy(&aqtmp, &frtmp.data, sizeof(frtmp.data));

            // }
            // break;

            case PUTM_CAN::BMS_LV_MAIN_CAN_ID:
            {
                PUTM_CAN::BMS_LV_main bmslv;
                memcpy(&bmslv, &frtmp.data, sizeof(frtmp.data));


                PUTM_EV_ROS2CAN::BMSLV bmslvros;

                bmslvros.averageTemperature    = bmslv.temp_avg;
                bmslvros.current               = bmslv.current;
                bmslvros.lvVoltage             = bmslv.voltage_sum;
                bmslvros.soc                   = bmslv.soc;

                BMSLV_Publisher.publish(bmslvros);

            }
            break;

            case PUTM_CAN::BMS_LV_TEMPERATURE_CAN_ID:
            {
                PUTM_CAN::BMS_LV_temperature bmslvtemps;
                memcpy(&bmslvtemps, &frtmp.data, sizeof(frtmp.data));

            }
            break;

            case PUTM_CAN::BMS_HV_MAIN_CAN_ID:
            {
                PUTM_CAN::BMS_HV_main bmshvmain;
                memcpy(&bmshvmain, &frtmp.data, sizeof(frtmp.data));

                PUTM_EV_ROS2CAN::BMSHV bmshvros;

                bmshvros.hvVoltage          = bmshvmain.voltage_sum;
                bmshvros.current            = bmshvmain.current;
                bmshvros.maxTemperature     = bmshvmain.temp_max;
                bmshvros.soc                = bmshvmain.soc;
                bmshvros.averageTemperature = bmshvmain.temp_avg;

                BMSHV_Publisher.publish(bmshvros);



            }
            break;

            case PUTM_CAN::DASH_MAIN_CAN_ID:
            {

            }
            break;

            case PUTM_CAN::DV_ASS_CAN_ID:
            {

            }
            break;

            case PUTM_CAN::LAP_TIMER_ACC_TIME_CAN_ID:
            {
                PUTM_CAN::Lap_timer_Acc_time laptimeracc;
                memcpy(&laptimeracc, &frtmp.data, sizeof(frtmp.data));

            }
            break;

            case PUTM_CAN::LAP_TIMER_LAP_TIME_CAN_ID:
            {
                PUTM_CAN::Lap_timer_Lap_time laptimerlap;
                memcpy(&laptimerlap, &frtmp.data, sizeof(frtmp.data));

            }
            break;

            case PUTM_CAN::LAP_TIMER_MAIN_CAN_ID:
            {
                PUTM_CAN::Lap_timer_Main laptimermain;
                memcpy(&laptimermain, &frtmp.data, sizeof(frtmp.data));

            }
            break;

            case PUTM_CAN::LAP_TIMER_SECTOR_CAN_ID:
            {
                PUTM_CAN::Lap_timer_Lap_time laptimersec;
                memcpy(&laptimersec, &frtmp.data, sizeof(frtmp.data));

            }
            break;

            case PUTM_CAN::LAP_TIMER_SKIDPAD_TIME_CAN_ID:
            {
                PUTM_CAN::Lap_timer_Lap_time laptimerskid;
                memcpy(&laptimerskid, &frtmp.data, sizeof(frtmp.data));

            }
            break;

            case PUTM_CAN::SF_MAIN_CAN_ID:
            {
                PUTM_CAN::SF_main sfmain;
                memcpy(&sfmain, &frtmp.data, sizeof(frtmp.data));

            }
            break;

            case PUTM_CAN::SF_SAFETY_CAN_ID:
            {
                PUTM_CAN::SF_safety sfsafety;
                memcpy(&sfsafety, &frtmp.data, sizeof(frtmp.data));

            }
            break;

            case PUTM_CAN::TC_IMU_ACC_CAN_ID:
            {
                PUTM_CAN::TC_imu_acc tc_imu_acc;
                memcpy(&tc_imu_acc, &frtmp.data, sizeof(frtmp.data));

                PUTM_EV_ROS2CAN::TC tcros;

                tcros.accX  = tc_imu_acc.acc_x;
                tcros.accY  = tc_imu_acc.acc_y;
                tcros.accZ  = tc_imu_acc.acc_z;

                TC_Publisher.publish(tcros);
            }
            break;

            case PUTM_CAN::TC_IMU_GYRO_CAN_ID:
            {
                PUTM_CAN::TC_imu_gyro tc_imu_gyro;
                memcpy(&tc_imu_gyro, &frtmp.data, sizeof(frtmp.data));

                PUTM_EV_ROS2CAN::TC tcros;

                tcros.gyroX = tc_imu_gyro.gyro_x;
                tcros.gyroY = tc_imu_gyro.gyro_y;
                tcros.gyroZ = tc_imu_gyro.gyro_z;

                TC_Publisher.publish(tcros);
            }
            break;


            case PUTM_CAN::TC_MAIN_CAN_ID:
            {
                PUTM_CAN::TC_main tc_main;
                memcpy(&tc_main, &frtmp.data, sizeof(frtmp.data));

                PUTM_EV_ROS2CAN::TC tcros;

                tcros.vehicleSpeed  = tc_main.vehicle_speed;
                tcros.motorSpeed    = tc_main.engine_speed;;
                tcros.motorCurrent  = tc_main.motor_current;

                TC_Publisher.publish(tcros);

            }
            break;


            case PUTM_CAN::TC_REAR_SUSPENSION_CAN_ID:
            {
                PUTM_CAN::TC_rear_suspension tc_rear;
                memcpy(&tc_rear, &frtmp.data, sizeof(frtmp.data));

                PUTM_EV_ROS2CAN::TC tcros;

                tcros.suspensionRightRear  = tc_rear.adc_susp_right;
                tcros.suspensionLeftRear   = tc_rear.adc_susp_left;

                TC_Publisher.publish(tcros);

            }
            break;


            case PUTM_CAN::TC_TEMPERATURES_CAN_ID:
            {
                PUTM_CAN::TC_temperatures tc_temperatures;
                memcpy(&tc_temperatures, &frtmp.data, sizeof(frtmp.data));

            }
            break;


            case PUTM_CAN::TC_WHEEL_VELOCITIES_CAN_ID:
            {
                PUTM_CAN::TC_wheel_velocities tc_wheels;
                memcpy(&tc_wheels, &frtmp.data, sizeof(frtmp.data));

                PUTM_EV_ROS2CAN::TC tcros;

                tcros.wheelSpeedLeftfront   = tc_wheels.left_front; 
                tcros.wheelSpeedRightfront  = tc_wheels.right_front;
                tcros.wheelSpeedLeftRear    = tc_wheels.left_rear; 
                tcros.wheelSpeedRightRear   = tc_wheels.right_rear; 

                TC_Publisher.publish(tcros);



            }
            break;


            case PUTM_CAN::WHEELTEMP_MAIN_CAN_ID:
            {
                PUTM_CAN::WheelTemp_main wheel_tmp_main;
                memcpy(&wheel_tmp_main, &frtmp.data, sizeof(frtmp.data));

            }
            break;


            case PUTM_CAN::YAWPROBE_AIR_FLOW_CAN_ID:
            {
                PUTM_CAN::YawProbe_air_flow yawprobe;
                memcpy(&yawprobe, &frtmp.data, sizeof(frtmp.data));

            }
            break;
        }
            //done.
    }
}   