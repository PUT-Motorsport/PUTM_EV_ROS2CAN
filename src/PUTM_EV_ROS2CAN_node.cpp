#include "../PUTM_DV_CAN_LIBRARY_RAII/include/can_rx.hpp"
#include "ros/ros.h"
#include "PUTM_EV_ROS2CAN/Apps.h"
#include "PUTM_EV_ROS2CAN/AqCard.h"
#include "PUTM_EV_ROS2CAN/BmsHv.h"
#include "PUTM_EV_ROS2CAN/BmsLv.h"
#include "PUTM_EV_ROS2CAN/TractionControl.h"
#include "PUTM_EV_ROS2CAN/Odrive.h"
#include "PUTM_EV_ROS2CAN/Yawprobe.h"
#include "PUTM_EV_ROS2CAN/AqTsButton.h"

int main(int argc, char *argv[])
{  
    ros::init(argc, argv, "ROS2CAN");

    ros::NodeHandle nh;

    ros::Publisher AppsPublisher       = nh.advertise<PUTM_EV_ROS2CAN::Apps>            ("Apps_data", 5);
    ros::Publisher AqCardPublisher     = nh.advertise<PUTM_EV_ROS2CAN::AqCard>          ("Aq_card_data", 5);
    ros::Publisher BmshvPublisher      = nh.advertise<PUTM_EV_ROS2CAN::BmsHv>           ("BMS_HV_Data", 5);
    ros::Publisher BmslvPublisher      = nh.advertise<PUTM_EV_ROS2CAN::BmsLv>           ("BMS_LV_Data", 5);
    ros::Publisher TcPublisher         = nh.advertise<PUTM_EV_ROS2CAN::TractionControl> ("Tracion_Control_Data", 5);
    ros::Publisher OdrivePublisher     = nh.advertise<PUTM_EV_ROS2CAN::Odrive>          ("OdriveDataCAN", 5);
    ros::Publisher AqTsButtonCardPublisher = nh.advertise<PUTM_EV_ROS2CAN::AqTsButton>  ("AqTsButton", 5);

    PUTM_EV_ROS2CAN::Odrive odrive;
    PUTM_EV_ROS2CAN::TractionControl tcros;
    PUTM_EV_ROS2CAN::BmsHv bmshvros;
    PUTM_EV_ROS2CAN::Apps apps;
    PUTM_EV_ROS2CAN::BmsLv bmslvros;

    //CanRX goes out of scope
    PUTM_CAN::CanRx can_rx("slcan0", PUTM_CAN::NO_TIMEOUT);
    can_frame frame = can_rx.receive();

    //main loop

    for(;;)
    {
        //switch case.
        can_frame random_device_data = can_rx.receive();
        switch(random_device_data.can_id)
        {
            case PUTM_CAN::APPS_MAIN_CAN_ID:
            {
                PUTM_CAN::Apps_main appstmp;
                memcpy(&appstmp, &random_device_data.data, sizeof(random_device_data.data));
                apps.pedalPosition  = appstmp.pedal_position;
                apps.counter        = appstmp.counter;
                apps.difference     = appstmp.position_diff;

                AppsPublisher.publish(apps);
            }
            break;

            case PUTM_CAN::AQ_TS_BUTTON_CAN_ID:
            {
                PUTM_CAN::AQ_ts_button aqTs;
                memcpy(&aqTs, &random_device_data.data, sizeof(random_device_data.data));

                PUTM_EV_ROS2CAN::AqTsButton aqcard;

                aqcard.AqTsbutton = aqTs.placeholder;
                AqTsButtonCardPublisher.publish(aqcard);
            }
            break;

            case PUTM_CAN::BMS_LV_MAIN_CAN_ID:
            {
                PUTM_CAN::BMS_LV_main bmslv;
                memcpy(&bmslv, &random_device_data.data, sizeof(random_device_data.data));
                bmslvros.averageTemperature    = bmslv.temp_avg;
                bmslvros.current               = bmslv.current;
                bmslvros.lvVoltage             = bmslv.voltage_sum;
                bmslvros.soc                   = bmslv.soc;

                BmslvPublisher.publish(bmslvros);

            }
            break;

            case PUTM_CAN::BMS_LV_TEMPERATURE_CAN_ID:
            {
                PUTM_CAN::BMS_LV_temperature bmslvtemps;
                memcpy(&bmslvtemps, &random_device_data.data, sizeof(random_device_data.data));

            }
            break;

            case PUTM_CAN::BMS_HV_MAIN_CAN_ID:
            {
                PUTM_CAN::BMS_HV_main bmshvmain;
                memcpy(&bmshvmain, &random_device_data.data, sizeof(random_device_data.data));
                bmshvros.hvVoltage          = bmshvmain.voltage_sum;
                bmshvros.current            = bmshvmain.current;
                bmshvros.maxTemperature     = bmshvmain.temp_max;
                bmshvros.soc                = bmshvmain.soc;
                bmshvros.averageTemperature = bmshvmain.temp_avg;

                BmshvPublisher.publish(bmshvros);
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
                memcpy(&laptimeracc, &random_device_data.data, sizeof(random_device_data.data));

            }
            break;

            case PUTM_CAN::LAP_TIMER_LAP_TIME_CAN_ID:
            {
                PUTM_CAN::Lap_timer_Lap_time laptimerlap;
                memcpy(&laptimerlap, &random_device_data.data, sizeof(random_device_data.data));

            }
            break;

            case PUTM_CAN::LAP_TIMER_MAIN_CAN_ID:
            {
                PUTM_CAN::Lap_timer_Main laptimermain;
                memcpy(&laptimermain, &random_device_data.data, sizeof(random_device_data.data));

            }
            break;

            case PUTM_CAN::LAP_TIMER_SECTOR_CAN_ID:
            {
                PUTM_CAN::Lap_timer_Lap_time laptimersec;
                memcpy(&laptimersec, &random_device_data.data, sizeof(random_device_data.data));

            }
            break;

            case PUTM_CAN::LAP_TIMER_SKIDPAD_TIME_CAN_ID:
            {
                PUTM_CAN::Lap_timer_Lap_time laptimerskid;
                memcpy(&laptimerskid, &random_device_data.data, sizeof(random_device_data.data));

            }
            break;

            case PUTM_CAN::SF_MAIN_CAN_ID:
            {
                PUTM_CAN::SF_main sfmain;
                memcpy(&sfmain, &random_device_data.data, sizeof(random_device_data.data));

            }
            break;

            case PUTM_CAN::SF_SAFETY_CAN_ID:
            {
                PUTM_CAN::SF_safety sfsafety;
                memcpy(&sfsafety, &random_device_data.data, sizeof(random_device_data.data));

            }
            break;

            case PUTM_CAN::TC_IMU_ACC_CAN_ID:
            {
                PUTM_CAN::TC_imu_acc tc_imu_acc;
                memcpy(&tc_imu_acc, &random_device_data.data, sizeof(random_device_data.data));
                tcros.accX  = tc_imu_acc.acc_x;
                tcros.accY  = tc_imu_acc.acc_y;
                tcros.accZ  = tc_imu_acc.acc_z;

                TcPublisher.publish(tcros);
            }
            break;

            case PUTM_CAN::TC_IMU_GYRO_CAN_ID:
            {
                PUTM_CAN::TC_imu_gyro tc_imu_gyro;
                memcpy(&tc_imu_gyro, &random_device_data.data, sizeof(random_device_data.data));
                tcros.gyroX = tc_imu_gyro.gyro_x;
                tcros.gyroY = tc_imu_gyro.gyro_y;
                tcros.gyroZ = tc_imu_gyro.gyro_z;

                TcPublisher.publish(tcros);
            }
            break;


            case PUTM_CAN::TC_MAIN_CAN_ID:
            {
                PUTM_CAN::TC_main tc_main;
                memcpy(&tc_main, &random_device_data.data, sizeof(random_device_data.data));
                tcros.vehicleSpeed  = tc_main.vehicle_speed;
                tcros.motorSpeed    = tc_main.engine_speed;;
                tcros.motorCurrent  = tc_main.motor_current;

                TcPublisher.publish(tcros);

            }
            break;


            case PUTM_CAN::TC_REAR_SUSPENSION_CAN_ID:
            {
                PUTM_CAN::TC_rear_suspension tc_rear;
                memcpy(&tc_rear, &random_device_data.data, sizeof(random_device_data.data));
                tcros.suspensionRightRear  = tc_rear.adc_susp_right;
                tcros.suspensionLeftRear   = tc_rear.adc_susp_left;

                TcPublisher.publish(tcros);

            }
            break;


            case PUTM_CAN::TC_TEMPERATURES_CAN_ID:
            {
                PUTM_CAN::TC_temperatures tc_temperatures;
                memcpy(&tc_temperatures, &random_device_data.data, sizeof(random_device_data.data));

            }
            break;


            case PUTM_CAN::TC_WHEEL_VELOCITIES_CAN_ID:
            {
                PUTM_CAN::TC_wheel_velocities tc_wheels;
                memcpy(&tc_wheels, &random_device_data.data, sizeof(random_device_data.data));
                tcros.wheelSpeedLeftfront   = tc_wheels.left_front; 
                tcros.wheelSpeedRightfront  = tc_wheels.right_front;
                tcros.wheelSpeedLeftRear    = tc_wheels.left_rear; 
                tcros.wheelSpeedRightRear   = tc_wheels.right_rear; 

                TcPublisher.publish(tcros);
            }
            break;


            case PUTM_CAN::WHEELTEMP_MAIN_CAN_ID:
            {
                PUTM_CAN::WheelTemp_main wheel_tmp_main;
                memcpy(&wheel_tmp_main, &random_device_data.data, sizeof(random_device_data.data));
            }
            break;

            case PUTM_CAN::YAWPROBE_AIR_FLOW_CAN_ID:
            {
                PUTM_CAN::YawProbe_air_flow yawprobe;
                memcpy(&yawprobe, &random_device_data.data, sizeof(random_device_data.data));
            }
            break;

            case PUTM_CAN::ODRIVE_HEARTBEAT_CAN_ID:
            {
                PUTM_CAN::Odrive_Heartbeat odrivehbeat;
                memcpy(&odrivehbeat, &random_device_data.data, sizeof(random_device_data.data));

                odrive.OdriveAxisError = odrivehbeat.Axis_Error;
                odrive.OdriveAxisState = odrivehbeat.Axis_State;

                //ROS_INFO("Odrive state: %i, error: %i", odrivehbeat.Axis_State, odrivehbeat.Axis_Error);

                OdrivePublisher.publish(odrive);
            }
            break;

            case PUTM_CAN::ODRIVE_GET_ENCODER_ESTIMATION_CAN_ID:
            {
                PUTM_CAN::Odrive_Get_Encoder_Estimation enc;
                memcpy(&enc, &random_device_data.data, sizeof(random_device_data.data));
            
                odrive.EncoderEstimation = enc.Pos_Estimate;
                OdrivePublisher.publish(odrive);
            }
            break;

            case PUTM_CAN::ODRIVE_GET_MOTOR_ERROR_CAN_ID:
            {
                PUTM_CAN::Odrive_Get_Motor_Error merror;
                memcpy(&merror, &random_device_data.data, sizeof(random_device_data.data));
                ROS_INFO("ODrive motor error");
                odrive.MotorError = merror.Active_Errors;
                odrive.MotorDisarmReason = merror.Disarm_Reason;

                OdrivePublisher.publish(odrive);
            }
            break;

            case PUTM_CAN::ODRIVE_GET_ENCODER_ERROR_CAN_ID:
            {
                PUTM_CAN::Odrive_Get_Motor_Error eerror;
                memcpy(&eerror, &random_device_data.data, sizeof(random_device_data.data));
                ROS_INFO("ODrive encoder error");
                odrive.EncoderError = eerror.Active_Errors;
                odrive.EncoderDisarmReason = eerror.Disarm_Reason;

                OdrivePublisher.publish(odrive);
            }
            break;

            case PUTM_CAN::ODRIVE_GET_IQ_CAN_ID:
            {
                PUTM_CAN::Odrive_Get_Iq iq;
                memcpy(&iq, &random_device_data.data, sizeof(random_device_data.data));
                odrive.IqCurrent = iq.Iq_Measured;
                OdrivePublisher.publish(odrive);
            }
        }
    }
} 