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
        can_frame fr1;
        can.structure_receive_random(fr1, PUTM_CAN::NO_TIMEOUT);

        //switch case.



        //done.






    }


}