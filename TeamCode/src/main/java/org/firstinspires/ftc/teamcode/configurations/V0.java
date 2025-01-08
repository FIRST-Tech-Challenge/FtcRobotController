/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the robot first version (chassis only)
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class V0 extends Configuration {

    protected void initialize() {

        /* Moving configuration */
        m_motors.put("front-left-wheel",new Motor("frontLeft",true));  // CH Motor 0
        m_motors.put("back-left-wheel",new Motor("backLeft",true)); // CH Motor 1
        m_motors.put("front-right-wheel",new Motor("frontRight",false)); // CH Motor 2
        m_motors.put("back-right-wheel",new Motor("backRight",true)); // CH Motor 3

        m_imus.put("built-in", new Imu("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        m_imus.put("otos", new Imu("sensor_otos"));
    }
}