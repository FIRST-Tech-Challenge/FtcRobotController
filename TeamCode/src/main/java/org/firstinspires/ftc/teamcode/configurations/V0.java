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
        m_motors.put("front-left-wheel",new MotorConf("frontLeft",true));  // CH Motor 0
        m_motors.put("back-left-wheel",new MotorConf("backLeft",true)); // CH Motor 1
        m_motors.put("front-right-wheel",new MotorConf("frontRight",false)); // CH Motor 2
        m_motors.put("back-right-wheel",new MotorConf("backRight",true)); // CH Motor 3

        m_imus.put("built-in", new ImuConf("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        m_imus.put("otos", new ImuConf("sensor_otos"));
    }
}