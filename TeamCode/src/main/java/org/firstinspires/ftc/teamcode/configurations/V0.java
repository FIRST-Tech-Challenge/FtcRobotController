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
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",true));    // CH Motor 0
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",true));      // CH Motor 1
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",false)); // CH Motor 2
        mMotors.put("back-right-wheel",new ConfMotor("backRight",true));    // CH Motor 3

        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        mImus.put("otos", new ConfImu("sensor_otos"));

    }

    protected void initializeTuning() {

    }
}