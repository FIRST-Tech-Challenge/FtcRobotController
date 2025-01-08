/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the robot first version (chassis only)
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class V0 extends HMapConfig {

    protected void initialize() {

        /* Moving configuration */
        FRONT_LEFT_WHEEL  = "frontLeft";                      // CH Motor 0
        BACK_LEFT_WHEEL   = "backLeft";                       // CH Motor 1
        FRONT_RIGHT_WHEEL = "frontRight";                     // CH Motor 2
        BACK_RIGHT_WHEEL  = "backRight";                      // CH Motor 3

        /* IMU configuration */
        BUILT_IN_IMU = "imu";                                 // CH I2C 0
        OTOS         = "sensor_otos";                         // CH I2C 3

        FRONT_LEFT_WHEEL_REVERSE     = true;
        BACK_LEFT_WHEEL_REVERSE      = true;
        FRONT_RIGHT_WHEEL_REVERSE    = false;
        BACK_RIGHT_WHEEL_REVERSE     = true;

        BUILT_IN_IMU_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        BUILT_IN_IMU_USB  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    }
}