/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for NLE chassis
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Test extends HMapConfig {

    protected void initialize(){

        /* Moving configuration */
        FRONT_LEFT_WHEEL  = "frontLeft";                      // CH Motor 0
        BACK_LEFT_WHEEL   = "backLeft";                       // CH Motor 1
        BACK_RIGHT_WHEEL  = "backRight";                      // CH Motor 2
        FRONT_RIGHT_WHEEL = "frontRight";                     // CH Motor 3

        /* IMU configuration */
        BUILT_IN_IMU = "imu";                                 // CH I2C 0
        OTOS         = "sensor_otos";                         // EH I2C 3

        /* Collecting motors configuration */
        OUTTAKE_SLIDES_LEFT    = "outtakeSlidesLeft";         // EH Motor 0
        OUTTAKE_SLIDES_RIGHT   = "outtakeSlidesRight";        // EH Motor 1
        INTAKE_SLIDES          = "intakeSlides";              // EH Motor 2

        /* Collecting servos configuration */
        OUTTAKE_WRIST_ROLL        = "outputWristRoll";        // CH Servo 0
        OUTTAKE_CLAW              = "outputClaw";             // CH Servo 1
        OUTTAKE_ELBOW_PITCH_LEFT  = "outputElbowPitchLeft";   // CH Servo 2
        OUTTAKE_ELBOW_PITCH_RIGHT = "outputElbowPitchRight";  // CH Servo 3
        INTAKE_ARM_PITCH_LEFT     = "inputArmPitchLeft";      // CH Servo 4
        INTAKE_ARM_PITCH_RIGHT    = "inputArmPitchRight";     // CH Servo 5
        INTAKE_ELBOW_PITCH        = "inputElbowPitch";        // EH Servo 0
        INTAKE_WRIST_ROLL         = "inputWristRoll";         // EH Servo 1
        INTAKE_CLAW               = "inputClaw";              // EH Servo 2

        FRONT_LEFT_WHEEL_REVERSE     = false;
        BACK_LEFT_WHEEL_REVERSE      = true;
        FRONT_RIGHT_WHEEL_REVERSE    = false;
        BACK_RIGHT_WHEEL_REVERSE     = true;

        INTAKE_SLIDES_REVERSE        = false;
        OUTTAKE_SLIDES_RIGHT_REVERSE = false;
        OUTTAKE_SLIDES_LEFT_REVERSE  = false;

        BUILT_IN_IMU_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        BUILT_IN_IMU_USB  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    }
}
