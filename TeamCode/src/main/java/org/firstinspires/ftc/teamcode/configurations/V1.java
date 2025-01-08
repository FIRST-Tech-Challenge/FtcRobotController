/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the robot second version (18th of january)
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class V1 extends Configuration {

    protected void initialize(){

        /* Moving configuration */
        m_motors.put("front-left-wheel",new Motor("frontLeft",false));      // CH Motor 0
        m_motors.put("back-left-wheel",new Motor("backLeft",false));        // CH Motor 1
        m_motors.put("front-right-wheel",new Motor("frontRight",true));     // CH Motor 2
        m_motors.put("back-right-wheel",new Motor("backRight",true));       // CH Motor 3

        m_imus.put("built-in", new Imu("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        m_imus.put("otos", new Imu("sensor_otos"));                                     // EH I2C 3

        /* Intake configuration */
        m_motors.put("intake-slides",new Motor("intakeSlides",true));        // EH Motor 2
        m_servos.put("intake-arm-left-pitch", new ServoMotor("inputArmPitchLeft"));     // CH Servo 4
        m_servos.put("intake-arm-right-pitch", new ServoMotor("inputArmPitchRight"));   // CH Servo 5
        m_servos.put("intake-elbow-pitch", new ServoMotor("inputElbowPitch"));          // EH Servo 0
        m_servos.put("intake-wrist-roll", new ServoMotor("inputWristRoll"));            // EH Servo 1
        m_servos.put("intake-claw", new ServoMotor("inputClaw"));                       // EH Servo 2

        /* Outtake configuration */
        m_motors.put("outtake-right-slides",new Motor("outtakeSlidesRight",true)); // EH Motor 1
        m_motors.put("outtake-left-slides",new Motor("outtakeSlidesLeft",false));  // EH Motor 0
        m_servos.put("outtake-elbow-left-pitch", new ServoMotor("outputElbowPitchLeft"));     // CH Servo 2
        m_servos.put("outtake-elbow-right-pitch", new ServoMotor("outputElbowPitchRight"));   // CH Servo 3
        m_servos.put("outtake-wrist-roll", new ServoMotor("outputWristRoll"));                // CH Servo 0
        m_servos.put("outtake-claw", new ServoMotor("outputClaw"));                           // CH Servo 1

    }
}
