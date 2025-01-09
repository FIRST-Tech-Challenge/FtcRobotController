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

        /* Moving configuration :
           -> Positive power makes wheel go forward */
        m_motors.put("front-left-wheel",new MotorConf("frontLeft",false));      // CH Motor 0
        m_motors.put("back-left-wheel",new MotorConf("backLeft",false));        // CH Motor 1
        m_motors.put("front-right-wheel",new MotorConf("frontRight",true));     // CH Motor 2
        m_motors.put("back-right-wheel",new MotorConf("backRight",true));       // CH Motor 3

        m_imus.put("built-in", new ImuConf("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        m_imus.put("otos", new ImuConf("sensor_otos"));                                     // EH I2C 3

        /* Intake configuration :
           -> Positive power makes slides extend
           ---> Increasing position makes arm go up
           -> Increasing position makes elbow go up
           -> Increasing position makes wrist go TBD
           -> Increasing position makes claw close */
        m_motors.put("intake-slides",new MotorConf("intakeSlides",true));                     // EH Motor 2
        m_servos.put("intake-arm-left-pitch", new ServoConf("intakeArmPitchLeft", false));    // CH Servo 5
        m_servos.put("intake-arm-right-pitch", new ServoConf("intakeArmPitchRight", true));   // EH Servo 1
        m_servos.put("intake-elbow-pitch", new ServoConf("intakeElbowPitch", false));         // EH Servo 0
        m_servos.put("intake-wrist-roll", new ServoConf("intakeWristRoll", false));           // CH Servo 4
        m_servos.put("intake-claw", new ServoConf("intakeClaw", false));                      // EH Servo 2

        /* Outtake configuration :
           -> Positive power makes slides extend
           -> Increasing position makes elbow go outside
           -> Increasing position makes wrist go TBD
           -> Increasing position makes claw close */
        m_motors.put("outtake-right-slides",new MotorConf("outtakeSlidesRight",true));              // EH Motor 1
        m_motors.put("outtake-left-slides",new MotorConf("outtakeSlidesLeft",false));               // EH Motor 0
        m_servos.put("outtake-wrist-roll", new ServoConf("outtakeWristRoll", false));               // CH Servo 0
        m_servos.put("outtake-claw", new ServoConf("outtakeClaw", false));                          // CH Servo 1
        m_servos.put("outtake-elbow-left-pitch", new ServoConf("outtakeElbowPitchLeft", false));    // CH Servo 2
        m_servos.put("outtake-elbow-right-pitch", new ServoConf("outtakeElbowPitchRight", false));  // CH Servo 3

        /* Intake servos reference positions */
        m_servos.get("intake-arm-left-pitch").setPosition("transfer", 0.97);
        m_servos.get("intake-arm-left-pitch").setPosition("overSub", 0.55);
        m_servos.get("intake-arm-left-pitch").setPosition("look", 0.44);
        m_servos.get("intake-arm-left-pitch").setPosition("grab", 0.39);
        m_servos.get("intake-arm-right-pitch").setPosition("transfer", 0.97);
        m_servos.get("intake-arm-right-pitch").setPosition("overSub", 0.55);
        m_servos.get("intake-arm-right-pitch").setPosition("look", 0.44);
        m_servos.get("intake-arm-right-pitch").setPosition("grab", 0.39);

        m_servos.get("intake-elbow-pitch").setPosition("transfer", 0.15);
        m_servos.get("intake-elbow-pitch").setPosition("grab", 0.68);
        m_servos.get("intake-elbow-pitch").setPosition("look", 0.70);
        m_servos.get("intake-elbow-pitch").setPosition("overSub", 0.73);

        m_servos.get("intake-wrist-roll").setPosition("center", 0.405);
        m_servos.get("intake-wrist-roll").setPosition("max", 0.82);
        m_servos.get("intake-wrist-roll").setPosition("min", 0.27);

        m_servos.get("intake-claw").setPosition("closed", 1);
        m_servos.get("intake-claw").setPosition("microrelease", 0.98);
        m_servos.get("intake-claw").setPosition("open", 0.62);

        m_servos.get("outtake-wrist-roll").setPosition("center", 0.13);

        m_servos.get("outtake-claw").setPosition("closed", 0.3);
        m_servos.get("outtake-claw").setPosition("microrelease", 0.28);
        m_servos.get("outtake-claw").setPosition("open", 0.1);

        m_servos.get("outtake-elbow-right-pitch").setPosition("vertical", 0.015);
        m_servos.get("outtake-elbow-right-pitch").setPosition("outside", 0.06);

    }
}
