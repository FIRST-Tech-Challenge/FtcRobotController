/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the robot second version (18th of january)
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Test extends Configuration {

    protected void initialize(){

        /* Moving configuration */
        m_motors.put("front-left-wheel",new Motor("frontLeft",false)); // CH Motor 0
        m_motors.put("back-left-wheel",new Motor("backLeft",true)); // CH Motor 1
        m_motors.put("back-right-wheel",new Motor("backRight",true)); // CH Motor 2
        m_motors.put("front-right-wheel",new Motor("frontRight",false)); // CH Motor 3

        m_imus.put("built-in", new Imu("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        m_imus.put("otos", new Imu("sensor_otos"));

        /* Intake configuration */
        m_motors.put("intake-slides",new Motor("intakeSlides",true));
        m_servos.put("intake-arm-left-pitch", new ServoMotor("inputArmPitchLeft"));
        m_servos.put("intake-arm-right-pitch", new ServoMotor("inputArmPitchRight"));
        m_servos.put("intake-elbow-pitch", new ServoMotor("inputElbowPitch"));
        m_servos.put("intake-wrist-roll", new ServoMotor("inputWristRoll"));
        m_servos.put("intake-claw", new ServoMotor("inputClaw"));

        /* Outtake configuration */
        m_motors.put("outtake-right-slides",new Motor("outtakeSlidesRight",true));
        m_motors.put("outtake-left-slides",new Motor("outtakeSlidesLeft",false));
        m_servos.put("outtake-elbow-left-pitch", new ServoMotor("outputElbowPitchLeft"));
        m_servos.put("outtake-elbow-right-pitch", new ServoMotor("outputElbowPitchRight"));
        m_servos.put("outtake-wrist-roll", new ServoMotor("outputWristRoll"));
        m_servos.put("outtake-claw", new ServoMotor("outputClaw"));

    }
}
