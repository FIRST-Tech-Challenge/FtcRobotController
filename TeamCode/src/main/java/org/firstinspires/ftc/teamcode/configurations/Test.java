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
        m_motors.put("front-left-wheel",new MotorConf("frontLeft",false)); // CH Motor 0
        m_motors.put("back-left-wheel",new MotorConf("backLeft",true)); // CH Motor 1
        m_motors.put("back-right-wheel",new MotorConf("backRight",true)); // CH Motor 2
        m_motors.put("front-right-wheel",new MotorConf("frontRight",false)); // CH Motor 3

        m_imus.put("built-in", new ImuConf("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        m_imus.put("otos", new ImuConf("sensor_otos"));

        /* Intake configuration */
        m_servos.put("intake-arm-left-pitch", new ServoConf("left", true));
        m_servos.put("intake-arm-right-pitch", new ServoConf("right", false));

        m_servos.get("intake-arm-left-pitch").setPosition("vertical", 0.65);
        m_servos.get("intake-arm-left-pitch").setPosition("overSub", 0.23);
        m_servos.get("intake-arm-left-pitch").setPosition("look", 0.13);
        m_servos.get("intake-arm-left-pitch").setPosition("grab", 0.07);
        m_servos.get("intake-arm-right-pitch").setPosition("vertical", 0.65);
        m_servos.get("intake-arm-right-pitch").setPosition("overSub", 0.23);
        m_servos.get("intake-arm-right-pitch").setPosition("look", 0.13);
        m_servos.get("intake-arm-right-pitch").setPosition("grab", 0.07);


    }
}
