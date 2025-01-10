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
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",false));   // CH Motor 0
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",true));      // CH Motor 1
        mMotors.put("back-right-wheel",new ConfMotor("backRight",true));    // CH Motor 2
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",false)); // CH Motor 3

        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        mImus.put("otos", new ConfImu("sensor_otos"));

        /* Intake configuration */
        mServos.put("intake-arm-pitch", new ConfServo(
            "left", true,                 // CH Servo 0
                "right", false));                // CH Servo 1

        mServos.get("intake-arm-pitch").addPosition("transfer", 0.65);
        mServos.get("intake-arm-pitch").addPosition("overSub", 0.23);
        mServos.get("intake-arm-pitch").addPosition("look", 0.13);
        mServos.get("intake-arm-pitch").addPosition("grab", 0.07);

    }

    protected void initializeTuning() {

        mSingleServos.put("intake-arm-left-pitch", new ConfServo("left", true));
        mSingleServos.put("intake-arm-right-pitch", new ConfServo("right", false));

    }
}
