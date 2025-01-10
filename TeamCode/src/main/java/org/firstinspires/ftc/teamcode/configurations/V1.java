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
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",false));      // CH Motor 0
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",false));        // CH Motor 1
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",true));     // CH Motor 2
        mMotors.put("back-right-wheel",new ConfMotor("backRight",true));       // CH Motor 3

        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        mImus.put("otos", new ConfImu("sensor_otos"));                                     // EH I2C 3

        /* Intake configuration :
           -> Positive power makes slides extend
           ---> Increasing position makes arm go up
           -> Increasing position makes elbow go up
           -> Increasing position makes wrist go TBD
           -> Increasing position makes claw close */
        mMotors.put("intake-slides",new ConfMotor("intakeSlides",true));                     // EH Motor 2
        mServos.put("intake-arm-pitch", new ConfServo(
                 "intakeArmPitchLeft", false,                                                  // CH Servo 5
                        "intakeArmPitchRight", true                                                               // EH Servo 1
        ));
        mServos.put("intake-elbow-pitch", new ConfServo("intakeElbowPitch", false));         // EH Servo 0
        mServos.put("intake-wrist-roll", new ConfServo("intakeWristRoll", false));           // CH Servo 4
        mServos.put("intake-claw", new ConfServo("intakeClaw", false));                      // EH Servo 2

        mMotors.put("outtake-slides",new ConfMotor(
                "outtakeSlidesLeft",false,                                                     // EH Motor 0
                "outtakeSlidesRight",true                                                                         // EH Motor 1
        ));
        mServos.put("outtake-wrist-roll", new ConfServo("outtakeWristRoll", false));         // CH Servo 0
        mServos.put("outtake-claw", new ConfServo("outtakeClaw", false));                    // CH Servo 1
        mServos.put("outtake-elbow-pitch", new ConfServo(
                "outtakeElbowPitchLeft", false,                                                // CH Servo 2
                "outtakeElbowPitchRight", false)                                                      // CH Servo 3
        );

        /* Intake servos reference positions */
        mServos.get("intake-arm-pitch").addPosition("transfer", 0.97);
        mServos.get("intake-arm-pitch").addPosition("overSub", 0.55);
        mServos.get("intake-arm-pitch").addPosition("look", 0.44);
        mServos.get("intake-arm-pitch").addPosition("grab", 0.39);

        mServos.get("intake-elbow-pitch").addPosition("transfer", 0.15);
        mServos.get("intake-elbow-pitch").addPosition("grab", 0.68);
        mServos.get("intake-elbow-pitch").addPosition("look", 0.70);
        mServos.get("intake-elbow-pitch").addPosition("overSub", 0.73);

        mServos.get("intake-wrist-roll").addPosition("center", 0.405);
        mServos.get("intake-wrist-roll").addPosition("max", 0.82);
        mServos.get("intake-wrist-roll").addPosition("min", 0.27);

        mServos.get("intake-claw").addPosition("closed", 1.0);
        mServos.get("intake-claw").addPosition("microrelease", 0.98);
        mServos.get("intake-claw").addPosition("open", 0.62);

        mServos.get("outtake-wrist-roll").addPosition("center", 0.13);

        mServos.get("outtake-claw").addPosition("closed", 0.3);
        mServos.get("outtake-claw").addPosition("microrelease", 0.28);
        mServos.get("outtake-claw").addPosition("open", 0.1);

        mServos.get("outtake-elbow-pitch").addPosition("vertical", 0.015);
        mServos.get("outtake-elbow-pitch").addPosition("outside", 0.06);

    }

    protected void initializeTuning() {
        mSingleServos.put("intake-arm-left-pitch", new ConfServo("intakeArmPitchLeft", false));
        mSingleServos.put("intake-arm-right-pitch", new ConfServo("intakeArmPitchRight", true));
        mSingleServos.put("intake-elbow-pitch", new ConfServo("intakeElbowPitch", false));
        mSingleServos.put("intake-wrist-roll", new ConfServo("intakeWristRoll", false));
        mSingleServos.put("intake-claw", new ConfServo("intakeClaw", false));

        mSingleServos.put("outtake-wrist-roll", new ConfServo("outtakeWristRoll", false));
        mSingleServos.put("outtake-claw", new ConfServo("outtakeClaw", false));
        mSingleServos.put("outtake-elbow-left-pitch", new ConfServo("outtakeElbowPitchLeft", false));
        mSingleServos.put("outtake-elbow-right-pitch", new ConfServo("outtakeElbowPitchRight", true));
    }
}
