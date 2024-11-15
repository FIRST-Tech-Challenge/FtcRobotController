package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTester extends LinearOpMode {
    public static double WRIST_SERVO_POSITION = 0;
    public static double SPEC_SERVO_POSITION = 0;
    public static double INTAKE_CLAW_POSITION = 0;
    public void runOpMode() throws InterruptedException {
        final Servo rightWristServo, specServo, clawIntake;
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        specServo = hardwareMap.get(Servo.class, "specServo");
        clawIntake = hardwareMap.get(Servo.class, "clawIntake");
        waitForStart();

        while(opModeIsActive()){
            rightWristServo.setPosition(WRIST_SERVO_POSITION);
            specServo.setPosition(SPEC_SERVO_POSITION);
            clawIntake.setPosition(INTAKE_CLAW_POSITION);
        }
    }

}
