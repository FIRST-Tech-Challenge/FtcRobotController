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
    public void runOpMode() throws InterruptedException {
        final Servo rightWristServo, specServo;
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        specServo = hardwareMap.get(Servo.class, "specServo");
        waitForStart();

        while(opModeIsActive()){
            rightWristServo.setPosition(WRIST_SERVO_POSITION);
            specServo.setPosition(SPEC_SERVO_POSITION);
        }
    }

}
