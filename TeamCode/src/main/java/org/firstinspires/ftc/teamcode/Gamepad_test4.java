package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Simple Test", group="Linear OpMode")
public class Gamepad_test4 extends LinearOpMode {

    public DcMotor leftwheel = null;
    public DcMotor rightwheel = null;
    public Servo leftservo;
    public Servo rightservo;

    @Override
    public void runOpMode() {
        leftwheel  = hardwareMap.get(DcMotor.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
        leftservo  = hardwareMap.get(Servo.class, "leftservo");
        rightservo = hardwareMap.get(Servo.class, "rightservo");

        waitForStart();

        leftwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Move wheels forward
//        leftwheel.setPower(0.5);
//        rightwheel.setPower(0.5);
//
//         Set servos to neutral
//        leftservo.setPosition(0.5);
//        rightservo.setPosition(0.5);
//
        sleep(10000);  // Run for 2 seconds, then stop
//
//        leftwheel.setPower(0);
//        rightwheel.setPower(0);
    }
}
