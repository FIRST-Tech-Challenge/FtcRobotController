package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "OffSeason_Teleop")
public class TeleOp extends LinearOpMode {

    DcMotorEx leftDrive;
    DcMotorEx rightDrive;
    DcMotorEx armMotor;

    Servo servo1, servo2;


    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        armMotor = hardwareMap.get(DcMotorEx.class, "Arm");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

            leftDrive.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            rightDrive.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);

            

        }




        }
    }
