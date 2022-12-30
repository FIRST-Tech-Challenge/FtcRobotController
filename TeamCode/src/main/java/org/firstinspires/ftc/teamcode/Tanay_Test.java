package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tanay_Test", group = "TeleOp")

public class Tanay_Test extends OpMode {


    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;
    DcMotor Slide;


    //Claw Mechanism
    Servo ClawX;
    Servo ClawY;



    //Motor Power
    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;
    double ClawPower;

    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        Slide = hardwareMap.dcMotor.get("Slide");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
        ClawX = hardwareMap.servo.get("ClawX");
        ClawY = hardwareMap.servo.get("ClawY");

    }


    @Override
    public void loop() {

        //Movement Controller
        right_drivePower = gamepad1.left_stick_y;
        back_left_drivePower = gamepad1.left_stick_y;
        left_drivePower = gamepad1.right_stick_y;
        back_right_drivePower = gamepad1.right_stick_y;


        left_drive.setPower(left_drivePower);
        right_drive.setPower(right_drivePower);
        back_left_drive.setPower(left_drivePower);
        back_right_drive.setPower(right_drivePower);


        boolean rightbumper = gamepad1.right_bumper; //Strafe Right
        boolean leftbumper = gamepad1.left_bumper; //Strafe Left
        //attachments



        // Precision moves
        if (gamepad1.x) {
            left_drive.setPower(-.23);
            right_drive.setPower(-.23);
            back_left_drive.setPower(-.23);
            back_right_drive.setPower(-.23);
        }

        if (gamepad1.a) {
            left_drive.setPower(-.4);
            right_drive.setPower(-.4);
            back_left_drive.setPower(-.4);
            back_right_drive.setPower(-.4);
        }


        if (rightbumper) {

            left_drive.setPower(-1);
            right_drive.setPower(1);
            back_left_drive.setPower(1);
            back_right_drive.setPower(-1);


        } else if (leftbumper) {


            left_drive.setPower(1); // left drive is 0
            right_drive.setPower(-1); // right drive is 2
            back_left_drive.setPower(-1); // back left drive is 1
            back_right_drive.setPower(1); // back right drive is 3
        }

        if (gamepad2.right_bumper) {
            ClawY.setPosition(0.62);
        }

        else if (gamepad2.y) {
            ClawY.setPosition(1);
        }

        else if (gamepad2.x) {
            ClawY.setPosition(.85);
        }

        else {
            ClawY.setPosition(0.75);
        }

        if (gamepad2.b) {
            ClawX.setPosition(0.97);
        }
        if (gamepad2.a) {
            ClawX.setPosition(0.9);
        }

        if (gamepad2.left_bumper) { //down
            Slide.setPower(-0.08); //(-0.2)
        } else if (gamepad2.right_bumper) { //up
            Slide.setPower(0.9);
        } else {
            Slide.setPower(0.3);
        }


        telemetry.addData("ClawX Pos", ClawX.getPosition());
        telemetry.addData("ClawY Pos", ClawY.getPosition());
        telemetry.update();
    }
}