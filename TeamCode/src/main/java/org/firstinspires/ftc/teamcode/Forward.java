package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Forward", group = "TeleOp")
public class Forward extends OpMode {

    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;

    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {


        telemetry.addData("Right Wheel", right_drive.getCurrentPosition());
        telemetry.addData("Left Wheel ", left_drive.getCurrentPosition());
        telemetry.addData("Back Right Wheel", back_right_drive.getCurrentPosition());
        telemetry.addData("Back Left Wheel", back_left_drive.getCurrentPosition());
        telemetry.update();



        if (gamepad1.a) {




            left_drive.setPower(-.75);
            right_drive.setPower(-.75);
            back_left_drive.setPower(-.75);
            back_right_drive.setPower(-.75);
            telemetry.addData("Right Wheel", right_drive.getCurrentPosition());
            telemetry.addData("Left Wheel ", left_drive.getCurrentPosition());
            telemetry.addData("Back Right Wheel", back_right_drive.getCurrentPosition());
            telemetry.addData("Back Left Wheel", back_left_drive.getCurrentPosition());
            telemetry.update();

        }
        else if (gamepad1.x) {

            left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        else {
            left_drive.setPower(0);
            right_drive.setPower(0);
            back_left_drive.setPower(0);
            back_right_drive.setPower(0);
        }


        telemetry.update();

    }


}

