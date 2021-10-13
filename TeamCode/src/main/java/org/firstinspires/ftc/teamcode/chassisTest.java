package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class chassisTest extends LinearOpMode {

    //Motor initalization
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            //defining driving variables.
            double fwd_throttle;
            double bk_throttle;
            double turn;

            fwd_throttle = gamepad1.right_trigger;
            bk_throttle = -gamepad1.left_trigger;
            turn = gamepad1.right_stick_x;

            //making motors run.
            frontLeft.setPower(fwd_throttle);
            frontRight.setPower(fwd_throttle);
            backLeft.setPower(fwd_throttle);
            backRight.setPower(fwd_throttle);

            frontLeft.setPower(bk_throttle);
            frontRight.setPower(bk_throttle);
            backLeft.setPower(bk_throttle);
            backRight.setPower(bk_throttle);

            frontLeft.setPower(-turn);
            frontRight.setPower(turn);
            backLeft.setPower(-turn);
            backRight.setPower(turn);
        }
    }
}
