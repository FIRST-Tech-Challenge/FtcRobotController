package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class TeleOp extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        //to do: improve motor initialization. It's probably wrong
        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("Init: ", "Ready for start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            //temporary code
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            double maxPower = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);

            //do stuff

            //to do: verify this works
            if(turn >= 0){ //forward and backward with turning || positive turn = turn right
                frontRight.setPower((drive - turn) / maxPower);
                backRight.setPower((drive - turn) / maxPower);
            } else {
                frontLeft.setPower((drive - turn) / maxPower);
                backLeft.setPower((drive - turn) / maxPower);
            }

            //cycle every 10 milliseconds, to prevent memory death
            sleep(10);
        }
    }
}
