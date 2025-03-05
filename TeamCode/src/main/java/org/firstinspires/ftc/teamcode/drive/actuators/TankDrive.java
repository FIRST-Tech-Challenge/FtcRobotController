package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//@TeleOp
public class TankDrive extends LinearOpMode {
    DcMotor backleft;
    DcMotor backright;
    DcMotor frontleft;
    DcMotor frontright;
    @Override
    public void runOpMode() throws InterruptedException{
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        double drive;
        double turn;
        double rightSidePower, leftSidePower;

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            leftSidePower = Range.clip(drive + turn, -1, 1);
            rightSidePower = Range.clip(drive - turn, -1, 1);

            frontleft.setPower(leftSidePower);
            backleft.setPower(leftSidePower);
            backright.setPower(rightSidePower);
            frontright.setPower(rightSidePower);

        }
    }
}