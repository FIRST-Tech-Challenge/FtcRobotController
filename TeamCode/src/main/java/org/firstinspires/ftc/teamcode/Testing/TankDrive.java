package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class TankDrive extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //TODO: Fix directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //Setting the direction of frontLeft to FORWARD
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //Setting the direction of backLeft to FORWARD
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //Setting the direction of frontRight to FORWARD
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //Setting the direction of backLeft to FORWARD

        waitForStart();

        while (opModeIsActive()){
            double throttle = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            //setting power for forward-backward movement
            frontLeft.setPower(throttle);
            //setting the power for frontLeft
            backLeft.setPower(throttle);
            //setting the power for backLeft
            frontRight.setPower(throttle);
            //setting the power for frontRight
            backRight.setPower(throttle);
            //setting the power for backRight

            frontLeft.setPower(turn);
            backLeft.setPower(turn);
            frontRight.setPower(-turn);
            backRight.setPower(-turn);
        }
    }

}
