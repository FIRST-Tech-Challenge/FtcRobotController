package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotClass;

public class TeachingClass extends LinearOpMode {

    //Initializing motor variables
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    //DO NOT CHANGE FOLLOWING LINES OF CODE

    public void runOpMode(){
        //Assigning configuration name to variable
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Setting Direction of Motors
        //TODO: Fix directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Waiting for Start button to be pressed
        waitForStart();

        //Looping while the opmode is running
        while (opModeIsActive()){
            //defining driving variables
            double throttle = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            //setting power for forward-backward movement
            frontLeft.setPower(throttle);
            backLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backRight.setPower(throttle);

            //setting power for turning
            frontLeft.setPower(-turn);
            backLeft.setPower(-turn);
            frontRight.setPower(turn);
            backRight.setPower(turn);
        }
    }
}
