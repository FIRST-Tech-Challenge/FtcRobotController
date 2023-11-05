package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ArcadeDrive extends LinearOpMode {

    //Initializing motor variables
    DcMotor frontLeft;
    //Initializing frontLeft motor variable
    DcMotor backLeft;
    //Initializing frontLeft motor variable
    DcMotor frontRight;
    //Initializing frontLeft motor variable
    DcMotor backRight;
    //Initializing frontLeft motor variable

    //DO NOT CHANGE FOLLOWING LINES OF CODE

    public void runOpMode(){
        //Assigning configuration name to variable (for frontLeft, backLeft, frontRight, backRight)
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Setting Direction of Motors
        //TODO: Fix directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //Setting the direction of frontLeft to FORWARD
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //Setting the direction of backLeft to FORWARD
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //Setting the direction of frontRight to FORWARD
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //Setting the direction of backLeft to FORWARD

        //Waiting for Start button to be pressed
        waitForStart();

        //Looping while the opmode is running
        while (opModeIsActive()){
            //defining driving variables (throttle = moving)
            double throttle = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            //setting power for forward-backward movement
            frontLeft.setPower(throttle);
            //setting the power for frontLeft
            backLeft.setPower(throttle);
            //setting the power for backLeft
            frontRight.setPower(throttle);
            //setting the power for frontRight
            backRight.setPower(throttle);
            //setting the power for backRight

            //setting up strafing
            frontLeft.setPower(strafe);
            backLeft.setPower(-strafe);
            frontRight.setPower(-strafe);
            backRight.setPower(strafe);

            //setting power for turning
            /*frontLeft and backLeft are -turn while frontRight and backRight are positive b/c two
            of the motors need to be positive and two have to be negative in order to have the
            robot properly turn
            */
            frontLeft.setPower(turn);
            backLeft.setPower(turn);
            frontRight.setPower(-turn);
            backRight.setPower(-turn);
        }
    }
}
