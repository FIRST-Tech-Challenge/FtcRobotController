package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// note: to change the tests of the code, go to line 53 and change the values
// note: around line 55 I set the front and back motors to the same target. this might be consequential in the future

@Autonomous(name = "Autonomous Mode (Djokovic will win a record breaking 7th ATP Final Title)")
public class autonomousphase1 extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private int leftFront_Pos;
    private int leftBack_Pos;
    private int rightFront_Pos;
    private int rightBack_Pos;


    @Override
    public void runOpMode() {
        // !!!! note: if we switch to opmode (not linearopmode), then the initialized code must be in a different method than the running code


        // setting motors
        leftFront = hardwareMap.get(DcMotor.class, "motorLeftFront");
        leftBack = hardwareMap.get(DcMotor.class, "motorLeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "motorRightFront");
        rightBack = hardwareMap.get(DcMotor.class, "motorRightBack");

        // reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront_Pos = 0;
        leftBack_Pos = 0;
        rightFront_Pos = 0;
        rightBack_Pos = 0;

        // reverse one of the motors so they don't spin in opposite directions
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // both positive means that it moves straight by 2000 ticks (around 1.5 feet?)
        // one positive one negative means it turns. ex. -2000, 2000 means it turns left, while 2000, -2000 means it turns right
        drive(-2000, -2000, 0.8);
        drive(-2000, 2000, 0.5);
    }

    // Stop is already declared, eStop is not
    private void eStop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    
    private void drive(int leftTarget, int rightTarget, double speed) {

        // set desired target
        leftFront_Pos += leftTarget;
        leftBack_Pos += leftTarget;

        rightFront_Pos += rightTarget;
        rightBack_Pos += rightTarget;
        leftFront.setTargetPosition(leftFront_Pos);
        leftBack.setTargetPosition(leftBack_Pos);
        rightFront.setTargetPosition(rightFront_Pos);
        rightBack.setTargetPosition(rightBack_Pos);

        // move to target until target is reached
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // sets velocity for robot based on desired target
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while(opModeIsActive() && leftFront.isBusy() && leftFront.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            // ensure that nothing happens while the robot is attempting to reach the target
            idle();

        }
    }

}
