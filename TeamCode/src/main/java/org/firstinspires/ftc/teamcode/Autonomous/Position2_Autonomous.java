package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Position2_Autonomous (Blocks to Java)")
public class Position2_Autonomous extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor leftRear;
    private DcMotor ViperMotor;
    private Servo gripper;

    int LeftPos;
    int RightPos;
    int LBPos;
    int RBPos;
    int LinearPos;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        ViperMotor = hardwareMap.get(DcMotor.class, "ViperMotor");
        gripper = hardwareMap.get(Servo.class, "gripper");

        // Put initialization blocks here.
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper.setPosition(0.4);
        sleep(1000);
        ViperMotor.setPower(-0.5);
        sleep(40);
        ViperMotor.setPower(0);
        RightPos = 0;
        LeftPos = 0;
        LBPos = 0;
        RBPos = 0;
        LinearPos = 0;
        waitForStart();
        // Put function below this comment-- FUNCTION OPTIONS: 1. "Position 2 Parking 1" 2. "Position 2 Parking 2" 3. "Position 2 Parking 3"
        Position_2_Parking_2();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Left Back Position", leftRear.getCurrentPosition());
                telemetry.addData("Left Position", leftFront.getCurrentPosition());
                telemetry.addData("Right Back Position", rightRear.getCurrentPosition());
                telemetry.addData("Right Position", rightFront.getCurrentPosition());
                telemetry.addData("Linear Position", ViperMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void drive(int leftTarget, int rightTarget, int LBack, int RBack, double speed) {
        LeftPos += leftTarget;
        RightPos += rightTarget;
        LBPos += LBack;
        RBPos += RBack;
        rightFront.setTargetPosition(rightTarget);
        leftFront.setTargetPosition(leftTarget);
        rightRear.setTargetPosition(RBack);
        leftRear.setTargetPosition(LBack);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setPower(speed);
        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);
        while (opModeIsActive() && rightFront.isBusy() && leftFront.isBusy() && rightRear.isBusy() && leftRear.isBusy()) {
            idle();
        }
    }

    /**
     * Describe this function...
     */
    private void reset_values() {
        int leftTarget;
        int rightTarget;
        int LBack;
        int RBack;
        int Linear_Target;

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightPos = 0;
        LeftPos = 0;
        LBPos = 0;
        RBPos = 0;
        leftTarget = 0;
        rightTarget = 0;
        RBack = 0;
        LBack = 0;
        LinearPos = 0;
        Linear_Target = 0;
    }

    /**
     * Describe this function...
     */
    private void Position_2_Parking_2() {
        drive(1050, -1050, -1050, 1050, 0.25);
        sleep(1000);
        reset_values();
        sleep(1000);
        drive(2200, 2200, 2200, 2200, 0.25);
        sleep(1000);
        reset_values();
        sleep(1000);
        drive(-580, 580, 580, -580, 0.25);
        sleep(1000);
        reset_values();
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setPower(-0.5);
        sleep(4300);
        ViperMotor.setPower(0);
        drive(300, 300, 300, 300, 0.25);
        sleep(2000);
        gripper.setPosition(0.6);
        reset_values();
        sleep(2000);
        drive(-200, -200, -200, -200, 0.25);
        gripper.setPosition(0.4);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ViperMotor.setPower(0.5);
        sleep(1500);
        reset_values();
        sleep(1000);
        ViperMotor.setPower(0);
        drive(-700, 700, 700, -700, 0.25);
    }

    /**
     * Describe this function...
     */
    private void Position_2_Parking_1() {
        drive(1050, -1050, -1050, 1050, 0.25);
        sleep(1000);
        reset_values();
        sleep(1000);
        drive(2200, 2200, 2200, 2200, 0.25);
        sleep(1000);
        reset_values();
        sleep(1000);
        drive(-580, 580, 580, -580, 0.25);
        sleep(1000);
        reset_values();
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setPower(-0.5);
        sleep(4300);
        ViperMotor.setPower(0);
        drive(300, 300, 300, 300, 0.25);
        sleep(2000);
        gripper.setPosition(0.6);
        reset_values();
        sleep(2000);
        drive(-200, -200, -200, -200, 0.25);
        gripper.setPosition(0.4);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ViperMotor.setPower(0.5);
        sleep(1500);
        reset_values();
        sleep(1000);
        ViperMotor.setPower(0);
        drive(-1800, 1800, 1800, -1800, 0.25);
    }

    /**
     * Describe this function...
     */
    private void Position_2_Parking_3() {
        drive(1050, -1050, -1050, 1050, 0.25);
        sleep(1000);
        reset_values();
        sleep(1000);
        drive(2200, 2200, 2200, 2200, 0.25);
        sleep(1000);
        reset_values();
        sleep(1000);
        drive(-580, 580, 580, -580, 0.25);
        sleep(1000);
        reset_values();
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setPower(-0.5);
        sleep(4300);
        ViperMotor.setPower(0);
        drive(300, 300, 300, 300, 0.25);
        sleep(2000);
        gripper.setPosition(0.6);
        reset_values();
        sleep(2000);
        drive(-200, -200, -200, -200, 0.25);
        gripper.setPosition(0.4);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ViperMotor.setPower(0.5);
        sleep(1500);
        reset_values();
        sleep(1000);
        ViperMotor.setPower(0);
        drive(700, -700, -700, 700, 0.25);
    }
}