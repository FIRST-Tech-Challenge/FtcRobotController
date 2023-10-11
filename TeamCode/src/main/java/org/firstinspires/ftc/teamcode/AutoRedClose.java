package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "AutoRedClose", group = "linear autoMode")

public class AutoRedClose extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private double speed;
    private final int oneRotaion = 540;
    private int newLFTarget;
    private int newLBTarget;
    private int newRFTarget;
    private int newRBTarget;

    public void driveForward(double speed, double rotations){

    }

    public void driveRight(double speed, double rotations){

    }

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "frontleft");
        leftBack = hardwareMap.get(DcMotor.class, "backleft");
        rightFront = hardwareMap.get(DcMotor.class, "frontright");
        rightBack = hardwareMap.get(DcMotor.class, "backright");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        speed = 1.0;

        newLFTarget = leftFront.getCurrentPosition() + oneRotaion;
        newLBTarget = leftBack.getCurrentPosition() + oneRotaion;
        newRFTarget = rightFront.getCurrentPosition() + oneRotaion;
        newRBTarget = rightBack.getCurrentPosition() + oneRotaion;

        leftFront.setTargetPosition(newLFTarget);
        leftBack.setTargetPosition(newLBTarget);
        rightFront.setTargetPosition(newRFTarget);
        rightBack.setTargetPosition(newRBTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            continue;
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(5000);

        speed = 0.25;

        newLFTarget = leftFront.getCurrentPosition() + oneRotaion;
        newLBTarget = leftBack.getCurrentPosition() + oneRotaion;
        newRFTarget = rightFront.getCurrentPosition() + oneRotaion;
        newRBTarget = rightBack.getCurrentPosition() + oneRotaion;

        leftFront.setTargetPosition(newLFTarget);
        leftBack.setTargetPosition(newLBTarget);
        rightFront.setTargetPosition(newRFTarget);
        rightBack.setTargetPosition(newRBTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            continue;
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(5000);


    }
}

