package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Encoder")
public class AutoEncoder extends BasicOpMode_Linear{

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private int leftFrontPosition;
    private int leftBackPosition;
    private int rightFrontPosition;
    private int rightBackPosition;

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_Front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_Front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Left_Back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Right_Back");

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontPosition = 0;
        leftBackPosition = 0;
        rightFrontPosition = 0;
        rightBackPosition = 0;

        waitForStart();

        drive(1000,1000,1000,1000, 0.5);
    }

        private void drive(int leftFrontTarget,
        int leftBackTarget,
        int rightFrontTarget,
        int rightBackTarget, double speed) {

            leftFrontPosition += leftFrontTarget;
            leftBackPosition += leftBackTarget;
            rightFrontPosition += rightFrontTarget;
            rightBackPosition += rightBackTarget;

            leftFrontDrive.setTargetPosition(leftFrontPosition);
            leftBackDrive.setTargetPosition(leftBackPosition);
            rightFrontDrive.setTargetPosition(rightFrontPosition);
            rightBackDrive.setTargetPosition(rightBackPosition);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            rightBackDrive.setPower(speed);


        }
    }

