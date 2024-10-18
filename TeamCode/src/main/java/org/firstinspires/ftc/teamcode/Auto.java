package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class Auto extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    int frontLeftPos;
    int frontRightPos;
    int backLeftPos;
    int backRightPos;

    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftPos = 0;
        frontRightPos = 0;
        backLeftPos = 0;
        backRightPos = 0;


        waitForStart();


       // drive(966.0846,966.0846,966.0846,966.0846,0.25);
       // drive(-736.413,736.413,736.413,-736.413,0.25);
       // drive(806.442,806.442,806.442,806.442,0.25);
       // drive(736.413,-736.413,-736.413,736.413,0.25);
        //drive(1500,1500,1500,1500,0.25);
        //drive(-1447.72,1447.72,1447.72,-1447.72,0.25);
        //drive(2370,2370,2370,2370,0.25);
        //drive(-300,-300,-300,-300,0.25);
        //drive(-1200,500,500,-1200,0.25);
    }
        private void drive(double bLeftTarget, double bRightTarget, double fRightTarget, double fLeftTarget, double speed ) {
            frontLeftPos += fLeftTarget;
            frontRightPos += fRightTarget;
            backLeftPos += bLeftTarget;
            backRightPos += bRightTarget;

            frontLeft.setTargetPosition(frontLeftPos);
            frontRight.setTargetPosition(frontRightPos);
            backLeft.setTargetPosition(backLeftPos);
            backRight.setTargetPosition(backRightPos);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);

            while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                idle();
                // sleep(1000);

                //frontLeft.setPower(0);
                //  frontRight.setPower(0);
                //  backLeft.setPower(0);
                //  backRight.setPower(0);
            }
            }
    }
