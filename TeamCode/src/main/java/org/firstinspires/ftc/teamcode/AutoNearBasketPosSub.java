package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class AutoNearBasketPosSub extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    Servo joint;

    int frontLeftPos;
    int frontRightPos;
    int backLeftPos;
    int backRightPos;

    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        joint = hardwareMap.get(Servo.class, "joint");

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

        joint.setPosition(0.15);

        waitForStart();
        //850 Ticks for 90 degrees
        //9.44 Tick/Degree
        //14.38 Tick/cm

       drive(1000, 1000, 1000, 1000, 0.4);
        //put specimen
       sleep(2000);
        //move backwards
        drive(-500,500,-500,-500, 0.4);
        //turn 90 degrees right
        drive(850,-850,-850,850, 0.4);
        //drive forward
        drive(1602.8,1602.8,1602.8, 1602.8, 0.4);
        //turn right
        drive(850,-850,-850,850,0.4);
        //drive forward
        drive(420, 420, 420, 420, 0.4);
        //pick specimen
        sleep(2000);
        //drive backward
        drive(-420, -420, -420, -420, 0.4);
        //turn left
        drive(-850, 850, 850, -850, 0.4);
        //drive backwards
        drive(-1902, -1902, -1902, -1902, 0.4);
        //turn 90 degrees left
        drive(-850, 850, 850, -850, 0.4);
        //drive forward
        drive(500, 500, 500, 500, 0.4);
        //put specimen
        sleep(2000);
        //move backwards
        drive(-500,500,-500,-500, 0.4);
        //turn 90 degrees right
        drive(850,-850,-850,850, 0.4);
        //drive forward
        drive(1902.8,1902.8,1902.8, 1902.8, 0.4);
        //turn left 90 degrees
        drive(-850, 850, 850, -850,0.4);
        //drive forward
        drive(640,640,640,640, 0.4);
        //turn right 90
        drive(850, -850, -850, 850, 0.4);
        //drive forward
        drive(250, 250, 250, 250, 0.4);
        //turn left 90
        drive(-850, 850, 850, -850, 0.4);
        //drive backwards
        drive(-1120, -1120, -1120, -1120, 0.4);
        sleep(2000);
        //drive forwards
        drive(1120, 1120, 1120, 1120, 0.4);
        //turn right 90
        drive(850, -850, -850, 850, 0.4);
        //drive forward
        drive(180, 180, 180, 180, 0.4);
        //turn left 90
        drive(-850, 850, 850, -850, 0.4);
        //drive backwards
        drive(-1120, -1120, -1120, -1120, 0.4);
        //drive backwards
        drive(-100, -100, -100, -100, 0.4);

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
