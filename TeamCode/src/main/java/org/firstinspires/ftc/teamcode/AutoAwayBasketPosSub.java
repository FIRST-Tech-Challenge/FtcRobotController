package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class AutoAwayBasketPosSub extends LinearOpMode {
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


        //Strafing Right 10 cm
        drive(-143.8, 143.8, -143.8, 143.8, 0.4);
        //Backwards 90 cm
        drive(-1490, -1490, -1490, -1490, 0.4);
        sleep(500);
        //Forward 32 cm
        drive(355.16, 355.16, 355.16, 355.16, 0.7);
        //Strafe Right 119 cm
        drive(-2700.22, 2700.22, -2700.22, 2700.22, 0.7);
        //Turn Right 60 degrees
        drive(750.4, -750.4, -750.4, 750.4, 0.7);
        //STrafe Right
        drive(-300,300,-300, 300, 0.4);
        //Backward 117cm
        drive(-2150.11, -2150.11, -2150.11, -2150.11, 0.4);
        sleep(500);
        //Forward 117cm

        drive(2000.11, 2000.11, 2000.11, 2000.11, 0.4);
        //Turn 30 degrees Right
        drive(175.6, -175.6, -175.6, 175.6, 0.7);
        //Strafe Right
        drive(-850.8, 850.8, -850.8, 850.8, 0.4);
        //Drive Back
        drive(-1900.11, -1900.11, -1900.11, -1900.11, 0.4);
        sleep(500);
        //Drive More Back
        //drive(-100, -100, -100, -100, 0.25);
        //Strafe Left
        drive(4658, -4658, 4658, -4658, 0.7);
        //Move forward
        drive(1900.11, 1900.11, 1900.11, 1900.11, 0.5);
        //Move Left 90 deg
        drive(-906, 906, 906, -906, 0.7);
        //Back
        drive(-500, -500, -500, -500, 0.5);

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

