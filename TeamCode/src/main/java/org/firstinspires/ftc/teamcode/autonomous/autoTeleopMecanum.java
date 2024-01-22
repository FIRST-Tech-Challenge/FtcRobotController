package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "AutoTelepMec")
public class autoTeleopMecanum extends LinearOpMode {

    //comment to help with commit2

    @Override
    public void runOpMode() throws InterruptedException {

//        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
//        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
//Based on testing, the front and back left motors were reversed.  Reversing them here (FIXED THE ISSUE!!!)

        DcMotor frontLeft = hardwareMap.dcMotor.get("bl");
        DcMotor backLeft = hardwareMap.dcMotor.get("fl");

        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        int robotStartPosition = 0;
        boolean startChecked = false;
        int robotCurrentPosition = 0;

        telemetry.setAutoClear(true);
        telemetry.update();
        float driveSpeed = 0.5f;

        waitForStart();

        if (isStopRequested()) return;

//        while (opModeIsActive() && (robotCurrentPosition - robotStartPosition) < 7000) {
        //NOTE: 7000 is roughly 6 feet
        //NOTE: 3000 was roughly 2.5 feet
        while (Math.abs((robotCurrentPosition - robotStartPosition)) < 4000) {

//            double y = -gamepad1.left_stick_y;
            double y = 0.5;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if(!startChecked) {
                robotStartPosition = backLeft.getCurrentPosition();
                startChecked = true;
            }

            robotCurrentPosition = backLeft.getCurrentPosition();

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("BL encCurPos: ", backLeft.getCurrentPosition());
            telemetry.addData("leftJoy x: ", x);
            telemetry.addData("leftJoy y: ", y);
            telemetry.addData("Distance travelled: ", Math.abs((robotCurrentPosition - robotStartPosition)));

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower * driveSpeed);
            backLeft.setPower(backLeftPower * driveSpeed);
            frontRight.setPower(frontRightPower * driveSpeed);
            backRight.setPower(backRightPower * driveSpeed);


            telemetry.update();

        }
            }
        }

