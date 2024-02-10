package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutoPixelRed")
public class autoTeleopPixelRed extends LinearOpMode {

    //These are the distances for each auto step
    IMU imu = hardwareMap.get(IMU.class, "imu");
    // Adjust the orientation parameters to match your robot
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
    //imu.initialize(parameters);
    //Field-centric initialization - end
    //imu.resetYaw();

    public static int step1, step2, step3, step4, step5;

    double secondLiftDesignBoost = 1.15;     //for original design it is 1

    @Override
    public void runOpMode() throws InterruptedException {

        driveRobot(step1, 0.0, 0.5);           //drive right
        driveRobot(step2, 0.5, 0.0);           //drive backward

        raisePixelArm();    //deliver PIXEL (raise PIXEL arm)
        sleep(3000);    //sleep for 3 seconds to let the pixels fall into place

        resetPixelArm();    //lower PIXEL arm
        sleep(1000);    //sleep for 3 seconds to let the pixels fall into place

        driveRobot(step3, -0.5, 0.0);          //drive forward
        driveRobot(step4, 0.0, -0.5);          //drive left
        driveRobot(step5, 0.5, 0.0);           //drive backward

        //turnRobot(0.5);
        //turnRobot(-0.5);

    }

    private void driveRobot(int distance, double leftStickY, double leftStickX) {

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

        //NOTE: 7000 is roughly 6 feet
        //NOTE: 3000 was roughly 2.5 feet
        while (Math.abs((robotCurrentPosition - robotStartPosition)) < distance) {

            double y = leftStickY;      //-gamepad1.left_stick_y;       //0.5; (this was driving backward)
            double x = leftStickX;      //gamepad1.left_stick_x;
            double rx = 0.0;    //gamepad1.right_stick_x;

            if (!startChecked) {
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

    private void turnRobot(double rightStickX) {

        DcMotor frontLeft = hardwareMap.dcMotor.get("bl");
        DcMotor backLeft = hardwareMap.dcMotor.get("fl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        int robotStartPosition = 0;
        boolean startChecked = false;
        int robotCurrentPosition = 0;

        double botHeadingOriginal = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.setAutoClear(true);
        telemetry.update();
        float driveSpeed = 0.5f;

        waitForStart();

        if (isStopRequested()) return;

        //NOTE: 7000 is roughly 6 feet
        //NOTE: 3000 was roughly 2.5 feet
        while (Math.abs((botHeadingOriginal - botHeading)) < 90) {

            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double y = 0.0;      //-gamepad1.left_stick_y;       //0.5; (this was driving backward)
            double x = 0.0;      //gamepad1.left_stick_x;
            double rx = rightStickX;     //rightStickX;    //gamepad1.right_stick_x;

            if (!startChecked) {
                robotStartPosition = backLeft.getCurrentPosition();
                startChecked = true;
            }

            robotCurrentPosition = backLeft.getCurrentPosition();

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

    private void raisePixelArm() {
        DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter");                //pixel lifter
        float lifterLowerMotorPower = 0;        //gamepad2.right_trigger;
        float lifterRaiseMotorPower = 0.5F;        //gamepad2.left_trigger;

        //raise the pixel arm to deliver the pixel
        while (lifterMotor.getCurrentPosition() < 695) {
            if (lifterRaiseMotorPower > 0) {

                if (Math.abs(lifterMotor.getCurrentPosition()) < 695 &&
                        Math.abs(lifterMotor.getCurrentPosition()) > 600) {                 //was 1025 with the old motor, now 701 (using a value that is 1% less than that)
                    //pixellifterRaise
                    lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.3 * (lifterRaiseMotorPower));

                } else if (Math.abs(lifterMotor.getCurrentPosition()) < 600) {                 //was 1025 with the old motor, now 701 (using a value that is 1% less than that)
                    //pixellifterRaise
                    lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.5 * (lifterRaiseMotorPower));

                } else {
                    lifterMotor.setPower(0f);
                }
            } else {
                //intake
                lifterMotor.setPower(0);
            }
        }
    }

    public void resetPixelArm() {
        DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter");                //pixel lifter
        float lifterLowerMotorPower = 0.5F;        //gamepad2.right_trigger;
        float lifterRaiseMotorPower = 0;        //gamepad2.left_trigger;

        while (lifterMotor.getCurrentPosition() > 0) {

            if (lifterLowerMotorPower > 0) {

                if (Math.abs(lifterMotor.getCurrentPosition()) > 600) {
                    //pixellifter - Lower
                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.7 * (lifterLowerMotorPower));

                } else if (Math.abs(lifterMotor.getCurrentPosition()) <= 600 &&
                        Math.abs(lifterMotor.getCurrentPosition()) > 200) {
                    //pixellifter - Lower
                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.5 * (lifterLowerMotorPower));

                } else if (Math.abs(lifterMotor.getCurrentPosition()) > 40) {
                    //pixellifter - Lower
                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.3 * (lifterLowerMotorPower));

                } else {
                    lifterMotor.setPower(0f);
                }
            }

        }

    }
}

