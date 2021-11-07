package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;

import java.io.IOException;


@TeleOp(name="OmniDirectionalDrive", group="Assisted Driving")
public class OmniDirectionalDrive extends LinearOpMode {
    private Robot robot;
    private final QuickTelemetry quickTelemetry = new QuickTelemetry(telemetry);
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double power = .30;

    private void initOpMode() throws IOException {
        ElapsedTime timer = new ElapsedTime();
        robot = new Robot(this, timer, true);
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        quickTelemetry.telemetry("Mode", "calibrating...");

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        quickTelemetry.telemetry("Mode", "waiting for start");
        quickTelemetry.telemetry("imu calib status", imu.getCalibrationStatus().toString());

        // wait for start button.
        waitForStart();

        quickTelemetry.telemetry("Mode", "running");

        // wait for 1 second
        sleep(1000);


        while (opModeIsActive())
        {
            //Get gamepad inputs
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            double rightStickY = -gamepad1.right_stick_y;
//
//            //Get robot angle
//            double robotAngle = imu.getAngularOrientation().firstAngle;
//            double robotAngle360 = to360(robotAngle);

            //Synthesize robot angle
            double robotAngle = Math.atan2(rightStickY, rightStickX) - Math.PI / 2;
            double robotAngle360 = to360(robotAngle);

            //Get controller angle
            double controllerAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 2;
            double controllerAngle360 = to360(controllerAngle);

            //Find angle between controller and robot
            double angleBetween = smallestAngleBetween(robotAngle360,controllerAngle360)*findRotationDirection(robotAngle360,controllerAngle360);
            double angleBetween360 = to360(angleBetween);

            //Convert angle to X and Y
            double correctedX = Math.cos(angleBetween360 + Math.PI/2);
            double correctedY = Math.sin(angleBetween360 + Math.PI/2);

            //Drive the motors
            double powers[] = calcMotorPowers(rightStickX, correctedY, 0);
            robot.drive.rearLeft.setPower(powers[0]);
            robot.drive.frontLeft.setPower(powers[1]);
            robot.drive.rearRight.setPower(powers[2]);
            robot.drive.frontRight.setPower(powers[3]);

            quickTelemetry.telemetry("Fake RBT Angle", ((Double) robotAngle360).toString());
            quickTelemetry.telemetry("CNTRL Angle", ((Double) controllerAngle360).toString());


            resetAngle();
        }

        // turn the motors off.
        stopMotors();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */


    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private void stopMotors() {
        //Stops the motor
        robot.frontLeftDriveMotor.setPower(0);
        robot.frontRightDriveMotor.setPower(0);
        robot.rearLeftDriveMotor.setPower(0);
        robot.rearRightDriveMotor.setPower(0);
    }

    private double to360(double angle) {
        //Converts from euler units to 360 degrees
        //Goes from 0 to 360 in a clockwise fasion
        //Accepts numbers between -180 and 180
        if (angle >= 0) {
            return angle;
        } else {
            return angle + 360;
        }
    }

    private double smallestAngleBetween(double angle1, double angle2) {
        //Returns the smallest angle between angle1 and angle 2
        //Accepts the range 0 - 360 for both angles
        double distanceBetween = Math.abs(angle2 - angle1);
        if ((360 - distanceBetween) < distanceBetween) {
            return 360 - distanceBetween;
        } else {
            return distanceBetween;
        }
    }

    private double findRotationDirection(double robot, double goal) {
        //Determines the shortest way to rotate to goal angle
        //Accepts angles from 0 - 360 for both inputs
        double i;
        if (robot <= 180) {
            if (goal < robot || goal > robot + 180) {
                i= -1;
            } else {
                i = 1;
            }
        } else {
            if (goal > robot || goal < robot - 180) {
                i = 1;
            } else {
                i = -1;
            }
        }

        return i;
    }

    private double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX) {
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }

}
