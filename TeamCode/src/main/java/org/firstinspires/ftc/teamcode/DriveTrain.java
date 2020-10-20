package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain {
    String DRIVE_TRAIN_CAPTION = "Drive Status";
    Telemetry telemetry;
    HardwareInnov8Robot robot;
    LinearOpMode opMode;


    double wheelOnePower = 0.0;
    double wheelTwoPower = 0.0;
    double wheelThreePower = 0.0;
    double wheelFourPower = 0.0;
    double wheelSpeedChanger = 1.1;
    public static double INCH_TO_TICK = (360/6); // The number of encoder ticks per inch for our wheels
    public static double SIDE_INCH_TO_TICK = (360/6); // The number of encoder ticks for one inch while travelling sideways, change later

    public DriveTrain(Telemetry telemetry, HardwareInnov8Robot robot, LinearOpMode opMode) {

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".

        this.opMode = opMode;
        this.robot = robot;
        this.telemetry = telemetry;
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Drive train initialized");
        this.telemetry.update();
    }

    public void goForward(double inches) {
        double startPosition = 0;
        double endPosition = 0;
        Log.d(DRIVE_TRAIN_CAPTION, "Robot is moving forward");
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Robot is moving forward");
        this.telemetry.addData("wheel power", this.robot.motorOne.getPower());
        this.telemetry.update();
        startPosition = this.robot.motorOne.getCurrentPosition();
        endPosition = startPosition + (inches * INCH_TO_TICK); // How far you need to travel
        while (this.robot.motorOne.getCurrentPosition() < endPosition && this.opMode.opModeIsActive()) {
            this.telemetry.addData("StartPosition", startPosition);
            this.telemetry.addData("EndPosition", endPosition);
            this.telemetry.addData("CurrentPosition", this.robot.motorOne.getCurrentPosition());
            this.robot.motorOne.setPower(wheelOnePower);
            this.robot.motorTwo.setPower(wheelTwoPower);
            this.robot.motorThree.setPower((wheelThreePower));
            this.robot.motorFour.setPower(wheelFourPower);
            Log.d("wheel one power", "" + this.robot.motorOne.getPower());
            Log.d("wheel two power", "" + this.robot.motorTwo.getPower());
            Log.d("wheel three power", "" + this.robot.motorThree.getPower());
            Log.d("wheel four power", "" + this.robot.motorFour.getPower());
            Log.d("CurrentPosition", "" + this.robot.motorOne.getCurrentPosition());

            this.telemetry.update();
        }
        this.stop();
        this.telemetry.update();
    }


    public void goBackward(double inches) {
        double startPosition = 0;
        double endPosition = 0;
        Log.d(DRIVE_TRAIN_CAPTION, "Robot is moving backwards");
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Robot is moving backwards");
        startPosition = this.robot.motorOne.getCurrentPosition();
        endPosition = startPosition - (inches * INCH_TO_TICK); // How far you need to travel
        while (this.robot.motorOne.getCurrentPosition() > endPosition && this.opMode.opModeIsActive()) {
            this.robot.motorOne.setPower(-wheelOnePower);
            this.robot.motorTwo.setPower(-wheelTwoPower);
            this.robot.motorThree.setPower(-wheelThreePower);
            this.robot.motorFour.setPower(-wheelFourPower);
            Log.d("wheel one power", "" + this.robot.motorOne.getPower());
            Log.d("wheel two power", "" + this.robot.motorTwo.getPower());
            Log.d("wheel three power", "" + this.robot.motorThree.getPower());
            Log.d("wheel four power", "" + this.robot.motorFour.getPower());
            Log.d("CurrentPosition", "" + this.robot.motorOne.getCurrentPosition());
        }
        this.stop();
        this.telemetry.update();
    }

    /**
     * @param degreesToTurn Number of degrees to turn. If negative, turns right. If positive, turns left.
     */
    public void turn(double degreesToTurn) {
        double turnCorrector = 1;
        degreesToTurn = degreesToTurn * turnCorrector;
        Orientation angles;
        angles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (degreesToTurn < 0) {
            while ((angles.firstAngle > degreesToTurn) && this.opMode.opModeIsActive()) {
                double generalPower = (degreesToTurn - angles.firstAngle)/degreesToTurn;
                this.robot.motorOne.setPower(generalPower * wheelOnePower);
                this.robot.motorTwo.setPower(generalPower * wheelTwoPower);
                this.robot.motorThree.setPower(-generalPower * wheelThreePower);
                this.robot.motorFour.setPower(-generalPower * wheelFourPower);
                angles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("angles", angles.firstAngle);
                telemetry.addData("degreesToTurn", degreesToTurn);
                telemetry.update();
                Log.d("Turning", "angles 164: "  + angles.firstAngle + ", " + angles.secondAngle + ", " + angles.thirdAngle);
            }
        } else {
            while ((angles.firstAngle < degreesToTurn) && this.opMode.opModeIsActive()) {
                double generalPower = (degreesToTurn - angles.firstAngle)/degreesToTurn;
                this.robot.motorOne.setPower(-generalPower * wheelOnePower);
                this.robot.motorTwo.setPower(-generalPower * wheelTwoPower);
                this.robot.motorThree.setPower(generalPower * wheelThreePower);
                this.robot.motorFour.setPower(generalPower * wheelFourPower);
                angles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("angles", angles.firstAngle);
                telemetry.addData("degreesToTurn", degreesToTurn);
                telemetry.update();
                Log.d("Turning", "angles 176: " + angles.firstAngle + ", " + angles.secondAngle + ", " + angles.thirdAngle);
            }
        }
        this.stop();
    }

    public void goLeft(double inches) {
        double startPosition = 0;
        double endPosition = 0;
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Robot is moving left");
        startPosition = this.robot.motorOne.getCurrentPosition();
        endPosition = startPosition - (inches * SIDE_INCH_TO_TICK); // How far you need to travel
        while (this.robot.motorOne.getCurrentPosition() > endPosition && this.opMode.opModeIsActive()) {
            Log.d("going Left", "End Position: " + endPosition);
            Log.d("going Left", "Start Pos: " + startPosition);
            Log.d("going Left", "Current Pos: " + this.robot.motorOne.getCurrentPosition());

            this.robot.motorOne.setPower(-wheelOnePower);
            this.robot.motorTwo.setPower(wheelTwoPower);
            this.robot.motorThree.setPower(wheelThreePower);
            this.robot.motorFour.setPower(-wheelFourPower);
        }
        this.telemetry.update();
        this.stop();
    }

    public void goRight(double inches) {
        double startPosition = 0;
        double endPosition = 0;
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Robot is moving right");
        startPosition = this.robot.motorOne.getCurrentPosition();
        endPosition = startPosition + (inches * SIDE_INCH_TO_TICK); // How far you need to travel
        while (this.robot.motorOne.getCurrentPosition() < endPosition && this.opMode.opModeIsActive()) {
            Log.d("goRight", "start position is " + startPosition);
            Log.d("goRight", "end position is " + endPosition);
            Log.d("goRight", "current position is " + this.robot.motorFour.getCurrentPosition());
            this.robot.motorOne.setPower(wheelOnePower);
            this.robot.motorTwo.setPower(-wheelTwoPower);
            this.robot.motorThree.setPower(-wheelThreePower);
            this.robot.motorFour.setPower(wheelFourPower);
        }
        this.stop();
        this.telemetry.update();
    }

    public void stop() {
        Log.d(DRIVE_TRAIN_CAPTION, "Stopping the drive train");
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Stopping the drive train");
        this.telemetry.addData("wheel power", this.robot.motorOne.getPower());
        this.telemetry.update();
        this.robot.motorOne.setPower(0);
        this.robot.motorTwo.setPower(0);
        this.robot.motorThree.setPower(0);
        this.robot.motorFour.setPower(0);
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Drive train is stopped");
        this.telemetry.addData("wheel power", this.robot.motorOne.getPower());
        this.telemetry.update();
    }

    public void teleopUpdate(Gamepad gamepad1, Gamepad gamepad2) {

        Log.d(DRIVE_TRAIN_CAPTION, "turning teleop updated");
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "gamepad updated");
        telemetry.addData("1_left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("1_left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("1_right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("1_right_stick_y", gamepad1.right_stick_y);

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.5; // 1.5 is to counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        this.robot.motorOne.setPower(y + x + rx);
        this.robot.motorTwo.setPower(y - x + rx);
        this.robot.motorThree.setPower(y - x - rx);
        this.robot.motorFour.setPower(y + x - rx);

        balancePower();


    }

    // Put powers in the range of -1 to 1 only if they aren't already (not
    // checking would cause us to always drive at full speed)
    public void balancePower() {
        if (Math.abs(wheelOnePower) > 1 || Math.abs(wheelTwoPower) > 1 ||
                Math.abs(wheelThreePower) > 1 || Math.abs(wheelFourPower) > 1) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(wheelOnePower), Math.abs(wheelTwoPower));
            max = Math.max(Math.abs(wheelThreePower), max);
            max = Math.max(Math.abs(wheelFourPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            wheelOnePower /= max;
            wheelTwoPower /= max;
            wheelThreePower /= max;
            wheelFourPower /= max;
            telemetry.addData("max", max);
        }
        this.telemetry.update();

    }

}
