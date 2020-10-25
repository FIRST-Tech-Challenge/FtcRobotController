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
    HardwareInnov8Hera hera;
    LinearOpMode opMode;


    double wheelOnePower = 0.0;
    double wheelTwoPower = 0.0;
    double wheelThreePower = 0.0;
    double wheelFourPower = 0.0;
    public static double INCH_TO_TICK = (360/6); // The number of encoder ticks per inch for our wheels
    public static double SIDE_INCH_TO_TICK = (360/6); // The number of encoder ticks for one inch while travelling sideways, change later

    public DriveTrain(Telemetry telemetry, HardwareInnov8Hera hera, LinearOpMode opMode) {

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".

        this.opMode = opMode;
        hera= hera;
        this.telemetry = telemetry;
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Drive train initialized");
        this.telemetry.update();
    }

    public void goForward(double inches) {
        double startPosition = 0;
        double endPosition = 0;
        showData("DRIVE_TRAIN_CAPTION", "Robot is moving forward");
        this.telemetry.addData("wheel power", hera.motorOne.getPower());
        this.telemetry.update();
        startPosition = hera.motorOne.getCurrentPosition();
        endPosition = startPosition + (inches * INCH_TO_TICK); // How far you need to travel
        while (hera.motorOne.getCurrentPosition() < endPosition && this.opMode.opModeIsActive()) {
            this.telemetry.addData("StartPosition", startPosition);
            this.telemetry.addData("EndPosition", endPosition);
            this.telemetry.addData("CurrentPosition", hera.motorOne.getCurrentPosition());
            hera.motorOne.setPower(wheelOnePower);
            hera.motorTwo.setPower(wheelTwoPower);
            hera.motorThree.setPower((wheelThreePower));
            hera.motorFour.setPower(wheelFourPower);
            showData("wheel one power", "" + hera.motorOne.getPower());
            showData("wheel two power", "" + hera.motorTwo.getPower());
            showData("wheel three power", "" + hera.motorThree.getPower());
            showData("wheel four power", "" + hera.motorFour.getPower());
            showData("CurrentPosition", "" + hera.motorOne.getCurrentPosition());

            this.telemetry.update();
        }
        this.stop();
        this.telemetry.update();
    }


    public void goBackward(double inches) {
        double startPosition = 0;
        double endPosition = 0;
        showData("DRIVE_TRAIN_CAPTION", "Robot is moving backwards");
        startPosition = hera.motorOne.getCurrentPosition();
        endPosition = startPosition - (inches * INCH_TO_TICK); // How far you need to travel
        while (hera.motorOne.getCurrentPosition() > endPosition && this.opMode.opModeIsActive()) {
            hera.motorOne.setPower(-wheelOnePower);
            hera.motorTwo.setPower(-wheelTwoPower);
            hera.motorThree.setPower(-wheelThreePower);
            hera.motorFour.setPower(-wheelFourPower);
            showData("wheel one power", "" + hera.motorOne.getPower());
            showData("wheel two power", "" + hera.motorTwo.getPower());
            showData("wheel three power", "" + hera.motorThree.getPower());
            showData("wheel four power", "" + hera.motorFour.getPower());
            showData("CurrentPosition", "" + hera.motorOne.getCurrentPosition());
        }
        this.stop();
    }

    /**
     * @param degreesToTurn Number of degrees to turn. If negative, turns right. If positive, turns left.
     */
    public void turn(double degreesToTurn) {
        double turnCorrector = 1;
        degreesToTurn = degreesToTurn * turnCorrector;
        Orientation angles;
        angles = hera.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (degreesToTurn < 0) {
            while ((angles.firstAngle > degreesToTurn) && this.opMode.opModeIsActive()) {
                double generalPower = (degreesToTurn - angles.firstAngle)/degreesToTurn;
                hera.motorOne.setPower(generalPower * wheelOnePower);
                hera.motorTwo.setPower(generalPower * wheelTwoPower);
                hera.motorThree.setPower(-generalPower * wheelThreePower);
                hera.motorFour.setPower(-generalPower * wheelFourPower);
                angles = hera.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("degreesToTurn", degreesToTurn);
                telemetry.update();
                String turnInfo = "angles: " + angles.firstAngle + ", " + angles.secondAngle + ", " + angles.thirdAngle;
                showData("Turning", turnInfo);
            }
        } else {
            while ((angles.firstAngle < degreesToTurn) && this.opMode.opModeIsActive()) {
                double generalPower = (degreesToTurn - angles.firstAngle)/degreesToTurn;
                hera.motorOne.setPower(-generalPower * wheelOnePower);
                hera.motorTwo.setPower(-generalPower * wheelTwoPower);
                hera.motorThree.setPower(generalPower * wheelThreePower);
                hera.motorFour.setPower(generalPower * wheelFourPower);
                angles = hera.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("degreesToTurn", degreesToTurn);
                telemetry.update();
                String turnInfo = "angles: " + angles.firstAngle + ", " + angles.secondAngle + ", " + angles.thirdAngle;
                showData("Turning", turnInfo);
            }
        }
        this.stop();
    }

    public void goLeft(double inches) {
        double startPosition = 0;
        double endPosition = 0;
        showData("DRIVE_TRAIN_CAPTION", "Robot is moving left");
        startPosition = hera.motorOne.getCurrentPosition();
        endPosition = startPosition - (inches * SIDE_INCH_TO_TICK); // How far you need to travel
        while (hera.motorOne.getCurrentPosition() > endPosition && this.opMode.opModeIsActive()) {
            showData("going Left", "End Position: " + endPosition);
            showData("going Left", "Start Pos: " + startPosition);
            showData("going Left", "Current Pos: " + hera.motorOne.getCurrentPosition());

            hera.motorOne.setPower(-wheelOnePower);
            hera.motorTwo.setPower(wheelTwoPower);
            hera.motorThree.setPower(wheelThreePower);
            hera.motorFour.setPower(-wheelFourPower);
        }
        this.stop();
    }

    public void goRight(double inches) {
        double startPosition = 0;
        double endPosition = 0;
        showData("DRIVE_TRAIN_CAPTION", "Robot is moving right");
        startPosition = hera.motorOne.getCurrentPosition();
        endPosition = startPosition + (inches * SIDE_INCH_TO_TICK); // How far you need to travel
        while (hera.motorOne.getCurrentPosition() < endPosition && this.opMode.opModeIsActive()) {
            showData("goRight", "start position is " + startPosition);
            showData("goRight", "end position is " + endPosition);
            showData("goRight", "current position is " + hera.motorFour.getCurrentPosition());
            hera.motorOne.setPower(wheelOnePower);
            hera.motorTwo.setPower(-wheelTwoPower);
            hera.motorThree.setPower(-wheelThreePower);
            hera.motorFour.setPower(wheelFourPower);
        }
        this.stop();
    }

    public void stop() {
        showData("DRIVE_TRAIN_CAPTION", "Stopping the drive train");
        this.telemetry.addData("wheel power", hera.motorOne.getPower());
        this.telemetry.update();
        hera.motorOne.setPower(0);
        hera.motorTwo.setPower(0);
        hera.motorThree.setPower(0);
        hera.motorFour.setPower(0);
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Drive train is stopped");
        this.telemetry.addData("wheel power", hera.motorOne.getPower());
        this.telemetry.update();
    }

    public void teleopUpdate(Gamepad gamepad1, Gamepad gamepad2) {

        showData("DRIVE_TRAIN_CAPTION", "Teleop updated");
        telemetry.addData("1_left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("1_left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("1_right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("1_right_stick_y", gamepad1.right_stick_y);

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.5; // 1.5 is to counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        hera.motorOne.setPower(y + x + rx);
        hera.motorTwo.setPower(y - x + rx);
        hera.motorThree.setPower(y - x - rx);
        hera.motorFour.setPower(y + x - rx);

        balancePower();
    }

    // Put powers in the range of -1 to 1 only if they aren't already (not
    // checking would cause us to always drive at full speed)
    public void balancePower() {
        showData("DRIVE_TRAIN_CAPTION", "Balancing power");
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

    }

    // Print data to both telemetry and log
    public void showData(String caption, String value) {
        this.telemetry.addData(caption, value);
        this.telemetry.update();
        Log.d(caption, value);
    }

}
