package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Timer;
import java.util.TimerTask;

public class DriveTrain {
    String DRIVE_TRAIN_CAPTION = "Drive Status";
    Telemetry telemetry;
    HardwareInnov8Hera hera;
    LinearOpMode opMode;
    Timer timer;
    TimerTask increasePower;
    private double wheelOnePower = 0.4;
    private double wheelTwoPower = 0.4;
    private double wheelThreePower = 0.4;
    private double wheelFourPower = 0.4;
    private double wheelOneRatio = 1;
    private double wheelTwoRatio = 1;
    private double wheelThreeRatio = 1;
    private double wheelFourRatio = 1;
    private int counter = 1;
    private double sumOfMotorOneDist;
    private double sumOfMotorTwoDist;
    private double sumOfMotorThreeDist;
    private double sumOfMotorFourDist;
    private double avgMotorOneDist;
    private double avgMotorTwoDist;
    private double avgMotorThreeDist;
    private double avgMotorFourDist;
    private double baseline = 0;
    public static double INCH_TO_TICK = (360/6); // The number of encoder ticks per inch for our wheels
    public static double SIDE_INCH_TO_TICK = (360/6); // The number of encoder ticks for one inch while travelling sideways, change later
    public DriveTrain(Telemetry telemetry, HardwareInnov8Hera hera, LinearOpMode opMode) {

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".

        this.opMode = opMode;
        this.hera = hera;
        this.telemetry = telemetry;
        this.telemetry.addData(DRIVE_TRAIN_CAPTION, "Drive train initialized");
        this.telemetry.update();

    }

    public void goForward(double inches) {
        double startPosition = 0;
        double endPosition = 0;
        showData("DRIVE_TRAIN_CAPTION", "Robot is moving forward");
        this.telemetry.update();
        startPosition = hera.motorOne.getCurrentPosition();
        endPosition = startPosition + (inches * INCH_TO_TICK); // How far you need to travel
        Orientation angles;
        angles = hera.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        while(!this.opMode.isStopRequested() && !hera.imu.isGyroCalibrated()) {
//            showData("IMU Status", "Calibrating");
//            showData("gyro is calibrated", "" + hera.imu.isGyroCalibrated());
//            this.opMode.sleep(50);
//            this.opMode.idle();
//        }
        double startingOrientation = angles.firstAngle;
        double error = 0;
        double steer = 0;
        while (hera.motorOne.getCurrentPosition() < endPosition && this.opMode.opModeIsActive()) {
            hera.motorOne.setPower(wheelOnePower);
            hera.motorTwo.setPower(wheelTwoPower);
            hera.motorThree.setPower(wheelThreePower);
            hera.motorFour.setPower(wheelFourPower);
            angles = hera.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = startingOrientation - angles.firstAngle;
            double powerChanger = (error/100)+1;
            if(error < 0){
                hera.motorThree.setPower(wheelThreePower * powerChanger);
                hera.motorFour.setPower(wheelFourPower * powerChanger);
                double increasePower = (Math.abs(error)/100) + 1;
                hera.motorOne.setPower(wheelOnePower * increasePower);
                hera.motorTwo.setPower(wheelTwoPower * increasePower);
            }
            if(error > 0){
                hera.motorThree.setPower(wheelThreePower * powerChanger);
                hera.motorFour.setPower(wheelFourPower * powerChanger);
                double decreasePower = 1 - (error/100);
                hera.motorOne.setPower(wheelOnePower * decreasePower);
                hera.motorTwo.setPower(wheelTwoPower * decreasePower);
            }
            showData("StartPosition", "" + startPosition);
            showData("EndPosition", "" + endPosition);
            showData("CurrentPosition", "" + hera.motorOne.getCurrentPosition());
            showData("wheel one power", "" + hera.motorOne.getPower());
            showData("wheel two power", "" + hera.motorTwo.getPower());
            showData("wheel three power", "" + hera.motorThree.getPower());
            showData("wheel four power", "" + hera.motorFour.getPower());
            showData("Initial Angle", "" + startingOrientation);
            showData("Current Angle", "" + angles.firstAngle);
            showData("error", "" + error);
            showData("steer", "" + steer);
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
        telemetry.addData("Degrees to Turn: ", degreesToTurn);
        Orientation angles;
        //test what difference between INTRINSIC and EXTRINSIC is
        angles = hera.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        degreesToTurn = angles.firstAngle + degreesToTurn;
        if (degreesToTurn < 0) {
            while ((angles.firstAngle > degreesToTurn) && this.opMode.opModeIsActive()) {
                double generalPower = (degreesToTurn - angles.firstAngle)/(degreesToTurn);
                hera.motorOne.setPower(generalPower * wheelOnePower);
                hera.motorTwo.setPower(generalPower * wheelTwoPower);
                hera.motorThree.setPower(-generalPower * wheelThreePower);
                hera.motorFour.setPower(-generalPower * wheelFourPower);
                angles = hera.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

    }

    // Put powers in the range of -1 to 1 only if they aren't already (not
    // checking would cause us to always drive at full speed)
    public void goStraight() {
        showData("DRIVE_TRAIN_CAPTION", "Balancing power");
        double wheelOneStartPos = hera.motorOne.getCurrentPosition();
        //double wheelTwoStartPos = hera.motorTwo.getCurrentPosition();
        double wheelThreeStartPos = hera.motorThree.getCurrentPosition();
        //double wheelFourStartPos = hera.motorFour.getCurrentPosition();
        showData("wheelThreeStartPos", "" + wheelThreeStartPos);
        double wheelOneEndPos = 0;
        //double wheelTwoEndPos = 0;
        double wheelThreeEndPos = 0;
        //double wheelFourEndPos = 0;
        while(hera.motorThree.getCurrentPosition() < wheelThreeStartPos + 50){
            showData("goStraight", "in the while loop");
        }
        wheelOneEndPos = Math.abs(hera.motorOne.getCurrentPosition() - wheelOneStartPos);
        //wheelTwoEndPos = Math.abs(hera.motorTwo.getCurrentPosition() - wheelTwoStartPos);
        wheelThreeEndPos = Math.abs(hera.motorThree.getCurrentPosition() - wheelThreeStartPos);
        //wheelFourEndPos = Math.abs(hera.motorFour.getCurrentPosition() - wheelFourStartPos);
        showData("wheelEndPos's", "" + wheelOneEndPos + ", " + wheelThreeEndPos);
        showData("wheelCurrentPos's", "" + hera.motorThree.getCurrentPosition() + ", " + hera.motorOne.getCurrentPosition());
        if(this.counter == 1) {
            baseline = Math.min(wheelOneEndPos, wheelThreeEndPos);
            showData("baseline calc: ", "" + wheelOneEndPos + ", " + wheelThreeEndPos);
            showData("baseline: ", "" + baseline);
        }
        sumOfMotorOneDist += wheelOneEndPos;
        //sumOfMotorTwoDist += wheelTwoEndPos;
        sumOfMotorThreeDist += wheelThreeEndPos;
        //sumOfMotorFourDist += wheelFourEndPos;
        avgMotorOneDist = sumOfMotorOneDist/counter;
        //avgMotorTwoDist = sumOfMotorTwoDist/counter;
        avgMotorThreeDist = sumOfMotorThreeDist/counter;
        //avgMotorFourDist = sumOfMotorFourDist/counter;
        showData("counter", "" + counter);
        counter++;
        //showData("wheelTwoDistance: ", "" + wheelTwoEndPos);
        //showData("wheelFourDistance: ", "" + wheelFourEndPos);

        if(baseline !=0){
            if(Math.abs(avgMotorOneDist - baseline) <= 0.5) {
                wheelOneRatio = 1;
            }
            else{
                wheelOneRatio = avgMotorOneDist/baseline;
            }
            //wheelTwoRatio = avgMotorTwoDist/baseline;
            if(Math.abs(avgMotorThreeDist - baseline) <= 0.5){
                wheelThreeRatio = 1;
            }
            else {
                wheelThreeRatio = avgMotorThreeDist / baseline;
            }
            //wheelFourRatio = avgMotorFourDist/baseline;
            showData("Ratios", "" + wheelOneRatio + ", " + wheelThreeRatio);
        }
        wheelOnePower /= wheelOneRatio;
        //wheelTwoPower /= wheelTwoRatio;
        wheelThreePower /= wheelThreeRatio;
        //wheelFourPower /= wheelFourPower;
        if(wheelOnePower>1){
            wheelOnePower = 1;
            hera.motorOne.setPower(wheelOnePower);
        }
        else {
            hera.motorOne.setPower(wheelOnePower);
        }
        //hera.motorTwo.setPower(wheelTwoPower);
        if(wheelThreePower > 1){
            wheelThreePower = 1;
            hera.motorThree.setPower(wheelThreePower);
        }
        else {
            hera.motorThree.setPower(wheelThreePower);
        }
        //hera.motorFour.setPower(wheelFourPower);
        showData("Powers" , "" + wheelOnePower + ", " + wheelThreePower);
        showData("All the Data", "" + baseline +", " + avgMotorOneDist + ", " + avgMotorThreeDist);
    }

    // Print data to both telemetry and log
    public void showData(String caption, String value) {
        this.telemetry.addData(caption, value);
        this.telemetry.update();
        Log.d(caption, value);
    }

}
