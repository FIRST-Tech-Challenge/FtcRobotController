package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain {
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final DistanceSensor horizontalDistanceSensor, verticalDistanceSensor;
    private final IMU imu;
    private final List<Double> xWeights, yWeights, rWeights;
    private final double distanceThreshold, angleThreshold, decelerationDistance;

    /**
     * Initializes the drivetrain.
     *
     * @param hardwareMap: hardware map from OpMode
     * @param distanceThreshold: distance threshold for wall alignment in CM
     * @param angleThreshold: angle threshold for wall alignment in DEGREES
     * @param decelerationDistance: distance threshold for deceleration in CM
     */
    public Drivetrain(HardwareMap hardwareMap, double distanceThreshold, double angleThreshold, double decelerationDistance) {
        this.distanceThreshold = distanceThreshold;
        this.angleThreshold = angleThreshold;
        this.decelerationDistance = decelerationDistance;

        // initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        horizontalDistanceSensor = hardwareMap.get(DistanceSensor.class, "horizontalDistanceSensor");
        verticalDistanceSensor = hardwareMap.get(DistanceSensor.class, "verticalDistanceSensor");

        xWeights = new ArrayList<Double>(); // (-1.0 to 1.0)
        yWeights = new ArrayList<Double>(); // (-1.0 to 1.0)
        rWeights = new ArrayList<Double>(); // (-1.0 to 1.0)

        // Set motor directions (adjust as needed)
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        // set run mode of all motors
        setRunWithoutEncoders();
    }

    private void setRunWithoutEncoders() {
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Calculates the average of an array of numbers.
     * @param array: array of numbers
     * @return average of array
     */
    private double calculateAverage(List<Double> array) {
        if (array == null || array.isEmpty()) {
            return 0.0; // Handle empty array case
        }

        double sum = 0;
        for (double number : array) {
            sum += number;
        }

        return sum / array.size();
    }

    /**
     * Calculates a parabolic weight based on the distance difference.
     *
     * @param distanceDifference The difference between the current and target distance.
     * @param maxDistance        The maximum distance for full power.
     * @return The calculated weight (between 0.0 and 1.0).
     */
    private double calculateParabolicWeight(double distanceDifference, double maxDistance) {
        // Ensure distanceDifference is not negative
        distanceDifference = Math.abs(distanceDifference);

        // Normalize the distance difference to a range of 0 to 1
        double normalizedDistance = Math.min(distanceDifference / maxDistance, 1.0);

        // Apply a parabolic function (y = 1 - (1-x)^2) for smooth acceleration
        return 1.0 - Math.pow(1.0 - normalizedDistance, 2.0);
    }

    public enum WallType {
        LEFT,
        BACK
    }

    /**
     * Continually call this function to align the robot to a wall.
     * Can be called several times to align to multiple walls.
     * @param wall: WallType.LEFT or WallType.BACK
     * @param distance: distance from wall in cm
     * @param angleOffset: angle offset from default position in degrees
     */
    public void alignToWall(WallType wall, double distance, double angleOffset) {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.DEGREES);

        // get distance from wall depending on which type is selected (left or back)
        double currentDistance = (wall == WallType.LEFT)
                ? horizontalDistanceSensor.getDistance(DistanceUnit.CM) : verticalDistanceSensor.getDistance(DistanceUnit.CM);

        // distance compensation calc
        if (currentDistance > (distance + distanceThreshold)) {

            // calculate weight using parabolic function
            double weight = calculateParabolicWeight(currentDistance - (distance + distanceThreshold), decelerationDistance);

            if (wall == WallType.LEFT){
                xWeights.add(-weight); // move left
            } else if (wall == WallType.BACK){
                yWeights.add(-weight); // move back
            }
        } else if (currentDistance < (distance - distanceThreshold)) {

            // calculate weight using parabolic function
            double weight = calculateParabolicWeight((distance - distanceThreshold) - currentDistance, decelerationDistance);

            if (wall == WallType.LEFT){
                xWeights.add(weight); // move right
            } else if (wall == WallType.BACK){
                yWeights.add(weight); // move forward
            }
        }

        // angle compensation calc
        if (currentAngle > (angleOffset + angleThreshold)) {
            rWeights.add(-0.5); // rotate left
        } else if (currentAngle < (angleOffset - angleThreshold)) {
            rWeights.add(0.5); // rotate right
        }
    }

    /**
     * Nudges the divetrain in a specific direction.
     * This will be averaged with the wall alignment calculations.
     *
     * @param x: direction (-1.0 to 1.0)
     * @param y: direction (-1.0 to 1.0)
     */
    public void nudgeInDirection(double x, double y){

        // make sure input is within range (-1.0 to 1.0)
        x /= Math.max(1.0, Math.abs(x));
        y /= Math.max(1.0, Math.abs(y));

        // add nudges to lists
        xWeights.add(x);
        yWeights.add(y);

    }

    /**
     * Checks if the drivetrain is aligned to a wall.
     * Must be called before nudging in any direction.
     *
     * @return boolean: true if aligned to wall, false otherwise
     */
    public boolean isAlignedToWall(){
        return yWeights.isEmpty() && xWeights.isEmpty() && rWeights.isEmpty();
    }

    /**
     * Must be continually called within your main loop to manage the drivetrain motors
     */
    public void update(){
        // Set run modes
        setRunWithoutEncoders();

        // average all nudges and wall alignments
        double y = calculateAverage(yWeights); // Forward/backward
        double x = calculateAverage(xWeights); // Strafe
        double rotation = calculateAverage(rWeights); // Rotate

        // clear lists
        xWeights.clear();
        yWeights.clear();
        rWeights.clear();

        // mecanum drive calculations
        double frontLeftPower = (y + x + rotation);
        double frontRightPower = (y - x - rotation);
        double backLeftPower = (y - x + rotation);
        double backRightPower = (y + x - rotation);

        // normalize the power values to be within -1 and 1
        frontLeftPower /= Math.max(1.0, Math.abs(frontLeftPower));
        frontRightPower /= Math.max(1.0, Math.abs(frontRightPower));
        backLeftPower /= Math.max(1.0, Math.abs(backLeftPower));
        backRightPower /= Math.max(1.0, Math.abs(backRightPower));

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }

    /**
     * Stops the drivetrain motors.
     */
    public void stop(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
