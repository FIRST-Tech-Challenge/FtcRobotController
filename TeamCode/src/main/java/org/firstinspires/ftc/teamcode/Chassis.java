package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Chassis {
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private DistanceSensor horizontalDistanceSensor, verticalDistanceSensor;
    private IMU imu;


    public Chassis(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        horizontalDistanceSensor = hardwareMap.get(DistanceSensor.class, "horizontalDistanceSensor");
        verticalDistanceSensor = hardwareMap.get(DistanceSensor.class, "verticalDistanceSensor");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Set motor directions (adjust as needed)
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        // Reset encoders
        resetEncoders();

        // Set run modes
        setRunUsingEncoders();
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunUsingEncoders() {
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Continually call this function to align the robot to a wall.
     * Can be called several times to align to multiple walls.
     * @param wall: WallType.LEFT or WallType.BACK
     * @param distance: distance from wall in cm
     * @param angleOffset: angle offset from default position in degrees
     */
    public void alignToWall(twoFeetAutoBasketProgramBase.WallType wall, double distance, double angleOffset) {
        int threshold = 20; // encoder ticks
        double currentDistance = 0;
        if (wall == twoFeetAutoBasketProgramBase.WallType.LEFT) {
            currentDistance = horizontalDistanceSensor.getDistance(DistanceUnit.CM);
        } else if (wall == twoFeetAutoBasketProgramBase.WallType.BACK) {
            currentDistance = verticalDistanceSensor.getDistance(DistanceUnit.CM);
        }

        if (currentDistance > (distance + threshold)) {

        } else if (currentDistance < (distance - threshold)) {

        }
    }
}
