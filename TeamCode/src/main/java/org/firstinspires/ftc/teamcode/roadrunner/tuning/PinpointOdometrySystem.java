package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.tuning.GoBildaPinpointDriver;

public class PinpointOdometrySystem {
    private GoBildaPinpointDriver pinpointDriver;
    private ElapsedTime timer;

    // Robot center position values
    private double xPos = 0.0;
    private double yPos = 0.0;
    private double heading = 0.0;

    public PinpointOdometrySystem(HardwareMap hardwareMap, String i2cDeviceName) {
        // Initialize I2C device for Pinpoint
        I2cDeviceSynch pinpointDevice = hardwareMap.get(I2cDeviceSynch.class, i2cDeviceName);
        pinpointDriver = new GoBildaPinpointDriver(pinpointDevice, true);

        // Initialize the odometry system
        pinpointDriver.initialize();
        timer = new ElapsedTime();
    }

    public void update() {
        // Update odometry data from the Pinpoint
        pinpointDriver.update();
        xPos = pinpointDriver.getXPosMM() / 0.0393701; // Convert mm to inches
        yPos = pinpointDriver.getYPosMM() / 0.0393701; // Convert mm to inches
        heading = pinpointDriver.getHeadingDegrees();

    }

    public void resetPosition() {
        // Reset odometry position and IMU calibration
        pinpointDriver.resetPosAndIMU();
    }

    public double getXPosition() {
        return xPos;
    }

    public double getYPosition() {
        return yPos;
    }

    public double getHeading() {
        return heading;
    }

    public void setOffsets(double xOffsetMM, double yOffsetMM, double headingOffsetDeg) {
        pinpointDriver.setOffsets(xOffsetMM, yOffsetMM, headingOffsetDeg);
    }

    public void setEncoderDirections(boolean xReversed, boolean yReversed) {
        GoBildaPinpointDriver.EncoderDirection x;
        GoBildaPinpointDriver.EncoderDirection y;
        if(xReversed)
        {
            x = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        }else {
            x= GoBildaPinpointDriver.EncoderDirection.FORWARD;
        }

        if(yReversed)
        {
            y = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        }else {
            y = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        }
        pinpointDriver.setEncoderDirections(x, y);
    }
}

