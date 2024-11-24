package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.GoBildaPinpointDriver;

public class PinpointOdometrySystem implements Localizer {
    private GoBildaPinpointDriver pinpointDriver;
    private ElapsedTime timer;

    // Robot center position values
    private double xPos = 0.0;
    private double yPos = 0.0;
    private double heading = 0.0;
    private boolean initialized;

    public double lastPosX;
    public double lastPosY;
    public Rotation2d lastHeading;
    public double lastCalled;
    public double headingOffsetDeg = 0;

    public PinpointOdometrySystem(HardwareMap hardwareMap, String i2cDeviceName) {
        // Initialize I2C device for Pinpoint

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, i2cDeviceName);
        pinpointDriver.setOffsets(-84.0, -168.0, headingOffsetDeg);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.recalibrateIMU();
        pinpointDriver.resetPosAndIMU();
//        pinpointDriver = new GoBildaPinpointDriver(pinpointDevice, true);

        // Initialize the odometry system
//        pinpointDriver.initialize();
        timer = new ElapsedTime();
    }

//    public void update() {
//        // Update odometry data from the Pinpoint
//        pinpointDriver.update();
//        xPos = pinpointDriver.getXPosMM() / 0.0393701; // Convert mm to inches
//        yPos = pinpointDriver.getYPosMM() / 0.0393701; // Convert mm to inches
//        heading = pinpointDriver.getHeadingDegrees();
//
//
//    }
public Twist2dDual<Time> update() {
    Pose2d pos = getPosition();
    Pose2D vel = pinpointDriver.getVelocity();
    Rotation2d heading = Rotation2d.exp(pos.heading.toDouble());
    if (!initialized){
        initialized = true;
        lastPosX = pos.position.x;
        lastPosY = pos.position.y;
        lastHeading = heading;
        lastCalled = System.currentTimeMillis();
        initialized = true;
        return new Twist2dDual<>(
                Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                DualNum.constant(0.0, 2)
        );
    }

    //Get the change in position in Field Centric X and Y
    double xDelta = pos.position.x - lastPosX ;
    double yDelta = pos.position.y - lastPosY;

    //Field centric --> Robot Centric Coordinates
    double traveledDist = Math.sqrt(Math.pow(xDelta,2) + Math.pow(yDelta,2));
    double localHeading = Math.atan2(yDelta, xDelta) - heading.toDouble()   ;
    xDelta =  traveledDist * Math.cos(localHeading);
    yDelta =  traveledDist * Math.sin(localHeading);;

    double headingDelta = heading.minus(lastHeading);

    //Calculate Velocities
    long timeNow = System.currentTimeMillis();
    double timeDeltaSecods = (timeNow - lastCalled)/1000.0;
    double xVel = vel.getX(DistanceUnit.INCH);
    double yVel = vel.getY(DistanceUnit.INCH);
    double headingVel = vel.getHeading(AngleUnit.DEGREES);



    Twist2dDual<Time> twist = new Twist2dDual<>(
            new Vector2dDual<>(
                    new DualNum<Time>(new double[] {
                            xDelta,
                            xVel,
                    }).times(1),
                    new DualNum<Time>(new double[] {
                            yDelta,
                            yVel,
                    }).times(1)
            ),
            new DualNum<>(new double[] {
                    headingDelta,
                    headingVel,
            })
    );


    lastPosX = pos.position.x;
    lastPosY = pos.position.y;
    lastHeading = heading;
    lastCalled = timeNow;

    return twist;
}

    public void resetPosition() {
        // Reset odometry position and IMU calibration
        pinpointDriver.resetPosAndIMU();
    }

    public double getXPosition() {
        return pinpointDriver.getXPosMM() / 25.4;
    }

    public double getYPosition() {
        return pinpointDriver.getYPosMM() / 25.4;
    }

    public double getHeading() {
        return pinpointDriver.getHeadingDegrees();
    }

    public Pose2d getPosition()
    {
        return new Pose2d(getXPosition(), -getYPosition(), getHeading());
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

