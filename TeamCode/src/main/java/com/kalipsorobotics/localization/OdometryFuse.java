package com.kalipsorobotics.localization;

import android.annotation.SuppressLint;

import com.kalipsorobotics.math.Point;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOmniOpMode_Linear;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.kalipsorobotics.utilities.OpModeUtilities;

import java.util.Collection;

public class OdometryFuse {
    OpModeUtilities opModeUtilities;
    private final SparkFunOTOS myOtos;
    private final DcMotor rightEncoder;
    private final DcMotor backEncoder;

    public OdometryFuse(SparkFunOTOS myOtos, DcMotor rightEncoder, DcMotor backEncoder) {
        this.myOtos = myOtos;
        this.rightEncoder = rightEncoder;
        this.backEncoder = backEncoder;
    }
    public Point WheelUpdateData() {
        double TICKSTOINCH = 40 / -13510.0 * (40.0 / 40.3612);
        return(new Point(backEncoder.getCurrentPosition() * TICKSTOINCH, rightEncoder.getCurrentPosition() * TICKSTOINCH));
    }
    public Point SparkUpdateData() {
        SparkFunOTOS.Pose2D SparkFunOTOS;
        com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return(new Point(-pos.x, -pos.y));
    }
    public Point AverageUpdateData() {
        double TICKSTOINCH = 40 / -13510.0 * (40.0 / 40.3612);
        com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return(new Point(((rightEncoder.getCurrentPosition() * TICKSTOINCH) + pos.x) / 2, ((backEncoder.getCurrentPosition() * TICKSTOINCH) + pos.y) / 2));
    }

    public double HeadingUpdateData(String direction) {
        if (direction.equals("right")) {
            SparkFunOTOS.Pose2D SparkFunOTOS;
            com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            return(-pos.h); }
        else if (direction.equals("left")) {
            SparkFunOTOS.Pose2D SparkFunOTOS;
            com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            return(pos.h); }
        else { return(0.0); }
    }

    public Point Filter(Point sparkPoint, Point wheelPoint) {
        int diffenceDebug = 1;
        if ((sparkPoint.getX() - wheelPoint.getX() < diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < diffenceDebug) && (sparkPoint.getX() - wheelPoint.getX() > -diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < -diffenceDebug)) { return(WheelUpdateData()); }
        else { return(SparkUpdateData()); }
    }
    public void SparkResetData(Boolean reCalibrate, double heading) {
        myOtos.resetTracking();
        if (reCalibrate) { myOtos.calibrateImu(); }
        myOtos.setOffset(new SparkFunOTOS.Pose2D(WheelUpdateData().getX(), WheelUpdateData().getY(), heading));
    }
    public void wheelResetData() {
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean autoTurn(Double degree, String direction) {
        final double offset = HeadingUpdateData(direction);
        while (true) {
            if (HeadingUpdateData(direction) - offset > degree) {
                break;
            }
        }
        return(true);
    }

    public boolean autoForward(double inches, String XorY) {
        final double offset;
        if (XorY.equalsIgnoreCase("x")) {
            offset = PointCollectData().getX();
            while (true) {
                if (PointCollectData().getX() - offset > inches) {
                    return true;
                }
            }
        } else if (XorY.equalsIgnoreCase("y")) {
            offset = PointCollectData().getY();
            while (true) {
                if (PointCollectData().getY() - offset > inches) {
                    return true;
                }
            }
        } else { return false; }
    }

    public boolean autoMecanum(double x, double y) {
        final Point offset = PointCollectData();
        while (true) {
            if (PointCollectData().getY() - offset.getY() > y &&
                    PointCollectData().getX() - offset.getX() > x) return true;
        }
    }



    public Point PointCollectData() {
        return(Filter(SparkUpdateData(), WheelUpdateData()));
    }

    @SuppressLint("DefaultLocale")
    public String configureOtos(SparkFunOTOS myOtos) {

        myOtos.setLinearUnit(DistanceUnit.INCH);

        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1/1.165);


        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

            // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        return("OTOS configured! \n Hardware version: " + hwVersion.major + hwVersion.minor + "\n" +
                "Firmware Version: " + fwVersion.major + fwVersion.minor);
        }
}