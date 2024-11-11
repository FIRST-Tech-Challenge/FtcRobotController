/*
package com.kalipsorobotics.localization; import android.annotation.SuppressLint; import com.kalipsorobotics.math.Point; import com.qualcomm.hardware.sparkfun.SparkFunOTOS; import com.qualcomm.robotcore.hardware.DcMotor; import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; import com.kalipsorobotics.utilities.OpModeUtilities; public class OdometryFuse { OpModeUtilities opModeUtilities; private final SparkFunOTOS myOtos; private final DcMotor rightEncoder; private final DcMotor backEncoder; public OdometryFuse(SparkFunOTOS myOtos, DcMotor rightEncoder, DcMotor leftEncoder, DcMotor backEncoder) { this.myOtos = myOtos; this.rightEncoder = rightEncoder; this.backEncoder = backEncoder; } public Point WheelUpdateData() { double TICKSTOINCH = 40 / -13510.0 * (40.0 / 40.3612); return(new Point(backEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition())); } public Point SparkUpdateData() { SparkFunOTOS.Pose2D SparkFunOTOS; com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition(); return(new Point(pos.x, pos.y)); } public Point AverageUpdateData() { SparkFunOTOS.Pose2D SparkFunOTOS; double TICKSTOINCH = 40 / -13510.0 * (40.0 / 40.3612); com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition(); return(new Point(((rightEncoder.getCurrentPosition() * TICKSTOINCH) + pos.x) / 2, ((backEncoder.getCurrentPosition() * TICKSTOINCH) + pos.y) / 2)); } public Point Filter(Point sparkPoint, Point wheelPoint) { int diffenceDebug = 2; if ((sparkPoint.getX() - wheelPoint.getX() < diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < diffenceDebug) && (sparkPoint.getX() - wheelPoint.getX() > -diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < -diffenceDebug)) { return(WheelUpdateData()); } else { return(SparkUpdateData()); } } public Void ResetData(Boolean reCalibrate) { myOtos.resetTracking(); if (reCalibrate) { myOtos.calibrateImu(); } return null; } public Point CollectData() { Point point = Filter(SparkUpdateData(), WheelUpdateData()); return(point); } @SuppressLint("DefaultLocale") public String configureOtos(SparkFunOTOS myOtos) { myOtos.setLinearUnit(DistanceUnit.INCH); myOtos.setAngularUnit(AngleUnit.DEGREES); SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0); myOtos.setOffset(offset); myOtos.calibrateImu(); myOtos.resetTracking(); SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0); myOtos.setPosition(currentPosition); SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version(); SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version(); myOtos.getVersionInfo(hwVersion, fwVersion); return("OTOS configured! \n Hardware version: " + hwVersion.major + hwVersion.minor + "\n" + "Firmware Version: " + fwVersion.major + fwVersion.minor); } }
*/

package com.kalipsorobotics.localization;

import android.annotation.SuppressLint;

import com.kalipsorobotics.math.Point;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.kalipsorobotics.utilities.OpModeUtilities;

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
        return(new Point(backEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition()));
    }
    public Point SparkUpdateData() {
        SparkFunOTOS.Pose2D SparkFunOTOS;
        com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return(new Point(pos.x, pos.y));
    }
    public Point AverageUpdateData() {
        double TICKSTOINCH = 40 / -13510.0 * (40.0 / 40.3612);
        com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return(new Point(((rightEncoder.getCurrentPosition() * TICKSTOINCH) + pos.x) / 2, ((backEncoder.getCurrentPosition() * TICKSTOINCH) + pos.y) / 2));
    }

    public double HeadingUpdateData() {
        SparkFunOTOS.Pose2D SparkFunOTOS;
        com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return(pos.h);
    }

    public Point Filter(Point sparkPoint, Point wheelPoint) {
        int diffenceDebug = 2;
        if ((sparkPoint.getX() - wheelPoint.getX() < diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < diffenceDebug) && (sparkPoint.getX() - wheelPoint.getX() > -diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < -diffenceDebug)) { ResetData(Boolean.FALSE, 0); return(WheelUpdateData()); }
        else { return(SparkUpdateData()); }
    }
    public void ResetData(Boolean reCalibrate, double heading) {
        myOtos.resetTracking();
        if (reCalibrate) { myOtos.calibrateImu(); }
        myOtos.setOffset(new SparkFunOTOS.Pose2D(WheelUpdateData().getX(), WheelUpdateData().getY(), heading));
    }

    public Point CollectData() {
        Point point = Filter(SparkUpdateData(), WheelUpdateData());
        return(point);
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