package com.kalipsorobotics.localization;

import android.annotation.SuppressLint;
import android.content.Context;

import com.kalipsorobotics.math.Point;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

public class OdometrySpark {
    OpModeUtilities opModeUtilities;
    private final SparkFunOTOS myOtos;
    private final DcMotor rightEncoder;
    private final DcMotor backEncoder;

    public OdometrySpark(SparkFunOTOS myOtos, DcMotor rightEncoder, DcMotor backEncoder) {
        this.myOtos = myOtos;
        this.rightEncoder = rightEncoder;
        this.backEncoder = backEncoder;
        wheelResetData();
        sparkResetData(true, 0.0);
    }
    public void fileDataUpdate(HardwareMap hardwareMap) {
        hardwareMap.appContext.deleteFile("odometry Data");
        FileOutputStream outputStream = null;
        try {
            outputStream = hardwareMap.appContext.openFileOutput("odometry Data", Context.MODE_PRIVATE);
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
        ObjectOutputStream objectOutputStream = null;
        try {
            objectOutputStream = new ObjectOutputStream(outputStream);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            objectOutputStream.writeObject(sparkUpdateData().getX());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            objectOutputStream.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            objectOutputStream.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            outputStream.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    public Point wheelUpdateData() {
        double TICKSTOINCH = 1 ;//40 / -13510.0 * (40.0 / 40.3612);
        return(new Point(backEncoder.getCurrentPosition() * TICKSTOINCH, rightEncoder.getCurrentPosition() * TICKSTOINCH));
    }
    public Point sparkUpdateData() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return(new Point(-pos.x, -pos.y));
    }
    public Point averageUpdateData() {
        double TICKSTOINCH = 40 / -13510.0 * (40.0 / 40.3612);
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return(new Point(((rightEncoder.getCurrentPosition() * TICKSTOINCH) + pos.x) / 2, ((backEncoder.getCurrentPosition() * TICKSTOINCH) + pos.y) / 2));
    }

    public double headingUpdateData(String direction, double xOffSet, double yOffset) {
        if (direction.equals("right")) {
            SparkFunOTOS.Pose2D offsets = new SparkFunOTOS.Pose2D(xOffSet, yOffset, 0);
            myOtos.setOffset(offsets);
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            return(-pos.h); }
        else if (direction.equals("left")) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            return(pos.h); }
        else { return(0.0); }
    }

    public Point filter(Point sparkPoint, Point wheelPoint) {
        int diffenceDebug = 1;
        if ((sparkPoint.getX() - wheelPoint.getX() < diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < diffenceDebug) && (sparkPoint.getX() - wheelPoint.getX() > -diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < -diffenceDebug)) { return(wheelUpdateData()); }
        else { return(sparkUpdateData()); }
    }
    public void sparkResetData(Boolean reCalibrate, double heading) {
        myOtos.resetTracking();
        if (reCalibrate) { myOtos.calibrateImu(); }
        myOtos.setOffset(new SparkFunOTOS.Pose2D(wheelUpdateData().getX(), wheelUpdateData().getY(), heading));
    }

    public void wheelResetData() {
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean autoTurn(Double degree, String direction) {
        final double offset = headingUpdateData(direction, 0, 0);
        while (true) {
            if (headingUpdateData(direction, 0, 0) - offset > degree) {
                break;
            }
        }
        return(true);
    }

    public boolean autoForward(double inches, String XorY) {
        final double offset;
        if (XorY.equalsIgnoreCase("x")) {
            offset = pointCollectData().getX();
            while (true) {
                if (pointCollectData().getX() - offset > inches) {
                    return true;
                }
            }
        } else if (XorY.equalsIgnoreCase("y")) {
            offset = pointCollectData().getY();
            while (true) {
                if (pointCollectData().getY() - offset > inches) {
                    return true;
                }
            }
        } else { return false; }
    }

    public boolean autoMecanum(double x, double y) {
        final Point offset = pointCollectData();
        while (true) {
            if (pointCollectData().getY() - offset.getY() > y &&
                    pointCollectData().getX() - offset.getX() > x) return true;
        }
    }



    public Point pointCollectData() {
        return(filter(sparkUpdateData(), wheelUpdateData()));
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