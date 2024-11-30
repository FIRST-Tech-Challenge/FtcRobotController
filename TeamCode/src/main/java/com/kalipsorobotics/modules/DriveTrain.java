package com.kalipsorobotics.modules;

import android.util.Log;

import com.kalipsorobotics.math.MathFunctions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.kalipsorobotics.utilities.OpModeUtilities;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveTrain {
    private final OpModeUtilities opModeUtilities;
    //private final DcMotor testMotorDeleteLater;
    private final DcMotor fLeft, fRight, bLeft, bRight;
    private final SparkFunOTOS otos;
    private final DcMotor backEncoder;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;

    public DriveTrain(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        fLeft = opModeUtilities.getHardwareMap().dcMotor.get("fLeft");
        fRight = opModeUtilities.getHardwareMap().dcMotor.get("fRight");
        bLeft = opModeUtilities.getHardwareMap().dcMotor.get("bLeft");
        bRight = opModeUtilities.getHardwareMap().dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        otos = opModeUtilities.getHardwareMap().get(SparkFunOTOS.class, "sprk sensor OTOS");
        sparkResetData(true, Math.toRadians(180));

        rightEncoder = bRight;
        leftEncoder = bLeft;
        backEncoder = fRight;

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // testMotorDeleteLater = opModeUtilities.getHardwareMap().dcMotor.get("testMotor");
    }



    public void setFLeftPower(double power) { fLeft.setPower(power); }

    public void setFRightPower(double power) { fRight.setPower(power); }

    public void setBLeftPower(double power) { bLeft.setPower(power); }

    public void setBRightPower(double power) { bRight.setPower(power); }

    public void setPower (double fLeftPower, double fRightPower, double bLeftPower, double bRightPower){
        setFLeftPower(fLeftPower);
        setFRightPower(fRightPower);
        setBLeftPower(bLeftPower);
        setBRightPower(bRightPower);
        Log.d("purepursaction_power", "power " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);
    }

    public void setPower(double power) {
        setFLeftPower(power);
        setFRightPower(power);
        setBLeftPower(power);
        setBRightPower(power);
    }
    public DcMotor getBackEncoder() {
        return backEncoder;
    }

    public DcMotor getRightEncoder() {
        return rightEncoder;
    }

    public DcMotor getLeftEncoder() {
        return leftEncoder;
    }

    public DcMotor getfLeft() {
        return fLeft;
    }

    public double getfLeftTicks() {
        return fLeft.getCurrentPosition();
    }

    public DcMotor getfRight() {
        return fRight;
    }

    public DcMotor getbLeft() {
        return bLeft;
    }

    public DcMotor getbRight() {
        return bRight;
    }

    public SparkFunOTOS getOtos() {
        return otos;
    }

    private void sparkResetData(Boolean reCalibrate, double heading) {
        otos.resetTracking();
        if (reCalibrate) { otos.calibrateImu(); }
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        double yOffset = (203-(42.9/2)) - 200;
        yOffset /= 25.4;
        otos.setOffset(new SparkFunOTOS.Pose2D(0, yOffset, heading));
        //8 1/2, 7 1/8
        //-0.72

        double angularScalar = (20*Math.PI) / (20*Math.PI + 0.418);
        angularScalar += (20*Math.PI) / (20*Math.PI + 0.415);
        angularScalar /= 2;

        otos.setAngularScalar(angularScalar); //angle is off by 0.02 radians after turning 10 rotations, 0.01 after turning 1


//        double linearScalar = 24/(0.5 * (25.76732634075329 + 25.554219026496753));
//        linearScalar += 24/(0.5 * (22.895127085813417 + 20.60628957172073));
//        linearScalar += 24/(0.5 * (21.696835597286775 + 20.874440832893924));
//        linearScalar += 24/(0.5 * (22.1072219488189 + 21.374319482037404));
//        linearScalar /= 4;
//


        double linearScalar = 24 / 22.232635906066438;
        linearScalar += -24 / -24.06409273550429;
        linearScalar += 24 / 24.100191556526028;
        linearScalar += -24 / -24.26461397512432;
        linearScalar += 96 / 97.53521848891732;

        linearScalar /= 5;

        otos.setLinearScalar(linearScalar);

        Log.d("sparkfun", "reset data");
        //7, 8
        //7, 8 1/2

        //7 1/2, 7 1/2

        //25.76732634075329 25.554219026496753
        //22.895127085813417 20.60628957172073
        //21.696835597286775 20.874440832893924
        //22.1072219488189  21.374319482037404 0.0022
        //24 24
    }

    /*public DcMotor getTestMotorDeleteLater() {
        return testMotorDeleteLater;
    }*/

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }


    public void setPowerWithRangeClippingMinThreshold(double fLeftPower, double fRightPower, double bLeftPower,
                                                      double bRightPower, double minPowerThreshold) {
        double biggestPowerAbs = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
        double scaleFactor = 1;

        //clipping power to -1 and 1 and min power is not 0.2 and -0.2
        if (biggestPowerAbs > 1) {
            scaleFactor = 1 / biggestPowerAbs;
        } else if (biggestPowerAbs < minPowerThreshold) {
            scaleFactor = (minPowerThreshold / biggestPowerAbs);
        }

        fLeftPower *= scaleFactor;
        fRightPower *= scaleFactor;
        bLeftPower *= scaleFactor;
        bRightPower *= scaleFactor;
        setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);
    }

    /*public void setTestPower(double power) throws InterruptedException {
        getTestMotorDeleteLater().setPower(power);
        Thread.sleep(3000);
        getTestMotorDeleteLater().setPower(0);
    }*/
}
