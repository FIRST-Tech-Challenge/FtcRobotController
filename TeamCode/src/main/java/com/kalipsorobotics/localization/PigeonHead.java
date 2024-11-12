package com.kalipsorobotics.localization;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class PigeonHead {
    OpModeUtilities opModeUtilities;
    private final Servo servo;
    private final SparkFunOTOS myOtos;
    private final DcMotor rightEncoder;
    private final DcMotor backEncoder;

    public PigeonHead(Servo servo, SparkFunOTOS myOtos, DcMotor rightEncoder, DcMotor backEncoder) {
        this.servo = servo;
        this.myOtos = myOtos;
        this.rightEncoder = rightEncoder;
        this.backEncoder = backEncoder;
    }

    public void prepare() {
        myOtos.resetTracking();
        myOtos.calibrateImu();
    }

    public void positionUpdate() {
        //might have to be fixed
        OdometryFuse odometryFuse = new OdometryFuse(myOtos, rightEncoder, backEncoder);
        double heading = odometryFuse.HeadingUpdateData("right");
        servo.setPosition(-heading);
    }

    public double getPigeonHeadPos() {
        return(servo.getPosition());
    }

    public void setPigeonHeadPos(double position) {
        servo.setPosition(position);
    }
    public void resetPigeonHeadPos() {
        servo.setPosition(0);
    }
}
