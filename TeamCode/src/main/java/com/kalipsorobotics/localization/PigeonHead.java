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
        OdometrySpark odometrySpark = new OdometrySpark(myOtos, rightEncoder, backEncoder);
        double heading = odometrySpark.headingUpdateData("right", 0, 0);
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
//
//import Servo; // Import the servo library
//
//
//
//public class PigeonHead {
//
//
//
//    Servo servo; // Create a Servo object
//
//
//
//    public void setup() {
//
//        servo = new Servo(pinNumber); // Initialize servo on the specified pin
//
//    }
//
//
//
//    public void moveBeak(int position) { // Function to move the beak
//
//        servo.setPosition(position);
//
//    }
//
//
//
//    public static void main(String[] args) {
//
//        PigeonHead pigeon = new PigeonHead();
//
//        pigeon.setup();
//
//
//
//        // Example: Move beak to open position
//
//        pigeon.moveBeak(0.5);
//
//
//
//        // Add more code for different movements as needed
//
//    }
//
//}
