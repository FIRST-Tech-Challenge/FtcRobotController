package com.hfrobots.tnt.season1819;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Collector {


    private double collectorServoDeployedPosition = 3;

    private double collectorServoStowedPosition = 20;

    private double collectionPower = .5;

    private double rejectionPower = -.5;

    private Servo stowServo;

    private Servo deployServo;

    private DcMotor collectorMotor;



    public void stowCollector() {
        stowServo.setPosition(collectorServoStowedPosition);
    }

    public void deployCollector() {
        deployServo.setPosition(collectorServoDeployedPosition);

    }

    public void motorCollectionMode() {
        collectorMotor.setPower(collectionPower);
    }

    public void motorRejectionMode(){
        collectorMotor.setPower(rejectionPower);
    }

}
