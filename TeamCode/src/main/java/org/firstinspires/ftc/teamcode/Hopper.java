package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class Hopper extends LinearOpMode {

    //init the two motors and distance sensor
    private DcMotor wheelmotor = null;
    private Servo platformservo = null;

    public Hopper (DcMotor m, Servo s) {
        wheelmotor = m;
        //servos move opposite
        platformservo = s;
    }

    public void out() { // TODO: find speeds
        wheelmotor.setPower(1);
    }

    public void rest() {
        wheelmotor.setPower(0);
    }

    public double getPower() {
        return wheelmotor.getPower();
    }

    public void moveplatform(double goal) { //move to given pos
        platformservo.setPosition(goal);
    }

    public void incrementplatform() {
        double x = platformservo.getPosition();
        platformservo.setPosition(x + 0.1); //TODO: find what 0.1 should actually be
    }

    public void decrementplatform() {
        double x = platformservo.getPosition();
        platformservo.setPosition(x - 0.1); //TODO: find what 0.1 should actually be
    }

    public double getPlatformPos() {
        return platformservo.getPosition();
    }

    public void runOpMode() {

    }
}
