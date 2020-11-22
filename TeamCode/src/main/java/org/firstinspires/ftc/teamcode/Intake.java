package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class Intake extends LinearOpMode {

    //init the two motors and distance sensor
    private DcMotor topmtr = null;
    private DcMotor botmtr = null;
//    private Rev2mDistanceSensor dist = null;

    public Intake (DcMotor l, DcMotor r)
                   //Rev2m DistanceSensor d)
                   {
        topmtr = l;
        botmtr = r;
        //direction for one is reversed so that
        //the collectors can suck bricks in and out
        topmtr.setDirection(DcMotor.Direction.REVERSE);
        botmtr.setDirection(DcMotor.Direction.FORWARD);
       // dist = d;
    }

    public void in() {
        topmtr.setPower(1);
        botmtr.setPower(1);
    }

    public void out() {
        topmtr.setPower(-1);
        botmtr.setPower(-1);
    }

    public void rest() {
        topmtr.setPower(0);
        botmtr.setPower(0);
    }

    //  //the sensor reads up to 2 meters.
    //in gpsbrain, once the distance sensor is under 10sm, the motors shut off.
//    public double getDistance() {
//        double distance = dist.getDistance(DistanceUnit.CM);
//        return distance;
//    }

    public String getPower() {
        double tp = topmtr.getPower();
        double bp = botmtr.getPower();
        String s = tp + " / " + bp;
        return s;
    }

    public void runOpMode() {

    }
}
