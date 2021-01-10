package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class Shooter extends LinearOpMode {

    //init the two motors and distance sensor
    private DcMotor Lmtr = null;
    private DcMotor Rmtr = null;
    private Servo leftservo = null;
    private Servo rightservo = null;
    private DigitalChannel touch = null;

    public Shooter (DcMotor l, DcMotor r, Servo sl, Servo sr, DigitalChannel t) {
        Lmtr = l;
        Rmtr = r;
        //direction for one is reversed so that
        Lmtr.setDirection(DcMotor.Direction.REVERSE); // TODO: try to find the built in PID stuff
        Rmtr.setDirection(DcMotor.Direction.FORWARD); // TODO: runWithEncoders or something
        //servos move opposite
        leftservo = sl;
        rightservo = sr;
        //touch sensor
        touch = t;
    }

    public void out() { // TODO: find speeds and find which motor should be reverse
        Lmtr.setPower(-1);
        Rmtr.setPower(-1);
    }

    public void rest() {
        Lmtr.setPower(0);
        Rmtr.setPower(0);
    }

    public String getPower() {
        double lp = Lmtr.getPower();
        double rp = Rmtr.getPower();
        String s = lp + " / " + rp;
        return s;
    }

    public void pivot(double goal) { // if goal = .1
        leftservo.setPosition(goal); // servo goes to .1
        rightservo.setPosition(Math.abs(goal - 1)); // servo goes to .9
    }

    public boolean getTouch() {
        return touch.getState();
    }

    public void runOpMode() {

    }
}
