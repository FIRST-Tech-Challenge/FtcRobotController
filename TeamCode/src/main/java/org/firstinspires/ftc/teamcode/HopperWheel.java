package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class HopperWheel extends LinearOpMode {

    //init the two motors and distance sensor
    private DcMotor shootermtr = null;
    private DcMotor wheelmtr = null;
//    private Rev2mDistanceSensor dist = null;

    public HopperWheel (DcMotor l, DcMotor r) {
        shootermtr = l;
        wheelmtr = r;
        //direction for one is reversed so that
        //the collectors can suck bricks in and out
        shootermtr.setDirection(DcMotor.Direction.REVERSE);
        wheelmtr.setDirection(DcMotor.Direction.FORWARD);
    }

    public void out() {
        shootermtr.setPower(-1);
        wheelmtr.setPower(-1);
    }

    public void rest() {
        shootermtr.setPower(0);
        wheelmtr.setPower(0);
    }

    public String getPower() {
        double sp = shootermtr.getPower();
        double hp = wheelmtr.getPower();
        String s = sp + " / " + hp;
        return s;
    }

    public void runOpMode() {

    }
}
