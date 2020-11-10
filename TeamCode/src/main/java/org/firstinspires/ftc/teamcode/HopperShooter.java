//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//
//public class HopperShooter extends LinearOpMode {
//
//    //init the two motors and distance sensor
//    private DcMotor shootermtr = null;
//    private DcMotor hoppermtr = null;
////    private Rev2mDistanceSensor dist = null;
//
//    public HopperShooter (DcMotor l, DcMotor r) {
//        shootermtr = l;
//        hoppermtr = r;
//        //direction for one is reversed so that
//        //the collectors can suck bricks in and out
//        shootermtr.setDirection(DcMotor.Direction.REVERSE);
//        hoppermtr.setDirection(DcMotor.Direction.FORWARD);
//    }
//
//    public void out() {
//        shootermtr.setPower(-1);
//        hoppermtr.setPower(-1);
//    }
//
//    public void rest() {
//        shootermtr.setPower(0);
//        hoppermtr.setPower(0);
//    }
//
//    public String getPower() {
//        double sp = shootermtr.getPower();
//        double hp = hoppermtr.getPower();
//        String s = sp + " / " + hp;
//        return s;
//    }
//
//    public void runOpMode() {
//
//    }
//}
