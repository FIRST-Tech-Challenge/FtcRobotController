package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Drivechain {
    public Drivechain(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fl.resetDeviceConfigurationForOpMode();
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr = hardwareMap.get(DcMotor.class, "fr");
        fr.resetDeviceConfigurationForOpMode();
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl = hardwareMap.get(DcMotor.class, "bl");
        bl.resetDeviceConfigurationForOpMode();
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        br = hardwareMap.get(DcMotor.class, "br");
        br.resetDeviceConfigurationForOpMode();
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /*
   public static int FRONT_LEFT = 0;
   public static int BACK_RIGHT = ;
   public static int BACK_LEFT = 4;
   public static int FRONT_RIGHT = 5;
   public static int GRAB = 6;
   public static int ELEV = 0;
   public static int JEWEL = 1;
   private VisualController v;*/
    /*private static double DRIVE_SPEED = 0.4;
    private static double ROTATE_SPEED = 0.3;*/

    private DcMotor fl, fr, bl, br;


    /*

    double[] power = new double[4];
    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {

//        fl = hardwareMap.get(DcMotor.class, "fl");
//        fl.resetDeviceConfigurationForOpMode();
//        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        fr = hardwareMap.get(DcMotor.class, "fr");
//        fr.resetDeviceConfigurationForOpMode();
//        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        bl = hardwareMap.get(DcMotor.class, "bl");
//        bl.resetDeviceConfigurationForOpMode();
//        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        br = hardwareMap.get(DcMotor.class, "br");
//        br.resetDeviceConfigurationForOpMode();
//        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
//        leftMotor.resetDeviceConfigurationForOpMode();
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
//        rightMotor.resetDeviceConfigurationForOpMode();
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        reset();
    }

    public void shoot(){
        //     boolean powered = false;
        //     int buttonCounter = 0;
        //     int sliceyCounter = 0;
        //         if (sliceyCounter == 0){
        //             slicey.setPosition(0.9);
        //             sliceyCounter = 100;
        //         }
        //         else if (sliceyCounter == 0){

        //             slicey.setPosition(-1);
        //         }
        //         if(sliceyCounter > 0){
        //             sliceyCounter = sliceyCounter-1;
        //         }

        //     if (powered) {
        //         leftMotor.setPower(1.0);
        //         rightMotor.setPower(-1.0);
        //  }
//
//        leftMotor.setTargetPosition(100000);
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setPower(-0.8);
//
//        rightMotor.setTargetPosition(100000);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setPower(0.8);
//


    }

    public void stoprobot() {
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
    }

    public void resetTicks() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void moveRobot(Telemetry telemetry, int flposition, int frposition, int blposition, int brposition, double flpower, double frpower, double blpower, double brpower){

        fl.setTargetPosition(flposition);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setPower(flpower);

        fr.setTargetPosition(frposition);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setPower(frpower);

        bl.setTargetPosition(blposition);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setPower(blpower);

        br.setTargetPosition(brposition);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setPower(brpower);

        telemetry.addData("",fl.getCurrentPosition());
        telemetry.update();

        // fl.setPower(flpower);
        // fr.setPower(frpower);
        // bl.setPower(blpower);
        // br.setPower(brpower);
    }


/*
    public void lowerJArm() {
        move(JEWEL, -700, 0.2);
    }
    public void raiseJArm() {
        move(JEWEL, 700, -0.2);
    }*/

    public int getPos(DcMotor motor) {
        return motor.getCurrentPosition();
    }

    public boolean isbusy(){
        if (Math.abs(fl.getCurrentPosition()-fl.getTargetPosition()) <10 ){
            return false;
        }
        return true;
    }
    /*
        public void look() throws InterruptedException {
            v.look();
            int i;
            if (v.pictograph == null) {
                move(FRONT_LEFT, 50, 0.1);
                move(BACK_LEFT, -50, -0.1);
                move(BACK_RIGHT, 50, 0.1);
                move(FRONT_RIGHT, -50, -0.1);
                while(motors[FRONT_LEFT].isBusy()) {
                    v.look();
                    if (v.pictograph != null) {
                        break;
                    }
                }
                move(FRONT_LEFT, 0, 0);
                move(BACK_LEFT, 0, 0);
                move(BACK_RIGHT, 0, 0);
                move(FRONT_RIGHT, 0, 0);
                if (v.pictograph == null) {
                    move(FRONT_LEFT, -50, -0.1);
                    move(BACK_LEFT, 50, 0.1);
                    move(BACK_RIGHT, -50, -0.1);
                    move(FRONT_RIGHT, 50, 0.1);
                    while(motors[FRONT_LEFT].isBusy()) {
                        v.look();
                        if (v.pictograph != null) {
                            break;
                        }
                    }
                }
                move(FRONT_LEFT, 0, 0);
                move(BACK_LEFT, 0, 0);
                move(BACK_RIGHT, 0, 0);
                move(FRONT_RIGHT, 0, 0);
            }
            if (v.pictograph == null) {
                v.blindLook();
                v.pictograph = RelicRecoveryVuMark.CENTER;
            }
        }*/

//    public void runOpMode() throws InterruptedException {
//
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        // fl.resetDeviceConfigurationForOpMode();
//        // fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        fr = hardwareMap.get(DcMotor.class, "fr");
//        // fr.resetDeviceConfigurationForOpMode();
//        // fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        bl = hardwareMap.get(DcMotor.class, "bl");
//        // bl.resetDeviceConfigurationForOpMode();
//        // bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        br = hardwareMap.get(DcMotor.class, "br");
//        // br.resetDeviceConfigurationForOpMode();
//        // br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
//        // leftMotor.resetDeviceConfigurationForOpMode();
//        // leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
//        // rightMotor.resetDeviceConfigurationForOpMode();
//        // rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        // leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
//
//        // rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
//
//        waitForStart();
//
//        while (opModeIsActive()){
//            moveRobot(telemetry, 1000, -1000, 1000, -1000, 0.25, -0.25, 0.25, -0.25);
//
//            moveRobot(telemetry, 1000, -1000, 1000, -1000, -0.25, 0.25, -0.25, 0.25);
//
//
//            resetTicks();
//
//
//            // moveRobot(telemetry, 500, 500, 500, 500, 0.5, -0.5, 0.5, -0.5);
//            // resetTicks();
//
//
//            // moveRobot(telemetry, 500, 500, 500, 500, -0.25, -0.25, -0.25, -0.25);
//            // resetTicks();
//
//
//
//            // moveRobot(telemetry, -500, -500, -500, -500, -0.25, -0.25, -0.25, -0.25);
//            // resetTicks();
//
//
//            // moveRobot(telemetry, 2000, -2000, 2000, -2000, 0.5, -0.5, 0.5, -0.5);
//            // resetTicks();
//
//        }
//    }
    public void reset() {}



}