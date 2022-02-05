package org.firstinspires.ftc.teamcode.Chassis;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;



import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Drivechain {

    DcMotorEx fl, fr, bl, br;
    public Drivechain(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        // fl.resetDeviceConfigurationForOpMode();
        // fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        fr = hardwareMap.get(DcMotorEx.class, "fr");
        // fr.resetDeviceConfigurationForOpMode();
        // fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        bl = hardwareMap.get(DcMotorEx.class, "bl");
        // bl.resetDeviceConfigurationForOpMode();
        // bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        br = hardwareMap.get(DcMotorEx.class, "br");
        // br.resetDeviceConfigurationForOpMode();
        // br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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




    /*

    double[] power = new double[4];
    fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
*/

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {

//        fl = hardwareMap.get(DcMotorEx.class, "fl");
//        fl.resetDeviceConfigurationForOpMode();
//        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        fr = hardwareMap.get(DcMotorEx.class, "fr");
//        fr.resetDeviceConfigurationForOpMode();
//        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        bl = hardwareMap.get(DcMotorEx.class, "bl");
//        bl.resetDeviceConfigurationForOpMode();
//        bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        br = hardwareMap.get(DcMotorEx.class, "br");
//        br.resetDeviceConfigurationForOpMode();
//        br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
//        leftMotor.resetDeviceConfigurationForOpMode();
//        leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
//        rightMotor.resetDeviceConfigurationForOpMode();
//        rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
//        leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        leftMotor.setPower(-0.8);
//
//        rightMotor.setTargetPosition(100000);
//        rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void switchToPower(){
        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runRobot(int flpower, int frpower, int blpower, int brpower){
        fl.setPower(flpower);
        fr.setPower(frpower);
        bl.setPower(blpower);
        br.setPower(brpower);
    }


    public void runRobot(double v, double v1, double v2, double v3){
        fl.setPower(0.5);
        // fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setPower(0.5);
        // fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setPower(0.5);
        // bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setPower(0.5);
    }



    public void showTicks(Telemetry telemetry){
        telemetry.addData("fl",fl.getCurrentPosition());
        telemetry.addData("fr",fr.getCurrentPosition());
        telemetry.addData("bl",bl.getCurrentPosition());
        telemetry.addData("br",br.getCurrentPosition());
        telemetry.update();
    }

    public void moveRobot(int flposition, int frposition, int blposition, int brposition, double flpower, double frpower, double blpower, double brpower){

        fl.setTargetPosition(flposition);
        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fl.setPower(flpower);

        fr.setTargetPosition(frposition);
        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setPower(frpower);

        bl.setTargetPosition(blposition);
        bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setPower(blpower);

        br.setTargetPosition(brposition);
        br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setPower(brpower);


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

    public int getPos(DcMotorEx motor) {
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
//        fl = hardwareMap.get(DcMotorEx.class, "fl");
//        // fl.resetDeviceConfigurationForOpMode();
//        // fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        // fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        fr = hardwareMap.get(DcMotorEx.class, "fr");
//        // fr.resetDeviceConfigurationForOpMode();
//        // fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        // fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        bl = hardwareMap.get(DcMotorEx.class, "bl");
//        // bl.resetDeviceConfigurationForOpMode();
//        // bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        // bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        br = hardwareMap.get(DcMotorEx.class, "br");
//        // br.resetDeviceConfigurationForOpMode();
//        // br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        // br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        // leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
//        // leftMotor.resetDeviceConfigurationForOpMode();
//        // leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        // leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        // rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
//        // rightMotor.resetDeviceConfigurationForOpMode();
//        // rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        // rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//
//        // leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
//
//        // rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
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