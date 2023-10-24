/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.configPID.*;
import static java.lang.Math.abs;
import static java.lang.Math.addExact;
import static java.lang.Math.floorDiv;
import static java.lang.Thread.sleep;

import android.util.Log;
import android.widget.Switch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class centerstageRX extends OpMode {
    public Switch swish;
    public DcMotorEx motorBR,motorBL,motorFL,motorFR,slider,melcsus,melcjos;
    public Servo gherutaL,gherutaR,plauncher;
    public CRServo maceta;
    double sm = 1, slow = 1, lb = 1, rb = 1,bval=0,xval=0,rt=0,rtval=0,slowl=1,slowr=1,distL,distR,pidResult;
    boolean dpd,dpu,enter = false,bl,xl,aprins1 = false,aprins2 = false;
    double y, x, rx, ghearaPoz=0.5, macetaPow=0;
    double max = 0,lastTime;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    ChestiiDeAutonom c = new ChestiiDeAutonom();
    boolean stop = false, lastx = false, lasty = false, sliderState = true, aIntrat = false,aAjuns = true,aInchis = true,setSetpoint;
    double intPoz = 0.4, servoPos = 0.0;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(kp,ki,kd);
    AnalogInput potentiometru;
    DistanceSensor distanceL,distanceR;
    private static final int DESIRED_TAG_ID = 1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean choco_senzor_parcare = false;
    int ret;
    /*Functia de init se ruleaza numai o data, se folosete pentru initializarea motoarelor si chestii :)*/
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        potentiometru = hardwareMap.get(AnalogInput.class,"potentiometru");

        distanceL = hardwareMap.get(DistanceSensor.class,"distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class,"distanceR");

        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Front-Right

        slider = hardwareMap.get(DcMotorEx.class,"slider");
        melcsus = hardwareMap.get(DcMotorEx.class,"melcsus");
        melcjos = hardwareMap.get(DcMotorEx.class,"melcjos");

        gherutaL = hardwareMap.get(Servo.class,"gherutaL");
        gherutaR = hardwareMap.get(Servo.class,"gherutaR");
        maceta = hardwareMap.get(CRServo.class,"maceta");
        plauncher = hardwareMap.get(Servo.class,"plauncher");

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        slider.setDirection(DcMotorEx.Direction.REVERSE);
        melcsus.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        melcsus.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        melcjos.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        melcjos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        melcsus.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slider.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        melcjos.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        melcsus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid.enable();
    }
    public void start(){
        Chassis.start();
        Systems.start();
        PID.start();
    }
    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!stop && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                c.kdf(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!stop) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                c.kdf(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            c.kdf(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            c.kdf(20);
        }
    }
    public int AprilTagInit2(){

        // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    Log.wtf("ok","e ok");
                    //  Check to see if we want to track towards this tag.
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        telemetry.addData("April tag detection id:",detection.corners);
                        Log.wtf("am ajuns", "ok");
                        return 1;

                }
                else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }

            }
        return  0;
    }
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while(!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y = gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;

                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, cu ajutorul puterilor de la controller*/
                pmotorFL = -y + x + rx;
                pmotorBL = -y - x + rx;
                pmotorBR = -y + x - rx;
                pmotorFR = -y - x - rx;

                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                /*...care este punerea tuturor puterilor motoarelor sub 1, cum puterile de la motoare pot fi numai intre 1 si -1*/
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                if(gamepad1.x != lastx){
                    rb += 0.5;
                    if(rb > 2){
                        rb = 0.5;
                    }
                }
                if(gamepad1.y != lasty){
                    lb += 0.5;
                    if(lb > 2){
                        lb = 0.5;
                    }
                }
                if(rb == 2){
                    sm = 4;
                }
                else if(lb == 2){
                    sm = 2;
                }
                else{
                    sm = 1;
                }
                lastx = gamepad1.x;
                lasty = gamepad1.y;
                /*ret = AprilTagInit2();
                if(ret==1) {
                    distL = distanceL.getDistance(DistanceUnit.CM);
                    distR = distanceR.getDistance(DistanceUnit.CM);
                    if (distL < 26) {
                        slowl = 4;
                        slowr = 4;
                    }
                }
                else{
                    choco_senzor_parcare = false;
                    slowl = 1;
                    slowr = 1;
                }*/
                POWER(pmotorFR / sm / slowr, pmotorFL / sm / slowl, pmotorBR / sm / slowr, pmotorBL / sm / slowl);
            }
        }
    });
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);;
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
    private final Thread PID = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!stop){
                //pid.setPID(kp,ki,kd);
                if (gamepad2.left_trigger > 0) {
                    slow = 2;
                }
                else {
                    slow = 1;
                }
                if(!aIntrat) {
                    slider.setPower(gamepad2.right_stick_y);
                }
                //if(gamepad2.left_stick_y != 0){
                if(!gamepad2.right_bumper) {
                    melcjos.setPower(gamepad2.left_stick_y / slow);
                    melcsus.setPower(gamepad2.left_stick_y / slow);
                    setSetpoint = true;
                    aIntrat = false;
                }
                else{
                    aIntrat = true;
                    c.melctarget(1.67,1500,10000);
                    c.target(0,2000,slider,2000,1);
                }
//                }}
//                else{
//                    if(setSetpoint){
//                        pid.setSetpoint(melcjos.getCurrentPosition());
//                        setSetpoint = false;
//                    }
//                    pidResult = pid.performPID(melcjos.getCurrentPosition());
//                    melcjos.setPower(pidResult);
//                    melcsus.setPower(pidResult);
//                }
            }
        }
    });
    /*Aici se declara thread-ul cu numele systems, pentru ca contine partea de program care se ocupa de sisteme*/
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (gamepad2.dpad_down) {
                    maceta.setPower(1);
                }
                if (gamepad2.dpad_up) {
                    maceta.setPower(-1);
                }

                if (gamepad2.left_trigger > 0) {
                    plauncher.setPosition(0.5);
                } else {
                    plauncher.setPosition(0.35);
                }

                if (gamepad2.b != bl) {
                    bval += 0.5;
                    if (bval >= 1) {
                        bval = 0;
                    }
                }
                bl = gamepad2.b;
                if (bval == 0.5 && gamepad2.b) {
                    bval += 0.0001;
                    gherutaR.setPosition(0.15);
                    gherutaL.setPosition(0.63);
                    lastTime = System.currentTimeMillis();
                    while (lastTime + 150 > System.currentTimeMillis()) {
                    }
                    gherutaL.setPosition(0.38);
                    gherutaR.setPosition(0.38);
                }
                if (potentiometru.getVoltage() > 1.65) {
                    gherutaR.setPosition(0.38);
                    gherutaL.setPosition(0.38);
                }
                else {
                    if (gamepad2.a) {
                        gherutaR.setPosition(0.15);
                        gherutaL.setPosition(0.63);
                    }
                    if (gamepad2.y) {
                        gherutaR.setPosition(0.38);
                        gherutaL.setPosition(0.38);
                    }
                }

            }
        }
    });
    /*Aici se afla partea de program care arata cand programul se opreste, este foarte folositor pentru functionarea thread-urilor*/
    public void stop(){stop = true;}

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override

    public void loop() {
        telemetry.addData("motorBL", motorBL.getCurrentPosition());
        telemetry.addData("motorFL", motorFL.getCurrentPosition());
        telemetry.addData("motorBR", motorBR.getCurrentPosition());
        telemetry.addData("motorFR", motorFR.getCurrentPosition());
        telemetry.addData("slider:", slider.getCurrentPosition());
        telemetry.addData("melcjos:", melcjos.getCurrentPosition());
        telemetry.addData("melcsus:", melcsus.getCurrentPosition());
        telemetry.addData("ghearaPoz:", ghearaPoz);
        telemetry.addData("macetaPow:", macetaPow);
        telemetry.addData("bval:", bval);
        telemetry.addData("xval:", xval);
        telemetry.addData("bl:", bl);
        telemetry.addData("gamepad2.b:", gamepad2.b);
        telemetry.addData("potentiometru:",potentiometru.getVoltage());
        telemetry.addData("distanceL:",distanceL.getDistance(DistanceUnit.CM));
        telemetry.addData("distanceR:",distanceR.getDistance(DistanceUnit.CM));
        telemetry.addData("setpoint:",pid.getSetpoint());
        telemetry.addData("error:",pid.getError());
        telemetry.addData("pidResult:",pidResult);
        telemetry.addData("rtval:",rtval);
        telemetry.update();
    }

}