package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;

import static java.lang.Math.abs;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class ChestiiDeAutonom extends LinearOpMode {
    public HardwareMap hwmap = null;
    public OpenCvCamera webcam;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    public DcMotorEx melcsus, melcjos, slider, motorBL,motorBR,motorFL,motorFR;
    public Servo ghearaR, ghearaL, plauncher;
    public Boolean startThread;
    public TouchSensor sliderTouch;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public CRServo maceta,extensorL,extensorR;
    AnalogInput potentiometru;
    ChestiiDeAutonom() {}
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
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    public void initTaguriAprilie(){
        initAprilTag();
        setManualExposure(6, 250);
    }
    public void detectieTaguriAprilie(int DESIRED_TAG_ID){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null){
                if(detection.id == DESIRED_TAG_ID) {
                    telemetry.addData("April tag detection corners:", detection.corners);
                    telemetry.update();
                    break;
                }
            }
            else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
    }
    public void initSasiu(HardwareMap hard){
        motorBL = hard.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hard.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hard.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hard.get(DcMotorEx.class, "motorFR"); // Motor Back-Left

        motorBR.setDirection(DcMotorEx.Direction.REVERSE);
        motorFR.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void init(HardwareMap hard) {
        potentiometru = hard.get(AnalogInput.class,"potentiometru");

        melcjos = hard.get(DcMotorEx.class, "melcjos");
        melcsus = hard.get(DcMotorEx.class, "melcsus");
        slider = hard.get(DcMotorEx.class, "slider");

        ghearaL = hard.get(Servo.class, "gherutaL");
        ghearaR = hard.get(Servo.class, "gherutaR");
        plauncher = hard.get(Servo.class,"plauncher");

        maceta = hard.get(CRServo.class,"maceta");
        extensorL = hard.get(CRServo.class, "extensorL");
        extensorR = hard.get(CRServo.class, "extensorR");

        slider.setDirection(DcMotorEx.Direction.REVERSE);
        melcsus.setDirection(DcMotorEx.Direction.REVERSE);
        extensorL.setDirection(CRServo.Direction.REVERSE);
        
        melcjos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        melcsus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        melcjos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        melcsus.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        melcjos.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        melcsus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void deschidere(){
        ghearaR.setPosition(0.15);
        ghearaL.setPosition(0.63);
    }
    public void inchidere(){
        ghearaR.setPosition(0.38);
        ghearaL.setPosition(0.38);
    }
    public void target(double poz, double vel, DcMotorEx motor, double t, int tolerance) {
        if(motor.getCurrentPosition() < poz){
            motor.setVelocity(vel);
        }
        else {
            motor.setVelocity(-vel);
        }
        double lastTime = System.currentTimeMillis();
        while (!isStopRequested()
                && lastTime + t > System.currentTimeMillis()
                && (abs(motor.getCurrentPosition() - poz) > tolerance)) {
        }
        motor.setVelocity(0);
    }
    public void extensorPower(double pow, int t){
        extensorR.setPower(pow);
        extensorL.setPower(pow);
        long lastTime = System.currentTimeMillis();
        while(lastTime + t > System.currentTimeMillis()){}
        extensorL.setPower(0);
        extensorR.setPower(0);
    }
    public void slidertarget(int poz, double vel, double t, int tolerance) {
        Log.wtf("melcjos target:", Integer.toString(-poz));
        Log.wtf("melcsus target:", Integer.toString(poz));

        if(melcsus.getCurrentPosition() < poz){
            melcjos.setVelocity(-abs(vel));
            melcsus.setVelocity(abs(vel));
        }
        else{
            melcjos.setVelocity(abs(vel));
            melcsus.setVelocity(-abs(vel));
        }

        double lastTime = System.currentTimeMillis();
        Log.wtf("melcjos busy:",Double.toString(melcjos.getVelocity()));
        Log.wtf("melcsus busy:",Double.toString(melcsus.getVelocity()));
        while (abs(poz - melcsus.getCurrentPosition()) > tolerance
                && abs(poz - melcjos.getCurrentPosition()) > tolerance
                && !isStopRequested()
                && lastTime + t > System.currentTimeMillis()) {
            Log.wtf("melcjos:", Integer.toString(melcjos.getCurrentPosition()));
            Log.wtf("melcsus:", Integer.toString(melcsus.getCurrentPosition()));
        }
        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }

    public void sliderHome(int vel, double t) {
        melcjos.setVelocity(vel);
        melcsus.setVelocity(-vel);
        double lastTime = System.currentTimeMillis();
        Log.wtf("sliderTouch",Boolean.toString(sliderTouch.isPressed()));
        while (!sliderTouch.isPressed()
                && !isStopRequested()
                && lastTime + t > System.currentTimeMillis()) {
            Log.wtf(String.valueOf(System.currentTimeMillis()), String.valueOf(sliderTouch.isPressed()));
        }
        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }
    public void Fata(double fata, double velocity, double tolerance){
        int lastPozBR = motorBR.getCurrentPosition();
        int lastPozBL = motorBL.getCurrentPosition();
        int lastPozFR = motorFR.getCurrentPosition();
        int lastPozFL = motorFL.getCurrentPosition();
        if(fata > 0){
            motorFL.setVelocity(velocity);
            motorBL.setVelocity(velocity);
            motorFR.setVelocity(velocity);
            motorBR.setVelocity(velocity);
            while(motorBR.getCurrentPosition() < fata + lastPozBR &&
                    motorBL.getCurrentPosition() < fata + lastPozBL &&
                    motorFR.getCurrentPosition() < fata + lastPozFR &&
                    motorFL.getCurrentPosition() < fata + lastPozFL && !isStopRequested());
        }
        else{
            motorFL.setVelocity(-velocity);
            motorBL.setVelocity(-velocity);
            motorBR.setVelocity(-velocity);
            motorFR.setVelocity(-velocity);
            while(motorBR.getCurrentPosition() > fata + lastPozBR &&
                    motorBL.getCurrentPosition() > fata + lastPozBL &&
                    motorFR.getCurrentPosition() > fata + lastPozFR &&
                    motorFL.getCurrentPosition() > fata + lastPozFL &&!isStopRequested());
        }
        motorBR.setVelocity(0);
        motorBL.setVelocity(0);
        motorFL.setVelocity(0);
        motorFR.setVelocity(0);
    }
    public void spitPixel(int t, double pow){
        maceta.setPower(pow);
        kdf(t);
        maceta.setPower(0);
    }
    public void melctarget(double pot, double vel, double t) {
        double lastTime = System.currentTimeMillis();
        if(potentiometru.getVoltage() > pot) {
            melcsus.setVelocity(vel);
            melcjos.setVelocity(vel);
            telemetry.addData("lastTime + t:",lastTime + t);
            telemetry.addData("current time:",System.currentTimeMillis());
            while(potentiometru.getVoltage() > pot && lastTime + t > System.currentTimeMillis()){}
        }
        else{
            melcsus.setVelocity(-vel);
            melcjos.setVelocity(-vel);
            telemetry.addData("lastTime + t:",lastTime + t);
            telemetry.addData("current time:",System.currentTimeMillis());
            while(potentiometru.getVoltage() < pot && lastTime + t > System.currentTimeMillis()){}
        }
        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }
    public void Dreapta(double dr, double velocity, double tolerance){
        int lastPozBR = motorBR.getCurrentPosition();
        int lastPozBL = motorBL.getCurrentPosition();
        int lastPozFR = motorFR.getCurrentPosition();
        int lastPozFL = motorFL.getCurrentPosition();
        if(dr > 0){
            motorFL.setVelocity(velocity);
            motorBL.setVelocity(-velocity);
            motorFR.setVelocity(-velocity);
            motorBR.setVelocity(velocity);
            while(motorBR.getCurrentPosition() < dr + lastPozBR &&
                    motorBL.getCurrentPosition() < dr + lastPozBL &&
                    motorFR.getCurrentPosition() < dr + lastPozFR &&
                    motorFL.getCurrentPosition() < dr + lastPozFL && !isStopRequested());
        }
        else{
            motorFL.setVelocity(-velocity);
            motorBL.setVelocity(velocity);
            motorBR.setVelocity(velocity);
            motorFR.setVelocity(-velocity);
            while(motorBR.getCurrentPosition() > dr + lastPozBR &&
                    motorBL.getCurrentPosition() > dr + lastPozBL &&
                    motorFR.getCurrentPosition() > dr + lastPozFR &&
                    motorFL.getCurrentPosition() > dr + lastPozFL &&!isStopRequested());
        }
        motorBR.setVelocity(0);
        motorBL.setVelocity(0);
        motorFL.setVelocity(0);
        motorFR.setVelocity(0);
    }
    public void Rotire(double rot, double velocity, double tolerance){
        int lastPozBR = motorBR.getCurrentPosition();
        int lastPozBL = motorBL.getCurrentPosition();
        int lastPozFR = motorFR.getCurrentPosition();
        int lastPozFL = motorFL.getCurrentPosition();
        if(rot > 0){
            motorFL.setVelocity(velocity);
            motorBL.setVelocity(velocity);
            motorFR.setVelocity(-velocity);
            motorBR.setVelocity(-velocity);
            while(motorBR.getCurrentPosition() < rot + lastPozBR &&
                    motorBL.getCurrentPosition() < rot + lastPozBL &&
                    motorFR.getCurrentPosition() < rot + lastPozFR &&
                    motorFL.getCurrentPosition() < rot + lastPozFL && !isStopRequested());
        }
        else{
            motorFL.setVelocity(-velocity);
            motorBL.setVelocity(-velocity);
            motorBR.setVelocity(velocity);
            motorFR.setVelocity(velocity);
            while(motorBR.getCurrentPosition() > rot + lastPozBR &&
                    motorBL.getCurrentPosition() > rot + lastPozBL &&
                    motorFR.getCurrentPosition() > rot + lastPozFR &&
                    motorFL.getCurrentPosition() > rot + lastPozFL &&!isStopRequested());
        }
        motorBR.setVelocity(0);
        motorBL.setVelocity(0);
        motorFL.setVelocity(0);
        motorFR.setVelocity(0);
    }
    public void powerRot(double pow){
        motorFL.setPower(-pow);
        motorBL.setPower(-pow);
        motorBR.setPower(pow);
        motorFR.setPower(pow);
    }
    public void close() {
        ghearaL.setPosition(0.1);
        ghearaR.setPosition(0.9);
    }

    public void open() {
        ghearaL.setPosition(0.9);
        ghearaR.setPosition(0.1);
    }

    public void kdf(int t) {
        double lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis()) ;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
