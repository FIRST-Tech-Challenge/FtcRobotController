package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class ChestiiDeAutonom{
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private DcMotorEx melcsus, melcjos, slider, motorBL, motorBR, motorFL, motorFR;
    private Servo ghearaR, ghearaL, plauncher;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private CRServo maceta, extensorL, extensorR;
    private AnalogInput potentiometru;
    private DistanceSensor distanceL, distanceR;
    private boolean sasiuInited;
    private boolean isStopRequested = false;

    public void init(HardwareMap hard){
        this.init(hard, null, false);
    }

    public void init(HardwareMap hard, Telemetry telemetry, boolean shouldInitSasiu) {
        this.hardwareMap = hard;
        this.telemetry = telemetry;

        if (shouldInitSasiu) {
            initSasiu(hard);
        }
        sasiuInited = shouldInitSasiu;

        potentiometru = hard.get(AnalogInput.class, "potentiometru");
        distanceL = hard.get(DistanceSensor.class, "distanceL");
        distanceR = hard.get(DistanceSensor.class, "distanceR");

        melcjos = hard.get(DcMotorEx.class, "melcjos");
        melcsus = hard.get(DcMotorEx.class, "melcsus");
        slider = hard.get(DcMotorEx.class, "slider");

        ghearaL = hard.get(Servo.class, "gherutaL");
        ghearaR = hard.get(Servo.class, "gherutaR");
        plauncher = hard.get(Servo.class, "plauncher");

        maceta = hard.get(CRServo.class, "maceta");
        extensorL = hard.get(CRServo.class, "extensorL");
        extensorR = hard.get(CRServo.class, "extensorR");

        slider.setDirection(DcMotorEx.Direction.REVERSE);
        melcsus.setDirection(DcMotorEx.Direction.REVERSE);
        extensorL.setDirection(CRServo.Direction.REVERSE);

        melcjos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        melcsus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        melcjos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        melcjos.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        melcsus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop(){
        this.isStopRequested = true;
    }

    public void initSasiu(HardwareMap hard) {
        motorBL = hard.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hard.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hard.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hard.get(DcMotorEx.class, "motorFR"); // Motor Back-Left

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public synchronized void POWER(double df1, double sf1, double ds1, double ss1) {
        if (sasiuInited) {
            motorFR.setPower(df1);
            motorBL.setPower(ss1);
            motorFL.setPower(sf1);
            motorBR.setPower(ds1);
        }
        else {
            throw new NullPointerException("Bro sasiul nu e initializat");
        }
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
            while (!isStopRequested && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                kdf(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                kdf(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            kdf(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            kdf(20);
        }
    }

    public void initTaguriAprilie() {
        initAprilTag();
        setManualExposure(6, 250);
    }
    public double getGhearaLPosition(){
        return ghearaL.getPosition();
    }
    public double getGhearaRPosition(){
        return ghearaR.getPosition();
    }
    public void detectieTaguriAprilie(int DESIRED_TAG_ID) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                if (detection.id == DESIRED_TAG_ID) {
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

    public void deschidere() {
        ghearaR.setPosition(0.14);
        ghearaL.setPosition(0.64);
    }

    public void inchidere() {
        ghearaR.setPosition(0.38);
        ghearaL.setPosition(0.38);
    }

    public synchronized void target(double poz, double vel, DcMotorEx motor, double t, int tolerance) {
        if (motor.getCurrentPosition() < poz) {
            motor.setVelocity(vel);
        }
        else {
            motor.setVelocity(-vel);
        }
        double lastTime = System.currentTimeMillis();
        while (!isStopRequested
                && lastTime + t > System.currentTimeMillis()
                && (abs(motor.getCurrentPosition() - poz) > tolerance)) {
        }
        motor.setVelocity(0);
    }

    public double getMotorBLPower() {
        return motorBL.getPower();
    }

    public double getMotorFLPower() {
        return motorFL.getPower();
    }

    public double getMotorBRPower() {
        return motorBR.getPower();
    }

    public double getMotorFRPower() {
        return motorFR.getPower();
    }

    public double getSliderPower() {
        return slider.getPower();
    }

    public double getMelcjosPower() {
        return melcjos.getPower();
    }

    public double getMelcsusPower() {
        return melcsus.getPower();
    }

    public double getMacetaPower() {
        return maceta.getPower();
    }

    public double getPotentiometruVoltage() {
        return potentiometru.getVoltage();
    }

    public double getDistanceL(DistanceUnit distanceUnit) {
        return distanceL.getDistance(distanceUnit);
    }

    public double getDistanceR(DistanceUnit distanceUnit) {
        return distanceR.getDistance(distanceUnit);
    }

    public DcMotorEx getSlider() {
        return slider;
    }

    public void setIsStopRequested(boolean value){
        isStopRequested = value;
    }

    public void setExtensorPower(double pow, int t) {
        extensorR.setPower(pow);
        extensorL.setPower(pow);
        try {
            Thread.sleep(t);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        extensorL.setPower(0);
        extensorR.setPower(0);
    }

    public synchronized void setMelcPower(double melcPower) {
        melcsus.setPower(melcPower);
        melcjos.setPower(melcPower);
    }

    public boolean isRobotFolded() {
        return getPotentiometruVoltage() > 1.7;
    }

    public void setSliderPower(double sliderPower) {
        slider.setPower(sliderPower);
    }

    public synchronized void setMacetaPower(double pow) {
        maceta.setPower(pow);
    }

    public synchronized void spitPixel(int t, double pow) {
        maceta.setPower(pow);
        kdf(t);
        maceta.setPower(0);
    }

    public void letPixelDrop(long delay) {
        deschidere();
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        inchidere();
    }

    public void melctarget(double pot, double vel, double t) {
        double lastTime = System.currentTimeMillis();
        if (potentiometru.getVoltage() > pot) {
            melcsus.setVelocity(vel);
            melcjos.setVelocity(vel);
            while (potentiometru.getVoltage() > pot && lastTime + t > System.currentTimeMillis()) {
            }
        }
        else {
            melcsus.setVelocity(-vel);
            melcjos.setVelocity(-vel);
            while (potentiometru.getVoltage() < pot && lastTime + t > System.currentTimeMillis()) {
            }
        }
        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }

    public synchronized void setPlauncherPosition(double position) {
        plauncher.setPosition(position);
    }

    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis());
    }
}
