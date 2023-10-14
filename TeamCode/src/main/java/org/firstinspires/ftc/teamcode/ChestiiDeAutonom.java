package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ChestiiDeAutonom extends LinearOpMode {
    public HardwareMap hwmap = null;
    public OpenCvCamera webcam;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    public DcMotorEx rotitor_sus, rotitor_jos, brat;
    public Servo ghearaR, ghearaL, servodubios;
    public Boolean startThread;
    public TouchSensor sliderTouch;

    ChestiiDeAutonom() {}

    public void init(HardwareMap hard) {
        rotitor_jos = hard.get(DcMotorEx.class, "rotitor_jos");
        rotitor_sus = hard.get(DcMotorEx.class, "rotitor_sus");
        brat = hard.get(DcMotorEx.class, "brat");

        ghearaL = hard.get(Servo.class, "ghearaL");
        ghearaR = hard.get(Servo.class, "ghearaR");
        servodubios = hard.get(Servo.class, "servodubios");
        sliderTouch = hard.get(TouchSensor.class, "sliderTouch");

        rotitor_jos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotitor_sus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotitor_jos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotitor_sus.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotitor_jos.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotitor_sus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void target(int poz, double vel, DcMotorEx motor, double t, int tolerance) {
        if(motor.getCurrentPosition() < poz){
            motor.setVelocity(vel);
        }
        else {
            motor.setVelocity(-vel);
        }
        double lastTime = System.currentTimeMillis();
        while (!isStopRequested()
                && lastTime + t > System.currentTimeMillis()
                && (Math.abs(motor.getCurrentPosition() - poz) > tolerance)) {
        }
        motor.setVelocity(0);
    }

    public void slidertarget(int poz, double vel, double t, int tolerance) {
        Log.wtf("rotitor_jos target:", Integer.toString(-poz));
        Log.wtf("rotitor_sus target:", Integer.toString(poz));

        if(rotitor_sus.getCurrentPosition() < poz){
            rotitor_jos.setVelocity(-Math.abs(vel));
            rotitor_sus.setVelocity(Math.abs(vel));
        }
        else{
            rotitor_jos.setVelocity(Math.abs(vel));
            rotitor_sus.setVelocity(-Math.abs(vel));
        }

        double lastTime = System.currentTimeMillis();
        Log.wtf("rotitor_jos busy:",Double.toString(rotitor_jos.getVelocity()));
        Log.wtf("rotitor_sus busy:",Double.toString(rotitor_sus.getVelocity()));
        while (Math.abs(poz - rotitor_sus.getCurrentPosition()) > tolerance
                && Math.abs(poz - rotitor_jos.getCurrentPosition()) > tolerance
                && !isStopRequested()
                && lastTime + t > System.currentTimeMillis()) {
            Log.wtf("rotitor_jos:", Integer.toString(rotitor_jos.getCurrentPosition()));
            Log.wtf("rotitor_sus:", Integer.toString(rotitor_sus.getCurrentPosition()));
        }
        rotitor_jos.setVelocity(0);
        rotitor_sus.setVelocity(0);
    }

    public void sliderHome(int vel, double t) {
        rotitor_jos.setVelocity(vel);
        rotitor_sus.setVelocity(-vel);
        double lastTime = System.currentTimeMillis();
        Log.wtf("sliderTouch",Boolean.toString(sliderTouch.isPressed()));
        while (!sliderTouch.isPressed()
                && !isStopRequested()
                && lastTime + t > System.currentTimeMillis()) {
            Log.wtf(String.valueOf(System.currentTimeMillis()), String.valueOf(sliderTouch.isPressed()));
        }
        rotitor_jos.setVelocity(0);
        rotitor_sus.setVelocity(0);
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
