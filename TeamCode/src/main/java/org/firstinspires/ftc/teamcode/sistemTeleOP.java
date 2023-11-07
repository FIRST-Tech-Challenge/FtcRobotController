package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class sistemTeleOP extends OpMode {
    public Servo ghearaL,ghearaR,plauncher;
    public CRServo maceta;
    double sm = 1, slow = 1, rtval = 0, rt = 0,bval=0,xval=0;
    boolean xl,bl;
    public DcMotorEx melcjos,melcsus, slider;
    double y, x, rx, ghearaPoz=0.5, macetaPow=0;
    double max = 0,lastTime;
    boolean stop = false, sliderState = true, aIntrat = false,aAjuns = true,aInchis = true;
    double intPoz = 0.4, servoPos = 0.0;
    ChestiiDeAutonom c = new ChestiiDeAutonom();
    AnalogInput potentiometru;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        
        melcjos = hardwareMap.get(DcMotorEx.class, "melcjos");
        melcsus = hardwareMap.get(DcMotorEx.class, "melcsus");
        maceta = hardwareMap.get(CRServo.class,"maceta");
        plauncher = hardwareMap.get(Servo.class,"plauncher");
        
        ghearaL = hardwareMap.get(Servo.class, "ghearaL");
        ghearaR = hardwareMap.get(Servo.class, "ghearaR");
        slider = hardwareMap.get(DcMotorEx.class, "slider");

        potentiometru = hardwareMap.get(AnalogInput.class,"potentiometru");
        //1.94
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        melcsus.setDirection(DcMotorSimple.Direction.REVERSE);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        melcjos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        melcsus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        melcjos.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        melcsus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    public void start(){
        Systems.start();
    }

    @Override
    public void loop() {
        telemetry.addData("slider:", slider.getCurrentPosition());
        telemetry.addData("rotitor_jos:", melcjos.getCurrentPosition());
        telemetry.addData("rotitor_sus:", melcsus.getCurrentPosition());
        telemetry.addData("ghearaPoz:", ghearaPoz);
        telemetry.addData("macetaPow:", macetaPow);
        telemetry.addData("bval:", bval);
        telemetry.addData("xval:", xval);
        telemetry.addData("bl:", bl);
        telemetry.addData("gamepad2.b:", gamepad2.b);
        telemetry.addData("potentiometer:",potentiometru.getVoltage());
    }

    private final Thread Systems = new Thread(new Runnable() {
        public void run() {
            while (!stop) {
                if (gamepad2.dpad_down) {
                    maceta.setPower(1);
                }
                if (gamepad2.dpad_up) {
                    maceta.setPower(-1);
                }
                if (gamepad2.left_trigger > 0) {
                    slow = 2;
                } else {
                    slow = 1;
                }
                if (gamepad2.left_trigger > 0) {
                    plauncher.setPosition(0.5);
                } else {
                    plauncher.setPosition(0.35);
                }
                melcjos.setPower(gamepad2.left_stick_y / slow);
                melcsus.setPower(gamepad2.left_stick_y / slow);
                if (gamepad2.b != bl) {
                    bval += 0.5;
                    if (bval >= 1) {
                        bval = 0;
                    }
                }
                bl = gamepad2.b;
                if (bval == 0.5 && gamepad2.b) {
                    bval += 0.0001;
                    ghearaR.setPosition(0.15);
                    ghearaL.setPosition(0.63);
                    lastTime = System.currentTimeMillis();
                    while (lastTime + 150 > System.currentTimeMillis()) {
                    }
                    ghearaL.setPosition(0.38);
                    ghearaR.setPosition(0.38);
                }
                if (potentiometru.getVoltage() > 1.65) {
                    ghearaR.setPosition(0.38);
                    ghearaL.setPosition(0.38);
                }
                else {
                    if (gamepad2.a) {
                        ghearaR.setPosition(0.15);
                        ghearaL.setPosition(0.63);
                    }
                    if (gamepad2.y) {
                        ghearaR.setPosition(0.38);
                        ghearaL.setPosition(0.38);
                    }
                }
                slider.setPower(gamepad2.right_stick_y);
                if (gamepad2.right_trigger != rt) {
                    rtval += 0.5;
                    if (rtval >= 1) {
                        rtval = 0;
                    }
                }
                rt = gamepad2.right_trigger;
                if(rtval==0.5){
                    rtval+=0.001;
                    melctarget(1.94,800);
                    c.target(1030,800,slider,2000,1);
                }
            }
        }
    });
    public void melctarget(double pot, double vel) {
        if(potentiometru.getVoltage() < pot) {
            melcsus.setVelocity(vel);
            melcjos.setVelocity(vel);
            while(potentiometru.getVoltage() < pot){}
        }
        else{
            melcsus.setVelocity(-vel);
            melcjos.setVelocity(-vel);
            while(potentiometru.getVoltage() > pot){}
        }

        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }
    public void stop() {
        stop = true;
    }
}


