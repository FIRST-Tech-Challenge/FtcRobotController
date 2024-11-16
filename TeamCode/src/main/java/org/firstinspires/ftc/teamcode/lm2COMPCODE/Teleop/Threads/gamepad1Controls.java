package org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.Threads;

import com.parshwa.drive.tele.Drive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.Teleop;

public class gamepad1Controls extends Thread{
    public boolean running = true;
    private double SPED;
    public boolean resetEnabled = false;
    public boolean bypassEnabled = false;
    private Gamepad gamepad1;
    private Drive driver;
    private gamepad2Controls controller2;
    private Servo led;
    private Teleop mainFile;
    public void init(Teleop main){
        this.gamepad1 = main.gamepad1;
        this.driver = main.driver;
        this.led = main.bottomLed;
        this.mainFile = main;
    }
    public void add2Controls(){
        this.controller2 = mainFile.gamepad2Thread;
    }
    public void run(){
        try{
            while(running && !mainFile.isStopRequested()){
                SPED = gamepad1.right_trigger;
                if(gamepad1.right_bumper){
                    SPED /= 2.0;
                }
                bypassEnabled = gamepad1.back;
                resetEnabled = gamepad1.start;
                if(resetEnabled && controller2.resetEnabled){
                    led.setPosition(0.28);
                    mainFile.safeWaitSeconds(100);
                    led.setPosition(0.0);
                    mainFile.safeWaitSeconds(100);
                    led.setPosition(0.28);
                    mainFile.safeWaitSeconds(100);
                    led.setPosition(0.0);
                    mainFile.safeWaitSeconds(100);
                    led.setPosition(0.28);
                    mainFile.safeWaitSeconds(100);
                    led.setPosition(0.0);
                    mainFile.safeWaitSeconds(100);
                    mainFile.reset();
                }else if(bypassEnabled && controller2.bypassEnabled){
                    controller2.sr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    controller2.sr.setPower(controller2.gamepad2.right_stick_y);
                    controller2.sc.setPower(controller2.gamepad2.left_stick_y);
                }
                driver.move(gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x,SPED);
            }
        }catch(Exception e){
            mainFile.telemetry.addLine("ERROR");
            mainFile.telemetry.addLine(String.valueOf(e));
        }
    }
}
