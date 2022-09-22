package org.firstinspires.ftc.teamcode.ebotssensors;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

public class EbotsBlinkin {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private static EbotsBlinkin ebotsBlinkin = null;


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private RevBlinkinLedDriver blinkinLedDriver;
    private StopWatch stopWatch = new StopWatch();
    private final long lockoutDuration = 500L;
    private LedState ledState = LedState.OFF;

    public enum LedState{
        ON,
        OFF
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private EbotsBlinkin(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public LedState getLedState() {
        return ledState;
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Class Methods
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public static EbotsBlinkin getInstance(HardwareMap hardwareMap){
//        if (ebotsBlinkin == null){
//            ebotsBlinkin = new EbotsBlinkin(hardwareMap);
//        }
        ebotsBlinkin = new EbotsBlinkin(hardwareMap);
        return ebotsBlinkin;
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public void lightsOn(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        ledState = LedState.ON;
    }
    public void lightsRed(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        ledState = LedState.ON;
    }
    public void lightsGreen(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        ledState = LedState.ON;
    }

    public void lightsYellow(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        ledState = LedState.ON;
    }


    public void lightsOff(){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        ledState = LedState.OFF;
    }

    private void toggleLedState(){
        if(ledState == LedState.OFF){
            lightsOn();
        } else {
            lightsOff();
        }
    }

    public void handleUserInput(Gamepad gamepad) {
        boolean lockoutActive = stopWatch.getElapsedTimeMillis() < lockoutDuration;
        if(lockoutActive) return;

        if(gamepad.dpad_right){
            toggleLedState();
        }
        stopWatch.reset();

    }

    }
