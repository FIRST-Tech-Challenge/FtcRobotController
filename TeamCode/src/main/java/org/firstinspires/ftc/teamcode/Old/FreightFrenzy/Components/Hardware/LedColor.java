package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LedColor
{

    // Define class members
    HardwareMap  hwMap      =  null;
    static  int  NO_OF_LEDS  =   4;         // Number of leds
    /*Display devices */
    private DigitalChannel[] redLED;
    private DigitalChannel [] greenLED;


    public LedColor(LinearOpMode opMode) {
        redLED = new DigitalChannel[5];
        greenLED = new DigitalChannel[5];
        // Save reference to Hardware map
        hwMap = opMode.hardwareMap;


        for (int i = 1; i< NO_OF_LEDS+1; i++) {
            redLED[i] = hwMap.get(DigitalChannel.class, "red "+i);
            greenLED[i] = hwMap.get(DigitalChannel.class, "green "+i);
            // change LED mode from input to output
            redLED[i].setMode(DigitalChannel.Mode.OUTPUT);
            greenLED[i].setMode(DigitalChannel.Mode.OUTPUT);
        }
    }


    public void LedOff(int led) {
//        op.telemetry.addData("led "+led,"off");
//        op.telemetry.update();
        redLED[led].setState(true);
        greenLED[led].setState(true);
    }

    public void LedGreen(int led) {
//        op.telemetry.addData("led "+led,"off");
//        op.telemetry.update();
        redLED[led].setState(true);
        greenLED[led].setState(false);
    }

    public void LedRed(int led) {
//        op.telemetry.addData("led "+led,"off");
//        op.telemetry.update();
        redLED[led].setState(false);
        greenLED[led].setState(true);
    }
    public void LedAmber(int led) {
//        op.telemetry.addData("led "+led,"off");
//        op.telemetry.update();
        redLED[led].setState(false);
        greenLED[led].setState(false);
    }

}

