package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LedLights {
    public final RevBlinkinLedDriver lights;
    public ledStates currentState;



    public enum ledStates {
        INIT,
        OPEN,
        RED,
        BLUE,
        HANG,
        ENDGAME,
        INTAKE,
        DARK
    }

    public LedLights(HardwareMap hardwareMap) {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        currentState = ledStates.INIT;
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

    public void setLed(ledStates state) {
        switch (state) {
            case OPEN:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                currentState = state;
                break;
            case RED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                currentState = state;
                break;
            case BLUE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                currentState = state;
                break;
            case HANG:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                currentState = state;
                break;
            case ENDGAME:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                currentState = state;
                break;
            case INTAKE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM);
                currentState = state;
                break;
            case DARK:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                currentState = state;
                break;
        }
    }
}
