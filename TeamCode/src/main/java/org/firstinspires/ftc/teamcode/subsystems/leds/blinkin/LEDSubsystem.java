package org.firstinspires.ftc.teamcode.subsystems.leds.blinkin;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.opmodes.teleop.SampleRevBlinkinLedDriver;

import java.util.concurrent.TimeUnit;

public class LEDSubsystem extends SubsystemBase {

    private final RevBlinkinLedDriver blinkinLedDriver;
    private DisplayKind displayKind = DisplayKind.AUTO;
    private RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW;


    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private int ledPeriod = 60;

    private Deadline ledCycleDeadline;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    public LEDSubsystem(final HardwareMap hwMap, final String deviceName){

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, deviceName);

    }

    public LEDSubsystem(final HardwareMap hwMap, final String deviceName, final int ledPeriod){

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, deviceName);
        this.ledPeriod = ledPeriod;

    }

    public LEDSubsystem(final HardwareMap hwMap, final String deviceName, RevBlinkinLedDriver.BlinkinPattern p, final int ledPeriod){

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, deviceName);

        setPattern(p);
        setLedPeriod(ledPeriod);
        setLedCycleDeadline();
    }

    public void setDisplayKind(DisplayKind dk){
        displayKind = dk;
    }


    public void setPattern(RevBlinkinLedDriver.BlinkinPattern p){
        pattern = p;
        blinkinLedDriver.setPattern(pattern);
    }

    public void setLedPeriod(final int ledPeriod){
        this.ledPeriod = ledPeriod;
    }

    public void setLedCycleDeadline(){
        ledCycleDeadline = new Deadline(ledPeriod, TimeUnit.SECONDS);
    }
}
