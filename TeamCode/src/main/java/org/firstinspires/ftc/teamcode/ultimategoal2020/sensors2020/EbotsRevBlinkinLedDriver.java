package org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;

import java.util.ArrayList;

public class EbotsRevBlinkinLedDriver {

    RevBlinkinLedDriver revBlinkinLedDriver;
    LedLocation ledLocation;
    RevBlinkinLedDriver.BlinkinPattern alliancePattern;
    RevBlinkinLedDriver.BlinkinPattern currentPattern;
    StopWatch patternTimer;

    boolean debugOn = true;
    String logTag = "Ebots";



    public enum LedLocation{
        MAIN("blinkin");

        String deviceName;
        LedLocation(String deviceNameIn){
            this.deviceName = deviceNameIn;
        }

        public String getDeviceName(){return this.deviceName;}
    }

    public LedLocation getLedLocation(){return this.ledLocation;}
    public RevBlinkinLedDriver.BlinkinPattern getPattern = this.currentPattern;

    public EbotsRevBlinkinLedDriver(LedLocation ledLocation, Alliance alliance, HardwareMap hardwareMap){
        this.ledLocation = ledLocation;
        if(debugOn) Log.d(logTag, "About to instantiate revBlinkinLedDriver from EbotsRevBlinkinLedDriver");
        this.revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, ledLocation.getDeviceName());
        setAlliancePattern(alliance);
        if(debugOn) Log.d(logTag, revBlinkinLedDriver.toString());
    }

    public void setAlliancePattern(Alliance alliance){
        if(alliance == Alliance.RED){
            this.alliancePattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        } else{
            this.alliancePattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        }
        this.setPattern(alliancePattern);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){

        this.currentPattern = pattern;
        this.revBlinkinLedDriver.setPattern(currentPattern);
    }

    public static EbotsRevBlinkinLedDriver getEbotsRevBlinkinLedDriverByLedLocation(
            LedLocation ledLocation, ArrayList<EbotsRevBlinkinLedDriver> ebotsRevBlinkinLedDrivers){

        EbotsRevBlinkinLedDriver returnDriver = null;
        for(EbotsRevBlinkinLedDriver driver: ebotsRevBlinkinLedDrivers){
            if(driver.ledLocation == ledLocation){
                returnDriver = driver;
                break;
            }
        }
        return returnDriver;
    }

}
