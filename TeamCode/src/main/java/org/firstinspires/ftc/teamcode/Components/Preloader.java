package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;


@Config
public class Preloader extends RFServo {
    public static double LOAD = 0.0, DEPOSIT = 1.0;
    boolean isLoaded = true;
    public Preloader(){
        super("preloadServo",1.0);
        load();
        super.setLastTime(-100);
    }
    public void load(){
        super.setPosition(LOAD); isLoaded = true;
    }
    public void deposit(){
        super.setPosition(DEPOSIT); isLoaded = false;
    }
    public boolean getLoaded(){
        return isLoaded;
    }
}
