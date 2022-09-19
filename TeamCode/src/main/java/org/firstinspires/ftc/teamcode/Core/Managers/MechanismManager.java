//
// Created by Neil Rodriguez 10/28/2021
//

package org.firstinspires.ftc.teamcode.Core.Managers;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.BaseClasses.EctoMechanism;

import java.util.ArrayList;

public class MechanismManager {

    //CONSTRUCTORS
    public MechanismManager() {
    }

    //VARIABLES
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    ArrayList<EctoMechanism> mechanisms = new ArrayList<EctoMechanism>();
    ArrayList<Double> lastTimeRunned = new ArrayList<>();

    //VOIDS/ FUNCTIONS
    public void addMechanism(EctoMechanism mechanism) {
        mechanisms.add(mechanism);
        lastTimeRunned.add(0.0);
    }

    public void initMechanisms() {
        for (int currentIndex = 0; currentIndex < mechanisms.size(); currentIndex++) {
            mechanisms.get(currentIndex).initMechanism(hardwareMap, telemetry);
            mechanisms.get(currentIndex).initMechanism();
            lastTimeRunned.set(currentIndex, (double) SystemClock.elapsedRealtime());
        }
    }

    public void startMechanisms() {
        for (int currentIndex = 0; currentIndex < mechanisms.size(); currentIndex++) {
            mechanisms.get(currentIndex).startMechanism();
        }
    }

    public void updateMechanisms() {
        for (int currentIndex = 0; currentIndex < mechanisms.size(); currentIndex++) {
            Double timeStep = SystemClock.elapsedRealtime() / 1000.0 - lastTimeRunned.get(currentIndex);
            lastTimeRunned.set(currentIndex, (double) SystemClock.elapsedRealtime());
            mechanisms.get(currentIndex).updateMechanism();
        }
    }

    public void stopMechanisms() {
        for (int currentIndex = 0; currentIndex < mechanisms.size(); currentIndex++) {
            mechanisms.get(currentIndex).stopMechanism();
        }
    }


}
