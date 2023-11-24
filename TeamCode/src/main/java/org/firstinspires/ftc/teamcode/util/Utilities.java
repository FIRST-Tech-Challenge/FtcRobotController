package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Utilities {

    public static Utilities sharedUtility = null;

    public HardwareMap hardwareMap = null;
    public Telemetry telemetry = null;
    public ElapsedTime runtime = new ElapsedTime();

    private ArrayList<PeriodicUpdateCallback> entitiesNeedPeriodicUpdate;
    private Boolean initialized = false;

    public void initialize(OpMode opmode) {
        hardwareMap = opmode.hardwareMap;
        telemetry = opmode.telemetry;
        runtime.reset();
        entitiesNeedPeriodicUpdate = new ArrayList<PeriodicUpdateCallback>();
        initialized = true;
    }

    public static Utilities getSharedUtility() throws IllegalArgumentException {
        if (sharedUtility == null) {
            sharedUtility = new Utilities();
        } else {
            if (!sharedUtility.initialized) {
                throw new IllegalArgumentException("Utility shared instance must be initilized in 1st usage.");
            }
        }
        return sharedUtility;
    }

    public static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            ;
        }
    }

    public void registerForPeriodicUpdate(PeriodicUpdateCallback entity) {
        if (!entitiesNeedPeriodicUpdate.contains(entity)) {
            entitiesNeedPeriodicUpdate.add(entity);
        }
    }

    public void updateAll() {
        for (int i = 0; i < entitiesNeedPeriodicUpdate.size(); i++) {
            entitiesNeedPeriodicUpdate.get(i).update();
        }
    }

    public double angleMinus(double angle1, double angle2) {
        double angleChanged = angle1 - angle2;
        if (angle2 > 160 && angle1 < -160) {
            angleChanged += 360;
        } else if (angle2 < -160 && angle1 > 160) {
            angleChanged -= 360;
        }
        return angleChanged;
    }
}