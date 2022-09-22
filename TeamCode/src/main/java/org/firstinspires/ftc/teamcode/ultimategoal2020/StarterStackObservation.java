package org.firstinspires.ftc.teamcode.ultimategoal2020;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020.TargetZone;

import java.util.ArrayList;

import static java.lang.String.format;

public class StarterStackObservation {
    private TargetZone.Zone zone;
    private static ArrayList <StarterStackObservation> observations = new ArrayList<>();
    private boolean debugOn = false;
    private String logTag = "EBOTS";
    private static int observationCount = 0;
    private static int countA;
    private static int countB;
    private static int countC;

    public StarterStackObservation (TargetZone.Zone z){
        if(debugOn) Log.d(logTag, "Creating StarterStackObservation...");
        this.zone = z;
        observations.add(this);
        if (observations.size() > 100){
            observations.remove(0);
        }
        observationCount++;
    }

    public static int getObservationCount(){return observationCount;}

    public static TargetZone.Zone getObservedTarget(){
        boolean debugOn = true;
        countA = 0;
        countB = 0;
        countC = 0;
        TargetZone.Zone zone;
        // Make sure that the observations list is not null
        if(observations != null) {
            for (StarterStackObservation o : observations) {
                if (o.zone == TargetZone.Zone.A) {
                    countA++;
                } else if (o.zone == TargetZone.Zone.B) {
                    countB++;
                } else {
                    countC++;
                }
            }
        }

        if ((countA > countB) && (countA > countC)){
                zone = TargetZone.Zone.A;
        } else if ((countB > countA) && (countB > countC)) {
            zone = TargetZone.Zone.B;
        } else {
            zone = TargetZone.Zone.C;
        }

        if (debugOn) Log.d("EBOTS", "Observations A/B/C: " + countA +
                " / " + countB + " / " + countC);
        return zone;
    }

    public static void clearObservations(){
        // Observations remain between runs, this clears then when entering the state
        observations.clear();
        observationCount = 0;
    }

    public static String getObservationReport(){
        TargetZone.Zone zone = getObservedTarget();
        return "Observed zone: " + zone.toString() + " with buffer readings for A/B/C: " +
                String.format("%d", countA) + " / " + String.format("%d", countB) + " / " +
                String.format("%d", countC) + " and " + String.format("%d", observationCount) +
                " total observations logged";
    }
}
