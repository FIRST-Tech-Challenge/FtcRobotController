package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import org.firstinspires.ftc.teamcode.ebotsenums.BarCodePosition;

import java.util.ArrayList;

public class BarCodeObservation {

    private static int bufferSize = 100;

    private static int observationCount = 0;  // cummulative observations

    private static ArrayList<BarCodePosition> barCodePositions = new ArrayList<>();

    public static int getObservationCount() {
        return observationCount;
    }

    public static void resetObservations(){
        barCodePositions = new ArrayList<>();
        observationCount = 0;
    }
    public static BarCodePosition giveBarCodePosition(){
        BarCodePosition barCodePosition = BarCodePosition.RIGHT;
        int middleCount = 0;
        int leftCount = 0;
        int rightCount = 0;
        for(BarCodePosition pos: barCodePositions) {
            if (pos == BarCodePosition.LEFT) leftCount++;
            if (pos == BarCodePosition.MIDDLE) middleCount++;
            if (pos == BarCodePosition.RIGHT) rightCount++;
        }
        if (leftCount > middleCount && leftCount > rightCount){
            barCodePosition = BarCodePosition.LEFT;
        } else if (middleCount > leftCount && middleCount > rightCount){
            barCodePosition = BarCodePosition.MIDDLE;
        }
        return barCodePosition;
    }

    public BarCodeObservation(BarCodePosition barCodePosition) {
        if (barCodePositions.size() >= bufferSize){
            barCodePositions.remove(0);

        }
        barCodePositions.add(barCodePosition);
        observationCount++;




    }
}
