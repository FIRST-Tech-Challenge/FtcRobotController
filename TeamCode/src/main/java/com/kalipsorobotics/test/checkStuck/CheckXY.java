package com.kalipsorobotics.test.checkStuck;

import static java.lang.Math.abs;

import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

public class CheckXY {
    OpModeUtilities opModeUtilities;
    double MIN_POSITION_THRESHOLD;
    public CheckXY(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        MIN_POSITION_THRESHOLD = 200; //replace with something accurate
        //in mm
    }
    public boolean isPositionOnPath(Path happyPath, int timeInMs) {
        int extra = timeInMs%1000;
        //path with increments each second
        Position intendedPos = happyPath.getPoint((timeInMs-extra)/1000);
        Position currentPos = SharedData.getOdometryPosition();
        if (abs(intendedPos.getX() - currentPos.getX()) < MIN_POSITION_THRESHOLD && abs(intendedPos.getY() - currentPos.getY()) < MIN_POSITION_THRESHOLD) {
            return false;
        }
        return false;
    }
}
