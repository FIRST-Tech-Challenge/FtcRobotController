package com.kalipsorobotics.actions;

import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.math.Position;

public class CheckPointDone extends Action {

    PurePursuitAction purePursuitAction;
    SparkfunOdometry sparkfunOdometry;
    Position point;

    public CheckPointDone(Position point, PurePursuitAction purePursuitAction, SparkfunOdometry sparkfunOdometry) {
        this.point = point;
        this.purePursuitAction = purePursuitAction;
        this.sparkfunOdometry = sparkfunOdometry;
        this.dependentAction = new DoneStateAction();
    }

    public double calculateError() {
        return sparkfunOdometry.getCurrentPosition().distanceTo(point);
    }

    @Override
    public boolean checkDoneCondition() {
        if(purePursuitAction.getHasStarted() && calculateError() < 50) { //TODO add ppa when tests run true
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {

    }
}
