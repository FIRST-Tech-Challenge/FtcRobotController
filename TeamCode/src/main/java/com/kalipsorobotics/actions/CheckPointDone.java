package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.AutoActions.PurePursuitAction;
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
        this.dependentActions.add(new DoneStateAction());
    }

    public double calculateError() {
        return sparkfunOdometry.getCurrentPosition().distanceTo(point);
    }

    @Override
    public boolean checkDoneCondition() {
        //TODO add ppa when tests run true
        return purePursuitAction.getHasStarted() && calculateError() < 50;
    }

    @Override
    public void update() {

    }
}
