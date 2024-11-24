package com.kalipsorobotics.actions;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;

public class CheckPointDone extends Action {

    PurePursuitAction purePursuitAction;
    Odometry odometry;
    Position point;

    public CheckPointDone(Position point, PurePursuitAction purePursuitAction, Odometry odometry) {
        this.point = point;
        this.purePursuitAction = purePursuitAction;
        this.odometry = odometry;
        this.dependentAction = new DoneStateAction();
    }

    public double calculateError() {
        return odometry.getCurrentPosition().distanceTo(point);
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
