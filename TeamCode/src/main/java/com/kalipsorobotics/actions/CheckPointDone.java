package com.kalipsorobotics.actions;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Point;

public class CheckPointDone extends Action {

    PurePursuitAction purePursuitAction;
    Odometry odometry;
    Point point;

    public CheckPointDone(Point point, PurePursuitAction purePursuitAction, Odometry odometry) {
        this.point = point;
        this.purePursuitAction = purePursuitAction;
        this.odometry = odometry;
        this.dependentAction = new DoneStateAction();
    }

    public double calculateError() {
        return odometry.getCurrentPosition().toPoint().distanceTo(point);
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
