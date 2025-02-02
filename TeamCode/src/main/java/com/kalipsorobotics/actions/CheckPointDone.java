package com.kalipsorobotics.actions;

import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.Position;

public class CheckPointDone extends Action {

    PurePursuitAction purePursuitAction;
    WheelOdometry wheelOdometry;
    Position point;

    public CheckPointDone(Position point, PurePursuitAction purePursuitAction, WheelOdometry wheelOdometry) {
        this.point = point;
        this.purePursuitAction = purePursuitAction;
        this.wheelOdometry = wheelOdometry;
        this.dependentActions.add(new DoneStateAction());
    }

    public double calculateError() {
        return SharedData.getOdometryPosition().distanceTo(point);
    }

    @Override
    public boolean checkDoneCondition() {
        if(isDone) {
            return true;
        }
        //TODO add ppa when tests run true
        return purePursuitAction.getHasStarted() && calculateError() < 10;
    }

    @Override
    public void update() {

    }
}
