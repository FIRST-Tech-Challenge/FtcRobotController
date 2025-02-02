package com.kalipsorobotics.actions;

import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.autoActions.SampleToBasketFunnelRoundTrip;
import com.kalipsorobotics.localization.WheelOdometry;

public class CheckPassXFunnel extends Action {

    PurePursuitAction purePursuitAction;
    WheelOdometry wheelOdometry;

    final double passingX = SampleToBasketFunnelRoundTrip.INTAKE_SAMPLE_X_FUNNEL;

    public CheckPassXFunnel(PurePursuitAction purePursuitAction, WheelOdometry wheelOdometry) {
        this.purePursuitAction = purePursuitAction;
        this.wheelOdometry = wheelOdometry;
        this.dependentActions.add(new DoneStateAction());
    }

    public boolean checkIfPastX() {
        return (SharedData.getOdometryPosition().getX() < passingX);
    }

    @Override
    public boolean checkDoneCondition() {
        if(isDone) {
            return true;
        }
        //TODO add ppa when tests run true
        return purePursuitAction.getHasStarted() && checkIfPastX();
    }

    @Override
    public void update() {

    }
}
