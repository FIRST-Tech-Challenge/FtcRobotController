package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.autoActions.SampleToBasketFunnelRoundTrip;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.modules.RevDistance;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class CloseWhenDetectDistanceAction extends KActionSet {

    PurePursuitAction purePursuitAction;
    WheelOdometry wheelOdometry;
    Outtake outtake;
    RevDistance revDistance;
    double targetDistance;


    public CloseWhenDetectDistanceAction(Outtake outtake, PurePursuitAction purePursuitAction, WheelOdometry wheelOdometry, RevDistance revDistance, double targetDistance) {
       this.purePursuitAction = purePursuitAction;
       this.wheelOdometry = wheelOdometry;
       this.outtake = outtake;
       this.revDistance = revDistance;
       this.targetDistance = targetDistance;
    }

    public boolean checkDistance() {
        return (revDistance.getDistance(DistanceUnit.MM) < targetDistance);
    }

    @Override
    public boolean checkDoneCondition() {
        if(isDone) {
            return true;
        }

        return purePursuitAction.getHasStarted() && checkDistance();
    }

    @Override
    public void update() {

    }
}
