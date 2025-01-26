package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.autoActions.SampleToBasketFunnelRoundTrip;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.modules.RevDistance;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class CloseWhenDetectDistanceAction extends KActionSet {

    PurePursuitAction purePursuitAction;
    Rev2mDistanceSensor revDistance;
    double targetDistance;

//    public CloseWhenDetectDistanceAction(RevDistance  revDistance, double targetDistance) {
//        this(null, null, null, revDistance, targetDistance);
//
//    }

    public CloseWhenDetectDistanceAction(Rev2mDistanceSensor revDistance, double targetDistance) {
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

        return checkDistance();
    }

    @Override
    public void update() {

    }
}
