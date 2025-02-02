package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BarHangDistanceSensorAction extends Action {

    Rev2mDistanceSensor revDistance;
    DistanceDetectionAction detectDistanceAction;
    PurePursuitAction purePursuitAction;
    KServoAutoAction closeClaw;

    public BarHangDistanceSensorAction(Outtake outtake, PurePursuitAction purePursuitAction) {
        this.revDistance = outtake.revDistanceBottom;
        detectDistanceAction = new DistanceDetectionAction(revDistance, 155); //145
        detectDistanceAction.setName("closeWhenDetectAction");

        this.purePursuitAction = purePursuitAction;

        closeClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        closeClaw.setName("closeClawAction");
        closeClaw.setDependentActions(detectDistanceAction);
    }

    @Override
    protected boolean checkDoneCondition() {
        return closeClaw.getIsDone();
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (purePursuitAction.getHasStarted()) {
            detectDistanceAction.updateCheckDone();
            closeClaw.updateCheckDone();
            if (!detectDistanceAction.getIsDone()) {
                purePursuitAction.updateCheckDone();
                if (purePursuitAction.getIsDone()) {
                    closeClaw.setIsDone(isDone);
                    Log.d("distanceForceStop", "Wall Distance:" + revDistance.getDistance(DistanceUnit.MM) +
                            "Pos: " + SharedData.getOdometryPosition().toString());
                }
            } else {
                purePursuitAction.finishedMoving();
                Log.d("cancelPurePursuit", "Bar " + revDistance.getDistance(DistanceUnit.MM));
            }

        }
    }
}