package com.kalipsorobotics.actions.outtake;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DistanceDetectionAction extends KActionSet {

    PurePursuitAction purePursuitAction;
    Rev2mDistanceSensor revDistance;
    double targetDistance;
    double startTime;

//    public CloseWhenDetectDistanceAction(RevDistance  revDistance, double targetDistance) {
//        this(null, null, null, revDistance, targetDistance);
//
//    }

    public DistanceDetectionAction(Rev2mDistanceSensor revDistance, double targetDistance) {
       this.revDistance = revDistance;
       this.targetDistance = targetDistance;
    }

    public boolean checkDistance() {
        long timestamp = System.currentTimeMillis();
        double distance_mm = revDistance.getDistance(DistanceUnit.MM);
        Log.d("DistanceDetectAction", "getDistance elapse ms " + (System.currentTimeMillis() - timestamp));

        double elapsedTime = System.currentTimeMillis() - startTime;

        if(elapsedTime > 2000) {
            return true;
        }

        return (distance_mm < targetDistance);
    }

    @Override
    public boolean checkDoneCondition() {
        if(isDone) {
            return true;
        }

        if(!hasStarted) {
            startTime = System.currentTimeMillis();
            hasStarted = true;
        }

        return checkDistance();
    }

    @Override
    public void update() {
    }
}
