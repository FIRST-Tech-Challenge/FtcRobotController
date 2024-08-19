package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;
import android.view.animation.AccelerateInterpolator;

import com.kalipsorobotics.fresh.Vision;

import java.lang.reflect.Field;

public class GoToPropAction extends Action{

    CalculateTickInches calculateTickInches;
    DriveTrain driveTrain;
    IMUModule imuModule;
    VisionPortalProcessor visionPortalProcessor;
    FieldPosition fieldPosition;
    FieldPosition.PROP_LOCATION propLocation;

    Boolean isRedAlliance;


    ActionSet actionInner;
    ActionSet actionOuter;
    ActionSet actionCenter;

    public GoToPropAction(Action detectPropPositionAction, FieldPosition fieldPosition, DriveTrain driveTrain, IMUModule imuModule, VisionPortalProcessor visionPortalProcessor, boolean isRedAlliance) {
        this.dependentAction = detectPropPositionAction;
        this.driveTrain = driveTrain;
        this.imuModule = imuModule;
        this.visionPortalProcessor = visionPortalProcessor;
        this.fieldPosition = fieldPosition;
        this.isRedAlliance = isRedAlliance;

        initActionSet();
    }

    public void initActionSet() {
        Log.d("goprop", "init action set");

        propLocation = fieldPosition.getPropLocation();

        if(propLocation == FieldPosition.PROP_LOCATION.INNER) {
            actionInner = new ActionSet();
        } else if (propLocation == FieldPosition.PROP_LOCATION.OUTER) {
            actionOuter = new ActionSet();
        } else {
            actionCenter = new ActionSet();
        }
    }

    public void initPaths() {
        int polarity = visionPortalProcessor.getIsRedAlliance() ? -1 : 1;

        if (propLocation == FieldPosition.PROP_LOCATION.INNER) {
            Log.d("goprop", "start scheduling");
            actionInner.scheduleSequential(new MoveRobotStraightInchesAction(-29, driveTrain));
            actionInner.scheduleSequential(new TurnRobotAction(-90 * polarity, driveTrain, imuModule));

            if (isRedAlliance) {
                actionInner.scheduleSequential(new MoveRobotStraightInchesAction(-2, driveTrain));
            } else  {
                actionInner.scheduleSequential(new MoveRobotStraightInchesAction(-3, driveTrain));
            }

            actionInner.scheduleSequential(new TurnRobotAction(-90 * polarity, driveTrain ,imuModule));

            actionInner.scheduleSequential(new MoveRobotStraightInchesAction(29.5, driveTrain));

            actionInner.scheduleSequential(new TurnRobotAction(90 * polarity, driveTrain, imuModule));

        } else if (propLocation == FieldPosition.PROP_LOCATION.OUTER) {
            Log.d("goprop", "start scheduling");
            if (isRedAlliance) {
                actionOuter.scheduleSequential(new MecanumRobotAction(-23, driveTrain));
            } else {
                actionOuter.scheduleSequential(new MecanumRobotAction(-19, driveTrain));
            }

            actionOuter.scheduleSequential(new TurnRobotAction(0, driveTrain, imuModule));

            actionOuter.scheduleSequential(new MoveRobotStraightInchesAction(-29, driveTrain));

            actionOuter.scheduleSequential(new TurnRobotAction(-90 * polarity, driveTrain, imuModule));

            actionOuter.scheduleSequential(new MoveRobotStraightInchesAction(-1.5, driveTrain));

            actionOuter.scheduleSequential(new TurnRobotAction(-90 * polarity, driveTrain, imuModule));

            actionOuter.scheduleSequential(new MoveRobotStraightInchesAction(9.5, driveTrain));

            actionOuter.scheduleSequential(new TurnRobotAction(90 * polarity, driveTrain, imuModule));

            actionOuter.scheduleSequential(new MecanumRobotAction(10 * polarity, driveTrain));

            actionOuter.scheduleSequential(new TurnRobotAction(90 * polarity, driveTrain, imuModule));
        } else {
            actionCenter.scheduleSequential(new MoveRobotStraightInchesAction(-2, driveTrain));

            if (isRedAlliance) {
                actionCenter.scheduleSequential(new MecanumRobotAction(-15, driveTrain));
            } else {
                actionCenter.scheduleSequential(new MecanumRobotAction(12, driveTrain));
            }

            actionCenter.scheduleSequential(new TurnRobotAction(0, driveTrain, imuModule));

            actionCenter.scheduleSequential(new MoveRobotStraightInchesAction(-34, driveTrain));

            actionCenter.scheduleSequential(new TurnRobotAction(-90, driveTrain, imuModule));

            actionCenter.scheduleSequential(new MoveRobotStraightInchesAction(6, driveTrain));

            actionCenter.scheduleSequential(new TurnRobotAction(90, driveTrain, imuModule));

            actionCenter.scheduleSequential(new MoveRobotStraightInchesAction(-10, driveTrain));

            actionCenter.scheduleSequential(new TurnRobotAction(90, driveTrain, imuModule));

            actionCenter.scheduleSequential(new MecanumRobotAction(10 * polarity, driveTrain));

        }

    }

    @Override
    boolean checkDoneCondition() {
        return false;
    }

    @Override
    void update() {
        Log.d("goprop", "updating");
        if(!hasStarted) {
            Log.d("goprop", "init paths");
            initPaths();
            hasStarted = true;
        }

        Log.d("goprop", "update check done");
        if(propLocation == FieldPosition.PROP_LOCATION.INNER) {
            actionInner.updateCheckDone();
        } else if(propLocation == FieldPosition.PROP_LOCATION.OUTER) {
            actionOuter.updateCheckDone();
        } else {
            actionCenter.updateCheckDone();
        }
    }
}
