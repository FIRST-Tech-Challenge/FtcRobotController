package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

public class GoToBoardAction extends Action{

    CalculateTickInches calculateTickInches;
    DriveTrain driveTrain;
    IMUModule imuModule;
    VisionPortalManager visionPortalManager;
    FieldPosition fieldPosition;
    FieldPosition.PROP_LOCATION propLocation;

    Boolean isRedAlliance;


    ActionSet actionInner;
    ActionSet actionOuter;
    ActionSet actionCenter;

    public GoToBoardAction(FieldPosition fieldPosition, DriveTrain driveTrain, IMUModule imuModule, VisionPortalManager visionPortalManager, boolean isRedAlliance) {
        this.dependentAction = new DoneStateAction();
        this.driveTrain = driveTrain;
        this.imuModule = imuModule;
        this.visionPortalManager = visionPortalManager;
        this.fieldPosition = fieldPosition;
        this.isRedAlliance = isRedAlliance;
    }

    public GoToBoardAction(Action detectPropPositionAction, FieldPosition fieldPosition, DriveTrain driveTrain, IMUModule imuModule, VisionPortalManager visionPortalManager, boolean isRedAlliance) {
        this.dependentAction = detectPropPositionAction;
        this.driveTrain = driveTrain;
        this.imuModule = imuModule;
        this.visionPortalManager = visionPortalManager;
        this.fieldPosition = fieldPosition;
        this.isRedAlliance = isRedAlliance;
    }

    public void initActionSet() {

        int polarity = visionPortalManager.getIsRedAlliance() ? -1 : 1;
        Log.d("goprop", "init polarity");

        Log.d("goprop", "init action set");

        propLocation = fieldPosition.getPropLocation();
        Log.d("goprop", "location is " + fieldPosition.getPropLocation());

        if(propLocation == FieldPosition.PROP_LOCATION.INNER) {
            Log.d("goprop", "init action set to inner");
            actionInner = new ActionSet();

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
            Log.d("goprop", "init action set to outer");
            actionOuter = new ActionSet();

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
            Log.d("goprop", "init action set to center");
            actionCenter = new ActionSet();

            actionCenter.scheduleSequential(new MoveRobotStraightInchesAction(-2, driveTrain));

            if (isRedAlliance) {
                actionCenter.scheduleSequential(new MecanumRobotAction(-15, driveTrain));
            } else {
                actionCenter.scheduleSequential(new MecanumRobotAction(12, driveTrain));
            }

            actionCenter.scheduleSequential(new TurnRobotAction(0, driveTrain, imuModule));

            actionCenter.scheduleSequential(new MoveRobotStraightInchesAction(-34, driveTrain));

            actionCenter.scheduleSequential(new TurnRobotAction(-90 * polarity, driveTrain, imuModule));

            // drop

            actionCenter.scheduleSequential(new WaitAction(2));

            actionCenter.scheduleSequential(new MoveRobotStraightInchesAction(6, driveTrain));

            actionCenter.scheduleSequential(new TurnRobotAction(90 * polarity, driveTrain, imuModule));

            actionCenter.scheduleSequential(new MoveRobotStraightInchesAction(-10, driveTrain));

            actionCenter.scheduleSequential(new TurnRobotAction(90 * polarity, driveTrain, imuModule));

            actionCenter.scheduleSequential(new MecanumRobotAction(10 * polarity, driveTrain));

        }

        Log.d("goprop", "finish init action set");

    }

    @Override
    boolean checkDoneCondition() {
        if(propLocation == FieldPosition.PROP_LOCATION.INNER) {
            if (actionInner.getIsDone()) {
                return true;
            } else {
                return false;
            }
        } else if(propLocation == FieldPosition.PROP_LOCATION.OUTER) {
            if (actionOuter.getIsDone()) {
                return true;
            } else {
                return false;
            }
        } else {
            if (actionCenter.getIsDone()) {
                return true;
            } else {
                return false;
            }
        }
    }

    @Override
    void update() {
        Log.d("goprop", "updating");
        if(!hasStarted) {
            Log.d("goprop", "starting now");
            initActionSet();
            hasStarted = true;
        }

        Log.d("goprop", "update check done");
        if(propLocation == FieldPosition.PROP_LOCATION.INNER) {
            actionInner.updateCheckDone();
        } else if(propLocation == FieldPosition.PROP_LOCATION.OUTER) {
            actionOuter.updateCheckDone();
        } else {
            Log.d("goprop", "center: updatecheckdone");
            actionCenter.updateCheckDone();
        }
    }
}
