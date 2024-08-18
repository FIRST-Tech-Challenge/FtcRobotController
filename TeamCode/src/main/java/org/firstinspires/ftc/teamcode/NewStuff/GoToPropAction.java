package org.firstinspires.ftc.teamcode.NewStuff;

public class GoToPropAction extends Action{

    DetectPropPositionAction.PROP_LOCATION propLocation;
    DetectPropPositionAction dependentAction;
    CalculateTickInches calculateTickInches;
    DriveTrain driveTrain;

    Boolean isRedAlliance;

    public GoToPropAction(DetectPropPositionAction detectPropPositionAction, DetectPropPositionAction.PROP_LOCATION propLocation, DriveTrain driveTrain, boolean isRedAlliance) {
        this.dependentAction = detectPropPositionAction;
        this.driveTrain = driveTrain;
        this.propLocation = propLocation;
        this.isRedAlliance = isRedAlliance;
    }

    @Override
    boolean checkDoneCondition() {
        return false;
    }

    @Override
    void update() {
        if (propLocation != null) {
            int polarity = dependentAction.visionPortalProcessor.getIsRedAlliance() ? -1 : 1;

            if (propLocation == DetectPropPositionAction.PROP_LOCATION.INNER) {
                MoveRobotStraightInchesAction straight1 = new MoveRobotStraightInchesAction(calculateTickInches.inchToTicksDriveTrain(-29), driveTrain);

                if (isRedAlliance) {
                    MoveRobotStraightInchesAction straight2 = new MoveRobotStraightInchesAction(calculateTickInches.inchToTicksDriveTrain(-2), driveTrain);
                } else  {
                    MoveRobotStraightInchesAction straight2 = new MoveRobotStraightInchesAction(calculateTickInches.inchToTicksDriveTrain(-3), driveTrain);
                }
            }
        }
    }
}
