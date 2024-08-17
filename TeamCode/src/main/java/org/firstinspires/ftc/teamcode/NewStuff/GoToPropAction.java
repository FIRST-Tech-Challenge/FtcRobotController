package org.firstinspires.ftc.teamcode.NewStuff;

public class GoToPropAction extends Action{

    DetectPropPositionAction.PROP_LOCATION propLocation;
    DetectPropPositionAction dependentAction;

    public GoToPropAction(DetectPropPositionAction detectPropPositionAction, DetectPropPositionAction.PROP_LOCATION propLocation) {
        this.dependentAction = detectPropPositionAction;
        this.propLocation = propLocation;
    }

    @Override
    boolean checkDoneCondition() {
        return false;
    }

    @Override
    void update() {
        if (propLocation != null) {
            int polarity = dependentAction.visionProcessor.isRedAlliance ? -1 : 1;

            if (propLocation == DetectPropPositionAction.PROP_LOCATION.INNER) {


            }
        }
    }
}
