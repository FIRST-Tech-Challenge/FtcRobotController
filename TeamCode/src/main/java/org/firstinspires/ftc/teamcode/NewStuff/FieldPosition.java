package org.firstinspires.ftc.teamcode.NewStuff;

import java.lang.reflect.Field;

public class FieldPosition {

    ALLIANCE_COLOR allianceColor;

    public enum ALLIANCE_COLOR {
        RED, BLUE;
    }


    public enum PROP_LOCATION {
        INNER, OUTER, CENTER;
    }

    PROP_LOCATION propLocation;

    public ALLIANCE_COLOR getAllianceColor() {
        return allianceColor;
    }

    public void setAllianceColor(ALLIANCE_COLOR allianceColor) {
        this.allianceColor = allianceColor;
    }

    public void setMarkerLocation(boolean isRedAlliance, boolean longPath, PropDetector.PROP_POSITION position) {
        if (longPath) {
            if ((position == PropDetector.PROP_POSITION.RIGHT && isRedAlliance)
                    || (position == PropDetector.PROP_POSITION.LEFT && !isRedAlliance)) {

                propLocation = PROP_LOCATION.INNER;

            } else if ((position == PropDetector.PROP_POSITION.LEFT && isRedAlliance)
                    || (position == PropDetector.PROP_POSITION.RIGHT && !isRedAlliance)) {

                propLocation = PROP_LOCATION.OUTER;

            } else {

                propLocation = PROP_LOCATION.CENTER;

            }
        } else {
            if ((position == PropDetector.PROP_POSITION.RIGHT && isRedAlliance)
                    || (position == PropDetector.PROP_POSITION.LEFT && !isRedAlliance)) {

                propLocation = PROP_LOCATION.OUTER;

            } else if ((position == PropDetector.PROP_POSITION.LEFT && isRedAlliance)
                    || (position == PropDetector.PROP_POSITION.RIGHT && !isRedAlliance)) {

                propLocation = PROP_LOCATION.INNER;

            } else {

                propLocation = PROP_LOCATION.CENTER;

            }
        }

    }

    public PROP_LOCATION getPropLocation() {
        return propLocation;
    }
}
