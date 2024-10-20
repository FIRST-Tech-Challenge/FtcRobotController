package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import java.lang.reflect.Field;

public class FieldPosition {

    ALLIANCE_COLOR allianceColor;

    public enum ALLIANCE_COLOR {
        RED, BLUE;
    }

    int wantedAprTagId;


    public enum PROP_LOCATION {
        INNER, OUTER, CENTER;
    }

    PROP_LOCATION propLocation;
    public Boolean isRedAlliance;
    Boolean longPath;

    public FieldPosition(Boolean isRedAlliance, Boolean longPath) {
        this.isRedAlliance = isRedAlliance;
        this.longPath = longPath;
    }

    public ALLIANCE_COLOR getAllianceColor() {
        return allianceColor;
    }

    public void setAllianceColor(ALLIANCE_COLOR allianceColor) {
        this.allianceColor = allianceColor;
    }

    public void setMarkerLocation(PropDetector.PROP_POSITION position) {
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

    public void setWantedAprTagId(PropDetector.PROP_POSITION position, PropDetector.ALLIANCE_COLOR allianceColor) {

        if (allianceColor == PropDetector.ALLIANCE_COLOR.RED) {
            switch (position) {
                case CENTER:
                    Log.d("vision", "setWantedAprTagId: tag = " + 5);
                    wantedAprTagId = 5;
                    break;
                case RIGHT:
                    Log.d("vision", "setWantedAprTagId: tag = " + 6);
                    wantedAprTagId = 6;
                    break;
                case LEFT:
                    Log.d("vision", "setWantedAprTagId: tag = " + 4);
                    wantedAprTagId = 4;
                    break;
                default:
                    Log.d("vision", "setWantedAprTagId: enter default. tag = " + 5);
                    wantedAprTagId = 5;
            }
        } else {
            switch (position) {
                case CENTER:
                    Log.d("vision", "setWantedAprTagId: tag = " + 2);
                    wantedAprTagId = 2;
                    break;
                case RIGHT:
                    Log.d("vision", "setWantedAprTagId: tag = " + 3);
                    wantedAprTagId = 3;
                    break;
                case LEFT:
                    Log.d("vision", "setWantedAprTagId: tag = " + 1);
                    wantedAprTagId = 1;
                    break;
                default:
                    Log.d("vision", "setWantedAprTagId: enter default. tag = " + 2);
                    wantedAprTagId = 2;
            }
        }
    }

    public PROP_LOCATION getPropLocation() {
        return propLocation;
    }
    public int getWantedAprTagId() {
        return wantedAprTagId;
    }
}
