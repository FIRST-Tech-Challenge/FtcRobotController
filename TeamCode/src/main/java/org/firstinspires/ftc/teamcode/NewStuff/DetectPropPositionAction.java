package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.VisionProcessor;

public class DetectPropPositionAction extends Action {

    PropProcessor propProcessor;
    ElapsedTime elapsedTime;
    public VisionProcessor visionProcessor;
    public PropDetector.PROP_POSITION position;
    public enum PROP_LOCATION {
        INNER, OUTER, CENTER;
    }
    public PROP_LOCATION markerLocation;
    Boolean longPath;

    public DetectPropPositionAction(Action dependentAction, VisionProcessor visionProcessor, Boolean longPath) {
        elapsedTime = new ElapsedTime();
        this.visionProcessor = visionProcessor;
        this.dependentAction = dependentAction;
        this.longPath = longPath;
    }

    public DetectPropPositionAction(VisionProcessor visionProcessor, Boolean longPath) {
        elapsedTime = new ElapsedTime();
        this.visionProcessor = visionProcessor;
        this.dependentAction = doneStateAction;
        this.longPath = longPath;
    }

    @Override
    boolean checkDoneCondition() {
        if (position == PropDetector.PROP_POSITION.UNDETECTED || position == PropDetector.PROP_POSITION.UNKNOWN) {
            return false;
        } else {
            setMarkerLocation(visionProcessor.isRedAlliance, longPath, position);
            return true;
        }
    }

    @Override
    void update() {
        Log.d("vision", "entered action update");
        int visionTimeout = 500; // timeout detection after 2 seconds
        double time;
        time = elapsedTime.milliseconds();

        //detect marker position
        position = PropDetector.PROP_POSITION.UNDETECTED;
        int i =  0;
        if (position == PropDetector.PROP_POSITION.UNDETECTED) {
            i++;
                    Log.d("vision", i + " undetected marker, keep looking" + visionProcessor.visionPortal.getCameraState());
            position = propProcessor.getPosition();
            //Log.d("color detection", String.valueOf(propProcessor.avgCenterCb));
            //Log.d("color detection", String.valueOf(markerProcessor.avgRightCb));
            //Log.d("color detection", String.valueOf(propProcessor.avgCenterCr));
            Log.d("elapsed time", String.valueOf(elapsedTime.milliseconds()));
            if ((elapsedTime.milliseconds() > time + visionTimeout) && position == PropDetector.PROP_POSITION.UNDETECTED) {
                position = PropDetector.PROP_POSITION.CENTER;
                Log.d("vision", "detected time out. Picking CENTER");
            }
        }

        Log.d("done process", "detected at" + String.valueOf(elapsedTime.milliseconds()));

        //save marker position and apriltag position in robot class
        setMarkerPos(position);
        //todo
//        setWantedAprTagId(position, visionProcessor.isRedAlliance ? MarkerDetector.ALLIANCE_COLOR.RED : MarkerDetector.ALLIANCE_COLOR.BLUE);
//        setSecondWantedTagId();

        //print position
        Log.d("done process", "detected position: " + position);
        visionProcessor.opModeUtilities.getTelemetry().addData("position ", this.position);
        visionProcessor.opModeUtilities.getTelemetry().update();
    }

    public void setMarkerPos(PropDetector.PROP_POSITION position) {
        this.position = position;
    }

    public void setMarkerLocation(boolean isRedAlliance, boolean longPath, PropDetector.PROP_POSITION position) {
        if (longPath) {
            if ((position == PropDetector.PROP_POSITION.RIGHT && isRedAlliance)
                    || (position == PropDetector.PROP_POSITION.LEFT && !isRedAlliance)) {

                markerLocation = PROP_LOCATION.INNER;

            } else if ((position == PropDetector.PROP_POSITION.LEFT && isRedAlliance)
                    || (position == PropDetector.PROP_POSITION.RIGHT && !isRedAlliance)) {

                markerLocation = PROP_LOCATION.OUTER;

            } else {

                markerLocation = PROP_LOCATION.CENTER;

            }
        } else {
            if ((position == PropDetector.PROP_POSITION.RIGHT && isRedAlliance)
                    || (position == PropDetector.PROP_POSITION.LEFT && !isRedAlliance)) {

                markerLocation = PROP_LOCATION.OUTER;

            } else if ((position == PropDetector.PROP_POSITION.LEFT && isRedAlliance)
                    || (position == PropDetector.PROP_POSITION.RIGHT && !isRedAlliance)) {

                markerLocation = PROP_LOCATION.INNER;

            } else {

                markerLocation = PROP_LOCATION.CENTER;

            }
        }

    }
}
