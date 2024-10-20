package org.firstinspires.ftc.teamcode.NewStuff.actions;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NewStuff.FieldPosition;
import org.firstinspires.ftc.teamcode.NewStuff.PropDetector;
import org.firstinspires.ftc.teamcode.NewStuff.VisionPortalManager;

public class DetectPropPositionAction extends Action {
    ElapsedTime elapsedTime;
    VisionPortalManager visionPortalManager;
    FieldPosition fieldPosition;
    PropDetector.PROP_POSITION position;
    Boolean longPath;
    int visionTimeout = 500; // timeout detection after 2 seconds
    double time;
    boolean timeStarted = false;

    public DetectPropPositionAction(Action dependentAction, VisionPortalManager visionProcessor, FieldPosition fieldPosition, Boolean longPath) {
        elapsedTime = new ElapsedTime();
        this.visionPortalManager = visionProcessor;
        this.fieldPosition = fieldPosition;
        this.dependentAction = dependentAction;
        this.longPath = longPath;
        position = PropDetector.PROP_POSITION.UNKNOWN;
    }

    public DetectPropPositionAction(VisionPortalManager visionPortalManager, FieldPosition fieldPosition, Boolean longPath) {
        Log.d("vision", "action: constructing");
        elapsedTime = new ElapsedTime();
        Log.d("vision", "action: about to set vision portal processor field");
        this.visionPortalManager = visionPortalManager;
        Log.d("vision", "action: set up vision portal processor field");
        this.fieldPosition = fieldPosition;
        this.dependentAction = new DoneStateAction();
        this.longPath = longPath;
        position = PropDetector.PROP_POSITION.UNKNOWN;
        Log.d("vision", "action: finished constructing");
    }

    @Override
    boolean checkDoneCondition() {
        if (position == PropDetector.PROP_POSITION.UNDETECTED || position == PropDetector.PROP_POSITION.UNKNOWN) {
            return false;
        } else {
            fieldPosition.setMarkerLocation(position);
            if (fieldPosition.isRedAlliance) {
                fieldPosition.setWantedAprTagId(position, PropDetector.ALLIANCE_COLOR.RED);
            } else {
                fieldPosition.setWantedAprTagId(position, PropDetector.ALLIANCE_COLOR.BLUE);
            }

            visionPortalManager.getVisionPortal().setProcessorEnabled(visionPortalManager.getPropProcessor(), false);
            Log.d("vision", "action: finished action");
            return true;
        }
    }

    @Override
    void update() {
        Log.d("vision", "action: entered action update");

        if(!timeStarted) {
            Log.d("vision","action: started detecting");
            time = elapsedTime.milliseconds();
            position = PropDetector.PROP_POSITION.UNDETECTED;
            visionPortalManager.getVisionPortal().setProcessorEnabled(visionPortalManager.getPropProcessor(), true);
            timeStarted = true;
        }

        //detect marker position
        int i =  0;
        if (position == PropDetector.PROP_POSITION.UNDETECTED || position == PropDetector.PROP_POSITION.UNKNOWN) {
            i++;
            Log.d("vision", "action: " + i + " undetected marker, keep looking" + visionPortalManager.getVisionPortal().getCameraState());
            position = visionPortalManager.getPropProcessor().getPosition();
            Log.d("vision", "action: get position is" + visionPortalManager.getPropProcessor().getPosition());
            //Log.d("color detection", String.valueOf(propProcessor.avgCenterCb));
            //Log.d("color detection", String.valueOf(markerProcessor.avgRightCb));
            //Log.d("color detection", String.valueOf(propProcessor.avgCenterCr));
            Log.d("vision", "action: elapsed time" + String.valueOf(elapsedTime.milliseconds()));
            if ((elapsedTime.milliseconds() > time + visionTimeout) && position == PropDetector.PROP_POSITION.UNDETECTED) {
                position = PropDetector.PROP_POSITION.CENTER;
                Log.d("vision", "action: detected time out. Picking CENTER");
            }
        }

        Log.d("vision", "action: done process, detected at" + String.valueOf(elapsedTime.milliseconds()));

        //save marker position and apriltag position in robot class
        //todo
//        setWantedAprTagId(position, visionProcessor.isRedAlliance ? MarkerDetector.ALLIANCE_COLOR.RED : MarkerDetector.ALLIANCE_COLOR.BLUE);
//        setSecondWantedTagId();

        //print position
        Log.d("vision", "action: done process, detected position: " + position);
        visionPortalManager.getOpModeUtilities().getTelemetry().addData("position", position);
        visionPortalManager.getOpModeUtilities().getTelemetry().update();
    }

    public FieldPosition.PROP_LOCATION getPropLocation() {
        return fieldPosition.getPropLocation();
    }
}
