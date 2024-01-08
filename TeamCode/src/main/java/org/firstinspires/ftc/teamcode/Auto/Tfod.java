package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class Tfod {

    private TfodProcessor tfodProcessor;

    public Tfod() {

        tfodProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10) // max # recognitions
                .setUseObjectTracker(true) // use object tracker
                .setTrackerMaxOverlap((float) 0.2) // max % of box overlapped by another box for recognition
                .setTrackerMinSize(16) // minimum size of a tracked/recognized object (units?)
                .build();
    }

    public TfodProcessor getTfodProcessor() {
        return tfodProcessor;
    }
}
