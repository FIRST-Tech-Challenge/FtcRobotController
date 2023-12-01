package org.firstinspires.ftc.teamcode.To_Be_Removed;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionSubsystemOld {
    //Declares TensorFlow and Vision Portal Variables
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    //Declares webcam hardware name.
    private WebcamName webcamName;

    public VisionSubsystemOld(WebcamName webcamName) {
        this.webcamName = webcamName;
        initialize();
    }

    public void initialize() {
        //Initializes TensorFlow utilizing the custom model file in Utilities.
        tfod = new TfodProcessor.Builder().setModelFileName(Constants.tensorFlowModelFileName).build();

        //Initializes and configures the Vision Builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcamName);
        builder.setCameraResolution(new Size(1280, 720));
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(true);

        //Adds TensorFlow as a processor for the Vision Builder and builds the Vision Portal.
        builder.addProcessor(tfod);
        visionPortal = builder.build();

        //Initially stops the camera stream to maintain performance.
        visionPortal.stopStreaming();
    }

    public SpikePositions detectElementPosition(SpikePositions spikePosition, allianceColors allianceColor) {
        boolean detected = false;
        //Initial Invalid X Position (If no detection updates this value, no recognition occurred.)
        double x = -1;
        visionPortal.resumeStreaming();

        //Depending on the input alliance color, finds the object label linked to the appropriate color and finds its x position.
        switch (allianceColor) {
            case RED:
                while (!detected) {
                    List<Recognition> currentRecognitions = tfod.getRecognitions();

                    for (Recognition recognition : currentRecognitions) {
                        if (recognition.getLabel() == Constants.tensorFlowRedTeamPropLabel) {
                            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            detected = true;
                        }
                    }
                }
                break;

            case BLUE:
                while (!detected) {
                    List<Recognition> currentRecognitions = tfod.getRecognitions();

                    for (Recognition recognition : currentRecognitions) {
                        if (recognition.getLabel() == Constants.tensorFlowBlueTeamPropLabel) {
                            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            detected = true;
                        }
                    }
                }
                break;

            default:
                break;
        }

        //Depending on the x position, returns the position of the spike.
        if (x < Constants.spikeLeftCameraPosition && x > -1) {
            visionPortal.stopStreaming();
            return SpikePositions.LEFT;
        }
        else if (x > Constants.spikeRightCameraPosition) {
            visionPortal.stopStreaming();
            return  SpikePositions.RIGHT;
        }
        else if (x > Constants.spikeLeftCameraPosition && x < Constants.spikeRightCameraPosition) {
            visionPortal.stopStreaming();
            return  SpikePositions.MIDDLE;
        }

        //Returned if the x position isn't valid (somehow).
        else {
            visionPortal.stopStreaming();
            return  SpikePositions.NO_DETECTION;
        }
    }

    //Camera Control Functions
    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public void closeVision() {
        visionPortal.close();
    }

    public enum SpikePositions {
        LEFT,
        RIGHT,
        MIDDLE,
        NO_DETECTION
    }

    public enum allianceColors {
        RED,
        BLUE
    }
}
