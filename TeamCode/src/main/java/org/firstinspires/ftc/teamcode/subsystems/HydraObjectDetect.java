package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.HydraObjectLocations;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class HydraObjectDetect {
    private final VisionPortal myVisionPortal;
    private final TfodProcessor myTfodProcessor;
    protected final float cXvalueForLeftToCenterObject = 200;
    private HydraOpMode mOp;

    public HydraObjectDetect(HydraOpMode op, String modelFilename) {
        mOp = op;
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName(modelFilename);
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("prop"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(4 / 3);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        // Use a webcam.
        myVisionPortalBuilder.setCamera(mOp.mHardwareMap.get(WebcamName.class, "Webcam 1"));
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Finds and returns the location of an object using the provided model
     * @return unknown if we saw nothing, left or center if we detected an object
     */
    public HydraObjectLocations GetObjectLocation() {
        HydraObjectLocations detectedLocation = HydraObjectLocations.ObjLocUnknown;
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;
        // Useful way to skip over object detection for testing
        /*{
            return cObjectLocationUnknown;
        }*/
        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        //telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            mOp.mTelemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            mOp.mTelemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
            // Display position.
            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            if (x < cXvalueForLeftToCenterObject) {
                detectedLocation = HydraObjectLocations.ObjLocLeftSpike;
            } else {
                detectedLocation = HydraObjectLocations.ObjLocCenterSpike;
            }
            // Display the position of the center of the detection boundary for the recognition
            mOp.mTelemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
        }
        return detectedLocation;
    }
}
