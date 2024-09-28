//package org.firstinspires.ftc.teamcode.Auto;
//
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//
//public class Tfod {
//
//    private TfodProcessor tfodProcessor;
//
//    private static final String TFOD_MODEL_ASSET = "new1_model.tflite";
//    private static final String[] LABELS = {
//            "blueCrown",
//            "redCrown"
//    };
//
//    private String aprilTagDirection = "right";
//
//    public Tfod() {
//
//        tfodProcessor = new TfodProcessor.Builder()
//                .setMaxNumRecognitions(10) // max # recognitions
//                .setUseObjectTracker(true) // use object tracker
//                .setTrackerMaxOverlap((float) 0.2) // max % of box overlapped by another box for recognition
//                .setTrackerMinSize(16) // minimum size of a tracked/recognized object (units?)
//
//
//                // With the following lines commented out, the default TfodProcessor Builder
//                // will load the default model for the season. To define a custom model to load,
//                // choose one of the following:
//                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
//                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                .setModelAssetName(TFOD_MODEL_ASSET)
//                //.setModelFileName(TFOD_MODEL_FILE)
//
//                // The following default settings are available to un-comment and edit as needed to
//                // set parameters for custom models.
//                .setModelLabels(LABELS)
//                .setIsModelTensorFlow2(true)
//                .setIsModelQuantized(true)
//                .setModelInputSize(300)
//                .setModelAspectRatio(16.0 / 9.0)
//                .build();
//
//        tfodProcessor.setMinResultConfidence(0.25f);
//
//    }
//
//    public TfodProcessor getTfodProcessor() {
//        return tfodProcessor;
//    }
//
//    public String getDirection() {
//        List<Recognition> currentRecognitions = tfodProcessor.getRecognitions();
//
//        if (currentRecognitions.size() != 0) {
//            double x = (currentRecognitions.get(0).getLeft() + currentRecognitions.get(0).getRight()) / 2 ;
//
//            if (x < 200) {
//                aprilTagDirection = "left";
//            } else if ((x >= 200) && (x < 450)) {
//                aprilTagDirection = "center";
//            } else if (x >= 450) {
//                aprilTagDirection = "right";
//            }
//
//        } else {
//            aprilTagDirection = "right";
//        }
//
//        return aprilTagDirection;
//    }
//
//}
//
//
