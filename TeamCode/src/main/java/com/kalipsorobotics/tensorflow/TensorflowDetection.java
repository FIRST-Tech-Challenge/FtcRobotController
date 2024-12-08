//
//package com.kalipsorobotics.tensorflow;
//
//import android.graphics.Bitmap;
//
//import androidx.annotation.CallSuper;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.function.Consumer;
//import org.firstinspires.ftc.robotcore.external.function.Continuation;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//import org.openftc.easyopencv.PipelineRecordingParameters;
//import org.tensorflow.lite.Interpreter;
//import org.tensorflow.lite.support.common.FileUtil;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//
//import java.nio.ByteBuffer;
//import java.util.LinkedList;
//import java.util.List;
//import org.opencv.core.Size;
//
//@TeleOp
//public class TensorflowDetection extends LinearOpMode {
//    private static final String TFOD_MODEL_ASSET = "robotv2_model.tflite";
//    //defines webcam as constant - set to true to use
//    private static final boolean USE_WEBCAM = true;
//    private Interpreter tflite;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Load the model file from the assets folder
//        try {
//            // Load model from assets directory
//            //FileUtil.loadmappedFile = method to read the model from the assets folder, and loading into the thing called 'ByteBuffer'
//            //We use bytebuffer because TFlite model is loaded into mem as a bytebuffer thing
//            //hardwaremap to access context and laod model
//            ByteBuffer modelFile = FileUtil.loadMappedFile(hardwareMap.appContext, TFOD_MODEL_ASSET);
//            tflite = new Interpreter(modelFile);
//        } catch (Exception e) {
//            telemetry.addData("Error", "Failed to load TensorFlow Lite model");
//            telemetry.update();
//            return;
//        }
//
//        waitForStart();
//
//        try {
//            while (opModeIsActive()) {
//                float[] input = new float[224 * 224 * 3];
//                float[] output = new float[224 * 224 * 3];
//
//                //runs inference and places result in output array (below)
//                tflite.run(input, output);
//
//                telemetry.addData("Output", output[0]);
//                telemetry.update();
//            }
//        } finally {
//            myStop();
//        }
//
//        //create the vision portal
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        //set the camera
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        //choose a camera resolution
//        builder.setCamera(new Size(640, 480)); //change resolution??
//        //enabled RC preview
//        builder.enableLiveView(true);
//        //set the stream format
//        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//        //this basically just tells whether or not LiveView stops if no processors
//        builder.setAutoStopLiveView(false);
//
//        //set and enable the processor
//        builder.addProcessor(tflite); //erm what
//
//        //build the vision portal
//        VisionPortal visionPortal = builder.build();
//
//        //save CPU resources
//        visionPortal.close();
//        //telemetry
//        //figure out how to have different dependencies
//
//        //list of recognitions + display info, for loop iterates through recognitions and calculates coordinates or object
//        //* later we need to do more than display telemetry - exit main OpMode loop at some point (refer to site)
//



//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2;
//            double y = (recognition.getTop() + recognition.getBottom()) / 2;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image","%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position","%.0f / %.0f", x, y);
//            telemetry.addData("- Size","%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//            telemetry.update();
//        }   // end for() loop )
//
//    }
//
//    private void myStop() {
//        // Don't forget to close the interpreter when done
//        if (tflite != null) {
//            tflite.close();
//        }
//
//
//package com.kalipsorobotics.tensorflow;
//
//import android.graphics.Bitmap;
//
//import androidx.annotation.CallSuper;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.function.Consumer;
//import org.firstinspires.ftc.robotcore.external.function.Continuation;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//import org.openftc.easyopencv.PipelineRecordingParameters;
//import org.tensorflow.lite.Interpreter;
//import org.tensorflow.lite.support.common.FileUtil;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//
//import java.nio.ByteBuffer;
//import java.util.LinkedList;
//import java.util.List;
//import org.opencv.core.Size;
//
//@TeleOp
//public class TensorflowDetection extends LinearOpMode {
//    private static final String TFOD_MODEL_ASSET = "robotv2_model.tflite";
//    //defines webcam as constant - set to true to use
//    private static final boolean USE_WEBCAM = true;
//    private Interpreter tflite;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Load the model file from the assets folder
//        try {
//            // Load model from assets directory
//            //FileUtil.loadmappedFile = method to read the model from the assets folder, and loading into the thing called 'ByteBuffer'
//            //We use bytebuffer because TFlite model is loaded into mem as a bytebuffer thing
//            //hardwaremap to access context and laod model
//            ByteBuffer modelFile = FileUtil.loadMappedFile(hardwareMap.appContext, TFOD_MODEL_ASSET);
//            tflite = new Interpreter(modelFile);
//        } catch (Exception e) {
//            telemetry.addData("Error", "Failed to load TensorFlow Lite model");
//            telemetry.update();
//            return;
//        }
//
//        waitForStart();
//
//        try {
//            while (opModeIsActive()) {
//                float[] input = new float[224 * 224 * 3];
//                float[] output = new float[224 * 224 * 3];
//
//                //runs inference and places result in output array (below)
//                tflite.run(input, output);
//
//                telemetry.addData("Output", output[0]);
//                telemetry.update();
//            }
//        } finally {
//            myStop();
//        }
//
//        //create the vision portal
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        //set the camera
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        //choose a camera resolution
//        builder.setCamera(new Size(640, 480)); //change resolution??
//        //enabled RC preview
//        builder.enableLiveView(true);
//        //set the stream format
//        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//        //this basically just tells whether or not LiveView stops if no processors
//        builder.setAutoStopLiveView(false);
//
//        //set and enable the processor
//        builder.addProcessor(tflite); //erm what
//
//        //build the vision portal
//        VisionPortal visionPortal = builder.build();
//
//        //save CPU resources
//        visionPortal.close();
//        //telemetry
//        //figure out how to have different dependencies
//
//        //list of recognitions + display info, for loop iterates through recognitions and calculates coordinates or object
//        //* later we need to do more than display telemetry - exit main OpMode loop at some point (refer to site)
//



//           // end for() loop )
//
//    }
//
//    private void myStop() {
//        // Don't forget to close the interpreter when done
//        if (tflite != null) {
//            tflite.close();
//        }
//
//    }
//
//    private void telemetryTfod() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2;
//            double y = (recognition.getTop() + recognition.getBottom()) / 2;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image","%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position","%.0f / %.0f", x, y);
//            telemetry.addData("- Size","%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//            telemetry.update();
//        }
 //   }
//
//}
//
//
//
//}
//
//}
//
//
//
