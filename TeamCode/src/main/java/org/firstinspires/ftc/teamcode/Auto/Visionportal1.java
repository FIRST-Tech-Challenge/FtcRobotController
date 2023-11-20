// "builder", not "easy"

package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(group="Concept")
public class Visionportal1 {

    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagProcessor myAprilTagProcessor;

    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

// Optional: specify a custom Library of AprilTags.
myAprilTagProcessorBuilder.setTagLibrary(CurrentGameTagLibrary); // CurrentGameTagLibrary = centerstage + sample apriltags

// Optional: set other custom features of the AprilTag Processor (4 are shown here).
myAprilTagProcessorBuilder.setDrawTagID(true);       // Default: true, for all detections.
myAprilTagProcessorBuilder.setDrawTagOutline(true);  // Default: true, when tag size was provided (thus eligible for pose estimation).
myAprilTagProcessorBuilder.setDrawAxes(true);        // Default: false.
myAprilTagProcessorBuilder.setDrawCubeProjection(true);        // Default: false.

// Create an AprilTagProcessor by calling build()
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();

}
