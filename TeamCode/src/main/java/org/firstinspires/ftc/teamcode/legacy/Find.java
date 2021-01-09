// package org.firstinspires.ftc.teamcode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
// import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
// import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import java.util.List;

// //these functions are taken from scripts provided by FIRST
// //they use TensorFlow to detect stones and skystones in the phone's camera
// //there's already a lot of documentation in it so i wont touch it too much
// //i also dont know how it works and im scared itll break if i do anything
// //also all of the telemetry is commented out because it telemetry only works
// //inside the autonomous or teleop script for some reason

// public class Find extends LinearOpMode {

//   private VuforiaSkyStone vuforiaSkyStone;
//   private TfodSkyStone tfodSkyStone;

//   List<Recognition> recognitions;
//   int ImageHeight;
//   float ObjectHeight;
//   double ObjectAngle;
//   double ObjectHeightRatio;
//   int SkystoneCount;
//   boolean SkystoneFound;
//   double TargetHeightRatio;

//   public Find() {
//     vuforiaSkyStone = new VuforiaSkyStone();
//     tfodSkyStone = new TfodSkyStone();

//     // Init Vuforia because Tensor Flow needs it.
//     vuforiaSkyStone.initialize(
//       "", // vuforiaLicenseKey
//       VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
//       false, // useExtendedTracking
//       false, // enableCameraMonitoring
//       VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
//       0, // dx
//       0, // dy
//       0, // dz
//       0, // xAngle
//       0, // yAngle
//       0, // zAngle
//       false); // useCompetitionFieldTargetLocations

//     //telemetry.addData("Vuforia", "initialized");
//     //telemetry.update();
//     // Let's use 70% minimum confidence and
//     // and no object tracker.
//     tfodSkyStone.initialize(vuforiaSkyStone, 0.7F, false, false);
//     //telemetry.addData(">", "Press Play to start");
//     //telemetry.update();
//     // Set target ratio of object height to image
//     // height value corresponding to the length
//     // of the robot's neck.
//     TargetHeightRatio = 0.8;

//     tfodSkyStone.activate();
//     // We'll loop until gold block captured or time is up
//     SkystoneFound = false;

//   }


//   public double[] findSkystoneAngle() {
//     int SkystoneCount = 0;
//     recognitions = tfodSkyStone.getRecognitions();
//     if (recognitions.size() > 0) {
//       for (Recognition recognition : recognitions) {
//         if (recognition.getLabel().equals("Skystone")) {
//           SkystoneCount += 1;
//           ObjectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
//           double result[] = {ObjectAngle, SkystoneCount};
//           return result;
//         }
//       }
//     }
//     double result[] = {0, 0};
//     return result;
//   }

//   public int countSkystones() {
//     recognitions = tfodSkyStone.getRecognitions();
//     // Report number of recognitions.
//     //telemetry.addData("Objects Recognized", recognitions.size());
//     // If some objects detected...
//     if (recognitions.size() > 0) {
//       // ...let's count how many are gold.
//       SkystoneCount = 0;
//       // Step through the stones detected.
//       for (Recognition recognition : recognitions) {
//         if (recognition.getLabel().equals("Skystone")) {
//           // A Skystone has been detected.
//           SkystoneCount = SkystoneCount + 1;
//         }
//       }
//       return SkystoneCount;
//     }
//     return SkystoneCount;
//   }




//   public void runOpMode() {

//   }
// }
