package Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
// 

@TeleOp(name = "AprilTag Detection C270", group = "Concept")
public class AprilTagDetectionC270 extends LinearOpMode {
    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagPipeline;

    static final double FEET_PER_METER = 3.28084;
    // hopefully fixed it
    // Lens intrinsics for Logitech C270
    double fx = 640.0;
    double fy = 640.0;
    double cx = 320.0;
    double cy = 240.0;

    // Tag size in meters
    double tagsize = 0.05;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = aprilTagPipeline.getLatestDetections();

            if (detections != null && detections.size() != 0) {
                for (AprilTagDetection detection : detections) {
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));

                    // Convert the rotation matrix to euler angles
                    Orientation orientation = Orientation.getOrientation(detection.pose.R, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", orientation.thirdAngle));
                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", orientation.secondAngle));
                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", orientation.firstAngle));
                }
            } else {
                telemetry.addLine("No tag detected.");
            }

            telemetry.update();
            sleep(20);
        }

        // Close camera to save resources
        webcam.closeCameraDevice();
    }
}
