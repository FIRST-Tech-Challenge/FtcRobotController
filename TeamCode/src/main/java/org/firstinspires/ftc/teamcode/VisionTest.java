package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.Point;

@Autonomous(name="VisionTest", group="Auto")
public class VisionTest extends LinearOpMode {

    SkystoneCV cv;
    FieldTracker tracker;

    public void runOpMode() {
        waitForStart();

        cv = new SkystoneCV("Webcam 1", new Point(15, 130), new Point(85, 130), new Point(155, 130), this);
        cv.init(SkystoneCV.CameraType.WEBCAM);
        cv.detector.printWindows(this);
        telemetry.addData("Skystone location: ", cv.getSkystonePosition());
        SkystoneCV.StonePosition skyStonePosition = cv.getSkystonePosition();
        cv.camera.stopStreaming();
        cv.camera.closeCameraDevice();
        telemetry.addData("Skystone position", skyStonePosition);

        tracker = new FieldTracker(hardwareMap, telemetry, true, true);
        telemetry.addData("Using Vuforia", "true");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {}

    }
}
