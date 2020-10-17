package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Point;

@Autonomous(name="Test CV", group="Linear Opmode")
public class TestCV extends LinearOpMode {
    SkystoneCV cv;
    SkystoneCV.StonePosition stonePosition;

    public void runOpMode () {
        cv = new SkystoneCV("Webcam 1", new Point(15, 130), new Point(85, 130), new Point(155, 130),this);
        cv.init(SkystoneCV.CameraType.WEBCAM);
        while(!isStarted()) {
            cv.setWindows(gamepad1);
            cv.detector.printWindows(this);
            telemetry.addData("Skystone location: ", cv.getSkystonePosition());
            telemetry.update();

            //throttle to 10Hz loop to avoid burning excess CPU cycles for no reason
            sleep(100);
        }

        //store block position info
        stonePosition = cv.getSkystonePosition();

        //keep opmode running so print block location can be read from screen
        while (opModeIsActive()) {}

        //after vision done
        cv.camera.stopStreaming();
        //cv.webcam.closeCameraDevice();
    }
}
