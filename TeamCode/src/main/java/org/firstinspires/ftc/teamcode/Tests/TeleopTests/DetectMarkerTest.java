package org.firstinspires.ftc.teamcode.Tests.TeleopTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.IOException;

@TeleOp(name="Detect Marker Test")
public class DetectMarkerTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode()
    {

        try {
            Robot robot = new Robot(this, timer);
        }
        catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
