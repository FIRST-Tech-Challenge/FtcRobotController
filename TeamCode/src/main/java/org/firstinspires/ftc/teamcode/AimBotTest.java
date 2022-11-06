package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.bots.AimBot;
import org.firstinspires.ftc.teamcode.bots.CameraBot;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="Aim Test", group="Tests")

public class AimBotTest extends LinearOpMode {


    protected AimBot robot = new AimBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
    }

}
