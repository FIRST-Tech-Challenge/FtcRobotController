package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Aruco;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

@TeleOp
public class VIsion extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Mat markerImage = new Mat();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_4X4_50);
        Aruco.drawMarker(dictionary, 32, 200, markerImage,1);
        Imgcodecs.imwrite("tse.png", markerImage);
    }
}
