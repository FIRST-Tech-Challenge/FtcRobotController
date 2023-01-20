package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class Test extends LinearOpMode {
    // the camera
    private OpenCvWebcam webcam;
    // to store the red and blue values from the camera
    public static double red, blue;


    // we save the finishing angle for the field oriented after this op mode
    public static double lastAngle;
    // when turning off the op mode the imu turns off and his last value is 0, therefor we need to save the value before that (create a delay)
    public static double delayMaker;

    SampleMecanumDrive drive;
    @Override
    public void runOpMode() {
        Servo puffer = hardwareMap.servo.get("puffer");

        double p = 0;
        double v = 1;

        waitForStart();

        while (opModeIsActive()){
            puffer.setPosition(p - gamepad1.left_stick_y * v);

            if (A_pressed()){
                p -= gamepad1.left_stick_y * v;
                v /= 2;
            }

            telemetry.addData("position", p - gamepad1.left_stick_y * v);
            telemetry.update();
        }
    }



    private boolean a = true;
    private boolean A_pressed(){
        if (gamepad.a){
            if(a){
                a = false;
                return true;
            }
        } else a = true;
        return false;
    }
}