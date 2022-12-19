package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.SignalState;
import org.firstinspires.ftc.teamcode.systems.SignalSystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "Hail Mary")
public class Nonsense extends LinearOpMode {
    Motor fl;
    Motor fr;
    Motor bl;
    Motor br;

    @Override
    public void runOpMode() throws InterruptedException {
         fl = new Motor(hardwareMap,"front_left_drive");
         fr = new Motor(hardwareMap,"front_right_drive");
         bl = new Motor(hardwareMap,"rear_left_drive");
         br = new Motor(hardwareMap,"rear_right_drive");

       /* fl.setRunMode(Motor.RunMode.VelocityControl);
        fr.setRunMode(Motor.RunMode.VelocityControl);
        bl.setRunMode(Motor.RunMode.VelocityControl);
        br.setRunMode(Motor.RunMode.VelocityControl);*/

         fr.setInverted(true);
         br.setInverted(true);

         waitForStart();

         fl.set(0.3);
         fr.set(0.3);
         bl.set(0.3);
         br.set(0.3);

         sleep(1500);

         fl.stopMotor();
         fr.stopMotor();
         bl.stopMotor();
         br.stopMotor();
    }
}
