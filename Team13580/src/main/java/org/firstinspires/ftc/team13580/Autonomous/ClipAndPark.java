package org.firstinspires.ftc.team13580.Autonomous;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team13580.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="clipAndPark", group="Robot")
public class ClipAndPark extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();
        double heading;
        waitForStart();
        AprilTagProcessor tag= new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(1130.80338323,1130.80338323, 1280.21111078, 368.731101737 )
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        //Closes the claw to secure the specimen
        robot.leftHand.setPosition(1);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);
        //Don't touch this it already works

        //robot.encoderArm(62, 10);
        //robot.upDown.setTargetPosition((int)((73*robot.ARM_TICKS_PER_DEGREE)));

        //go straight for 2 squares
        robot.encoderDrive(0.7,24.0,24.0,24.0,24.0, 15);
        robot.encoderArm(60, 10);
        sleep(50);
        robot.encoderDrive(0.5,8,8,8,8,20);
        sleep(100);
        robot.encoderSpoolie(0.2,10,20);


        //strafe to the left a little bit
        //robot.encoderDrive(0.6,-10.0,10.0,10.0,-10.0, 15);




        //go straight for 2 squares
        //robot.encoderDrive(5.0,8,8,8,8,15);
        sleep(100);
        robot.leftHand.setPosition(0);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(300);
        robot.encoderDrive(3.0,-10,-10,-10,-10,15);
        //sleep(100);
        robot.encoderArm(-70 ,15);
        //robot.encoderDrive(5.0,0.4,0.4,-0.4,-0.4 ,20);




        //robot.encoderArm(-70 ,15);
        robot.encoderDrive(0.8,31.5,-31.5,-31.5,31.5, 15);
        sleep(100);
        robot.encoderDrive(0.8,26.3,26.3,26.3,26.3,15);
        robot.encoderDrive(0.8,4.3,4.3,4.3,4.3,20);
        sleep(100);
        robot.encoderDrive(0.8,12,-12,-12,12, 15);
        sleep(300);
        robot.encoderDrive(0.8,-42.0,-42.0,-42.0,-42.0, 20);
        robot.encoderDrive(0.2,-2,-2,-2,-2,20);
        sleep(300);
        robot.encoderDrive(0.8, 43.0, 43.00, 43.0, 43.0, 15);
        robot.encoderDrive(0.3,3,3,3,3,20);
        sleep(100);
        robot.encoderDrive(0.8, 14.5, -14.5, -14.5, 14.5, 15);
        sleep(100);
        robot.encoderDrive(0.8,-43.0,-43.0,-43.0,-43.0, 15);
        robot.encoderDrive(0.2,-2,-2,-2,-2,20);
        sleep(200);
        robot.encoderDrive(0.8, 17,17,17,17,12);
        sleep(200);
        robot.encoderDrive(0.8, -19,19,19,-19,15);
        sleep(2000);
        robot.encoderDrive(0.8,-17,-17,-17,-17,20);
    }
}
