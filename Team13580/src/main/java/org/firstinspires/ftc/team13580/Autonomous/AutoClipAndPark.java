package org.firstinspires.ftc.team13580.Autonomous;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team13580.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="Robot auto clip and park", group="Robot")
public class AutoClipAndPark extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();

        //Don't touch this it already works

        //Closes the claw to secure the specimen
        robot.setHandPositions(-0.8);
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1015);



        robot.encoderArm(80, 13);

        //Closes the claw to secure the specimen



        //go straight for 2 squares
        robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.0075)) {
            telemetry.addData("Path", "Leg3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        //strafe to the left a little bit
        robot.setDrivePower(-0.5, 0.5, 0.5, -0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.0002)) {
            telemetry.addData("Path", "Leg2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        //go straight for 2 squares
        robot.setDrivePower(0.2, 0.2, 0.2, 0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.00000000001)) {
            telemetry.addData("Path", "Leg3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        //makes the elbow go down
        robot.encoderArm(-40, 10);
        sleep(1000);

        //Open claw
        robot.setHandPositions(0.5);
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg6: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //go back to not run into the submersible
        robot.setDrivePower(-0.6, -0.6, -0.6, -0.6);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //The don't touch has ended, you can now modify the code
        /* Hello, let me show you the basics
        <-the 86 is referred to as line 86 of code so each line has a number
        the code bellow works like this:
        robot.setDrivePower(power of left front wheel, power of left back wheel, power of right front wheel, power of right back wheel);
        comas and semicolons are important do not skip them, you can get an error in your code
        then the while loop has a number in this part runtime.seconds()<0.35<- this number and the ones I explained on top
        are the only thing you can change so if it is something else please ask me first because you can cause an error
         */

        //strafe to the left a little bit
        robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.0002)) {
            telemetry.addData("Path", "Leg2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        //strafe to the right a little bit
        robot.setDrivePower(-0.6, -0.6, -0.6, -0.6);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.235)) {
            telemetry.addData("Path", "Leg2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        //go straight to get into position
        robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        //go straight for 2 squares
        robot.setDrivePower(-0.2, -0.2, -0.2, -0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.00000000001)) {
            telemetry.addData("Path", "Leg3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);
        AprilTagProcessor tag= new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        while (!isStopRequested() && opModeIsActive() ){
            if (tag.getDetections().size() > 0){
                AprilTagDetection detect= tag.getDetections().get(0);
                telemetry.addData ("x", detect.ftcPose.x);
                telemetry.addData ("y", detect.ftcPose.y);
                telemetry.addData ("z", detect.ftcPose.z);
                telemetry.addData ("roll", detect.ftcPose.roll);
                telemetry.addData("pitch", detect.ftcPose.pitch);
                telemetry.addData("yaw", detect.ftcPose.yaw);
            }
            telemetry.update();
        }
    }
}
