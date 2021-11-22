package org.firstinspires.ftc.Team19567;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

public class RoadrunnerTest extends LinearOpMode {

    private tsePipeline pipeline = new tsePipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        waitForStart();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.setPipeline(pipeline);
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        Trajectory woggywou = chassis.trajectoryBuilder(new Pose2d(0,0,0)).forward(10).build();
        chassis.followTrajectory(woggywou);

        Trajectory goihou = chassis.trajectoryBuilder(new Pose2d(10,10,0)).lineTo(new Vector2d(0,0)).build();
        chassis.followTrajectory(goihou);

        Trajectory faiyoi = chassis.trajectoryBuilder(new Pose2d(0,0,0)).strafeLeft(15).build();
        chassis.followTrajectory(faiyoi);

        Trajectory fazhedyoi = chassis.trajectoryBuilder(new Pose2d(0,0,0)).strafeTo(new Vector2d(0,15)).build();
        chassis.followTrajectory(fazhedyoi);

        Trajectory byozilious = chassis.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(90))).splineTo(new Vector2d(15,15),Math.toRadians(90)).build();
        chassis.followTrajectory(byozilious);
    }
}
