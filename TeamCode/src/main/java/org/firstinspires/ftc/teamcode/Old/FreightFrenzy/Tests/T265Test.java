package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Tests;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;
@Disabled

@Autonomous(name = "T265Test", preselectTeleOp = "OneGPTeleop")
public class T265Test extends LinearOpMode {
    T265Camera slamra;
    public void runOpMode() {
        slamra = new T265Camera(new Transform2d(), 1, hardwareMap.appContext);
        Translation2d startPose = new Translation2d(0.01,0.01);
        Rotation2d startAng = new Rotation2d(0);
        slamra.setPose(new Pose2d(startPose,startAng));
        ElapsedTime op = new ElapsedTime();
        slamra.start();
        waitForStart();
        op.reset();
        while(op.seconds()<20) {
            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if (up == null) continue;

            // We divide by 0.0254 to convert meters to inches
            Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = up.pose.getRotation();
            telemetry.addData("x",translation.getX());
            telemetry.addData("y",translation.getY());
            telemetry.addData("a",rotation.getDegrees());
            telemetry.update();
        }
        slamra.stop();
        sleep(1000);
        stop();
    }
}

