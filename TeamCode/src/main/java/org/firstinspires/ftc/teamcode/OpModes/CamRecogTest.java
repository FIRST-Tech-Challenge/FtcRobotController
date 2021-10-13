package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;


@TeleOp(name = "Cam Rec Test", group = "Robot15173")
//@Disabled
public class CamRecogTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        T265Camera slamra = null;
        try {
            Transform2d cam = new Transform2d();
            double encoderMeasurementCovariance = 0.8;
            Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());

            slamra = new T265Camera(cam, encoderMeasurementCovariance, hardwareMap.appContext);
            slamra.setPose(startingPose);

            slamra.start();

            telemetry.addData("Track Cam","Ready to start");
            telemetry.update();
            waitForStart();




            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                T265Camera.CameraUpdate data =  slamra.getLastReceivedCameraUpdate();
                if (data != null) {
                    telemetry.addData("X", data.pose.getX() / 0.0254);
                    telemetry.addData("Y", data.pose.getY() / 0.0254);
                    telemetry.addData("Rotation degrees", data.pose.getRotation().getDegrees());

                }
                else
                {
                    telemetry.addData("Track Cam", "No data");
                }
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        } finally {
            if (slamra != null) {
                slamra.stop();
                slamra.free();
            }
        }
    }
}
