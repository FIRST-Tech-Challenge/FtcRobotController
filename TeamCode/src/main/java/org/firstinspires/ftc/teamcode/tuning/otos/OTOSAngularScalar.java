package org.firstinspires.ftc.teamcode.tuning.otos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
@TeleOp
public class OTOSAngularScalar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0,0,0));
        double radsTurned = 0;
        Rotation2d lastHeading = Rotation2d.fromDouble(0);
        telemetry.addLine("OTOS Angular Scalar Tuner");
        telemetry.addLine("Press START, then rotate the robot on the ground 10 times (3600 degrees).");
        telemetry.addLine("Then copy the scalar into SparkFunOTOSDrive.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            radsTurned += drive.pose.heading.minus(lastHeading);
            lastHeading = drive.pose.heading;
            telemetry.addData("Uncorrected Degrees Turned", Math.toDegrees(radsTurned));
            telemetry.addData("Calculated Angular Scalar", 3600 / Math.toDegrees(radsTurned));
            telemetry.update();
        }


    }
}
