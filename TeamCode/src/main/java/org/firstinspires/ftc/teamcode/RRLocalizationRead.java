package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.*;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class RRLocalizationRead {
    MecanumDrive drive;

    public void initLocalization(HardwareMap hardwareMap)
    {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        // Pseudo code for localizer initiation.
        // Should be something like Standardlocalizer localizer = new stsndard(hardwaremap);
    }

    public Pose2d returnPose()
    {
        drive.updatePoseEstimate();
        return new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble()));
        // Pseudo code for localizer.update() and return Pose
    }


}
