package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.MatchRecorder.MatchLogger;

public class MatchLoggerSubsystem extends SubsystemBase {
    MecanumDrive mecanumDrive;
    final MatchLogger MATCH_LOGGER;

    public MatchLoggerSubsystem(HardwareMap hardwareMap, final Pose2d POSE_ESTIMATE) {
        mecanumDrive = new MecanumDrive(hardwareMap, POSE_ESTIMATE);
        MATCH_LOGGER = MatchLogger.getInstance();
    }

    public void periodic() {
        mecanumDrive.updatePoseEstimate();
        MATCH_LOGGER.logRobotPose(mecanumDrive.pose);
        // NOTE: Some of the subsystems (like the ArmSubsystem) already has the logging system implemented in the code.
        // Remember to not log things twice.
        super.periodic();
    }
}
