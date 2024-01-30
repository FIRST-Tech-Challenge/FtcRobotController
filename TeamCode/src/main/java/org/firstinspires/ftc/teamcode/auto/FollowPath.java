package org.firstinspires.ftc.teamcode.auto;


import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.ChassisFeedForward.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.*;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utils.PID.*;

import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FollowPath extends BTCommand {
    private final ElapsedTime m_timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final MecanumDriveKinematics m_kinematics;
    private final BTHolonomicDriveController m_controller;
    private final Consumer<ChassisSpeeds> m_outputChassisSpeeds;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final Consumer<Pose2d> m_resetOdometry;
    private Telemetry dashboard =FtcDashboard.getInstance().getTelemetry();

    public FollowPath(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Supplier<Rotation2d> desiredRotation,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            MecanumDriveKinematics kinematics,
            Consumer<Pose2d> resetOdometry,
            Subsystem... requirments) {
        this(
                trajectory,
                pose,
                new BTHolonomicDriveController(
                        xController,
                        yController,
                        thetaController),
                desiredRotation,
                outputChassisSpeeds,
                kinematics,
                resetOdometry,
                requirments
                );
    }

    public FollowPath(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            BTHolonomicDriveController controller,
            Supplier<Rotation2d> desiredRotation,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            MecanumDriveKinematics kinematics,
            Consumer<Pose2d> resetOdometry,
            Subsystem... requirments) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_controller = controller;
        m_desiredRotation = desiredRotation;
        m_outputChassisSpeeds = outputChassisSpeeds;
        m_kinematics = kinematics;
        this.m_resetOdometry= resetOdometry;
        addRequirements(requirments);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        //        m_controller.m_thetaController.setConstraints(
//                new TrapezoidProfile.Constraints(
//                        in.get("maxV"),
//                        Math.pow(in.get("maxV"),2)
//                        0,0
//                ));
//        m_controller.m_thetaController.setPID(
//                in.get("kP"),
//                in.get("kI"),
//                in.get("kD")
//        );
        m_resetOdometry.accept(m_trajectory.sample(0).poseMeters);
        Constants.ChassisConstants.feedForward= new SimpleMotorFeedforward(ffks,ffkv,ffka);
    }

    @Override
    public void execute() {
        double curTime = m_timer.time();
        Trajectory.State desiredState = m_trajectory.sample(curTime);
        if (m_desiredRotation.get() == null) {
            ChassisSpeeds targetChassisSpeeds =
                    m_controller.calculate(m_pose.get(), desiredState, m_trajectory.getStates().get(m_trajectory.getStates().size() - 1).poseMeters.getRotation());
            m_outputChassisSpeeds.accept(targetChassisSpeeds);

        } else {
            ChassisSpeeds targetChassisSpeeds =
                    m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
            m_outputChassisSpeeds.accept(targetChassisSpeeds);
        }


    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return m_timer.time() > (m_trajectory.getTotalTimeSeconds());
    }
}





