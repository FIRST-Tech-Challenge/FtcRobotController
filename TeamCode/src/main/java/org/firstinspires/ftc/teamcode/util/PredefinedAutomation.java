package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.HashMap;

public class PredefinedAutomation {
    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem armSubsystem;
    private final WristSubsystem wristSubsystem;
    private final FingerSubsystem fingerSubsystem;
    private ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("PredefAuto");

    public HashMap<commands, SequentialCommandGroup> automation;

    public enum commands {
        TESTING,
    }

    public PredefinedAutomation(final DriveSubsystem _driveSubsystem,
                                final ArmSubsystem _armSubsystem,
                                final WristSubsystem _wristSubsystem,
                                final FingerSubsystem _fingerSubsystem) {
        driveSubsystem = _driveSubsystem;
        armSubsystem = _armSubsystem;
        wristSubsystem = _wristSubsystem;
        fingerSubsystem = _fingerSubsystem;

        dbp.createNewTelePacket();

        init();
    }

    public void init() {
        automation = new HashMap<>();
        automation.put(commands.TESTING,
                new SequentialCommandGroup(
                        new RunCommand(
                                () -> {
                                    dbp.debug("Moving robot...", true);
                                    driveSubsystem.moveRobot(0.5, 0, 0);
                                },
                                driveSubsystem
                        ).withTimeout(2000).whenFinished(driveSubsystem::resetDriveMotors)
                                .andThen(
                                        new RunCommand(
                                                () -> driveSubsystem.moveRobot(0, 0.5, 0),
                                                driveSubsystem
                                        ).withTimeout(2000).whenFinished(driveSubsystem::resetDriveMotors)
                                ),
                        new RunCommand(
                                () -> {
                                    dbp.debug("Moving wrist...", true);
                                    wristSubsystem.moveWrist(0, 0.5);
                                },
                                wristSubsystem
                        ).withTimeout(1000).whenFinished(() ->
                                wristSubsystem.moveWrist(0,0)),
                        new RunCommand(
                                () -> fingerSubsystem.locomoteFinger(FingerSubsystem.FingerPositions.CLOSED),
                                fingerSubsystem
                        ).withTimeout(1000).whenFinished(() -> {
                                    fingerSubsystem.locomoteFinger(FingerSubsystem.FingerPositions.OPEN);
                                }
                        ).withTimeout(1000),
                        new RunCommand(
                                () -> armSubsystem.manualMoveArm(ArmSubsystem.Direction.FRONTWARD,
                                        1)
                        ).withTimeout(1000).whenFinished(armSubsystem::haltArm),
                        new RunCommand(
                                () -> armSubsystem.manualMoveArm(ArmSubsystem.Direction.BACKWARD,
                                        1)
                        ).withTimeout(1000).whenFinished(armSubsystem::haltArm)
                )
        );
    }
}
