package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;

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
        init();
    }

    public void init() {
        automation.put(commands.TESTING,
                new SequentialCommandGroup(
                        new RunCommand(
                                () -> driveSubsystem.moveRobot(0.5, 0, 0),
                                driveSubsystem
                        ).withTimeout(1),
                        new RunCommand(
                                () -> wristSubsystem.moveWrist(0.5, 0),
                                wristSubsystem
                        ),
                        new RunCommand(
                                () -> fingerSubsystem.locomoteFinger(FingerSubsystem.FingerPositions.CLOSED),
                                fingerSubsystem
                        ).whenFinished(() -> fingerSubsystem.locomoteFinger(FingerSubsystem.FingerPositions.OPEN)),
                        new RunCommand(
                                () -> armSubsystem.positionMoveArm(ArmSubsystem.ArmPositions.BOARD)
                        )
                )
        );
    }
}
