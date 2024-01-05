package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.AutonomousAwareness;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem.FingerPositions;
import static org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem.ArmPositions;

@Autonomous(name = "AutoCommandMode")
public class AutoCommandMode extends CommandOpMode {
    // private AutonomousAwareness AA;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("AutoCommandMode");
    public DriveSubsystem driveSubsystem;
    public ArmSubsystem armSubsystem;
    public WristSubsystem wristSubsystem;
    public FingerSubsystem fingerSubsystem;

    public SequentialCommandGroup autoCommand;

    @Override
    public void initialize() {
        dbp.createNewTelePacket();
        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> motors = RobotHardwareInitializer.
                initializeDriveMotors(hardwareMap, this);

        assert motors != null;

        driveSubsystem = new DriveSubsystem(motors);
        armSubsystem = new ArmSubsystem(RobotHardwareInitializer.initializeArm(this));
        wristSubsystem = new WristSubsystem(RobotHardwareInitializer.initializeWrist(this));
        fingerSubsystem = new FingerSubsystem(RobotHardwareInitializer.initializeFinger(this));

        autoCommand = new SequentialCommandGroup(
                new RunCommand(
                        () -> driveSubsystem.moveRobot(0.5, 0, 0),
                        driveSubsystem
                ).withTimeout(1),
                new RunCommand(
                        () -> wristSubsystem.moveWrist(0.5, 0),
                        wristSubsystem
                ),
                new RunCommand(
                        () -> fingerSubsystem.locomoteFinger(FingerPositions.CLOSED),
                        fingerSubsystem
                ).whenFinished(() -> fingerSubsystem.locomoteFinger(FingerPositions.OPEN)),
                new RunCommand(
                        () -> armSubsystem.positionMoveArm(ArmPositions.BOARD)
                )
        );

        schedule(autoCommand);
    }
}
