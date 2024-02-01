package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.PredefinedAutomation;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.util.PredefinedAutomation.commands.*;

@Deprecated
@Disabled
@Autonomous(name = "AutoCommandMode")
public class AutoCommandMode extends CommandOpMode {
    // private AutonomousAwareness AA;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("AutoCommandMode");
    public DriveSubsystem driveSubsystem;
    public ArmSubsystem armSubsystem;
    public WristSubsystem wristSubsystem;
    public FingerSubsystem fingerSubsystem;

    public SequentialCommandGroup autoCommand;
    private PredefinedAutomation PA;

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

        PA = new PredefinedAutomation(driveSubsystem, armSubsystem, wristSubsystem, fingerSubsystem);

        schedule(PA.automation.get(TESTING));
    }
}
