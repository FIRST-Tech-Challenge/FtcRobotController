package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;
import org.firstinspires.ftc.teamcode.opModes.team1.teleop.Team1GenericTeleOp;

@Disabled
@TeleOp(name="Drivetrain Demo [la-reine-c v1]", group="Demo") // Registers TeleOp OpMode Entrypoint
public class LAREINECExampleDriveTrain extends TeleOpModeBase { // Ensure you extend the base
    Team1GenericTeleOp teleop;

    DifferentialDrive drive;

    @Override
    public void setup() {

        // Runs once at INIT
        teleop = new Team1GenericTeleOp();
        // Run generic TeleOp
        teleop.setup(TeamColour.RED);

        drive = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
    }

    @Override
    public void every_tick() {
        // Runs in the main loop
        teleop.every_tick();
        TelemetryContainer.getTelemetry().addLine("No Transpose");
        LAREINECJoystick leftLAREINECJoystick = new LAREINECJoystick(Inputs.gamepad1.getLeftX(), Inputs.gamepad1.getLeftY());

        // Drive
        drive.arcadeDrive(leftLAREINECJoystick.y, leftLAREINECJoystick.x);
        TelemetryContainer.getTelemetry().addData("Joystick Y", Inputs.gamepad1.getLeftY());
        TelemetryContainer.getTelemetry().addData("Left Joystick - uses Coordinates class - x", leftLAREINECJoystick.x);
        TelemetryContainer.getTelemetry().addData("Left Joystick - uses coordinates class - y", leftLAREINECJoystick.y/2);

        // Spinning Motor
        HardwareMapContainer.motor2.set(Inputs.gamepad1.getRightX());
    }
}


