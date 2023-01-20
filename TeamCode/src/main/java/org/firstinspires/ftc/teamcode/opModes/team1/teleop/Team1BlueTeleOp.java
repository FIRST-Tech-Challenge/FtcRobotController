package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

@TeleOp(name="Team 1 blue", group="Team 1")
public class Team1BlueTeleOp extends TeleOpModeBase {
    Team1GenericTeleOp teleop;
    DifferentialDrive drive;

    @Override
    public void setup() {
        teleop = new Team1GenericTeleOp();
        teleop.setup(TeamColour.BLUE);
        drive = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);

    }

    @Override
    public void every_tick() {
        teleop.every_tick();

        TelemetryContainer.getTelemetry().addLine("No Transpose");
        Joystick leftJoystick = new Joystick(Inputs.gamepad1.getLeftX(), Inputs.gamepad1.getLeftY());
        Joystick rightJoystick = new Joystick(Inputs.gamepad1.getRightX(), Inputs.gamepad1.getRightY());
        drive.arcadeDrive(leftJoystick.y, leftJoystick.x);
        TelemetryContainer.getTelemetry().addData("Joystick Y", Inputs.gamepad1.getLeftY());
        TelemetryContainer.getTelemetry().addData("Left Joystick - uses Coordinates class - x", leftJoystick.x);
        TelemetryContainer.getTelemetry().addData("Left Joystick - uses coordinates class - y", leftJoystick.y);



        HardwareMapContainer.motor2.set(rightJoystick.x);

    }
}
