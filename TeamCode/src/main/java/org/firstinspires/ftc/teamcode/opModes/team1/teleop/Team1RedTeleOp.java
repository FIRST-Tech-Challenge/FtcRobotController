package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

@TeleOp(name="Team 1 red", group="Team 1") // Registers TeleOp OpMode Entrypoint
public class Team1RedTeleOp extends TeleOpModeBase { // Ensure you extend the base
    Team1GenericTeleOp teleop;

    @Override
    public void setup() {
        // Runs once at INIT
        teleop = new Team1GenericTeleOp();
        // Run generic TeleOp
        teleop.setup(TeamColour.RED);
    }

    @Override
    public void every_tick() {
        // Runs in the main loop
        teleop.every_tick();
        GamepadEx gamepad = Inputs.gamepad1;
        Motor left_motor = HardwareMapContainer.motor0;
        Motor right_motor = HardwareMapContainer.motor1;

        /* Arcade drive */
        DifferentialDrive drive = new DifferentialDrive(left_motor, right_motor);
        drive.arcadeDrive(gamepad.getLeftY()*2+1, gamepad.getLeftX());
        TelemetryContainer.getTelemetry().addData("Joystick Y", gamepad.getLeftY());
    }
}
