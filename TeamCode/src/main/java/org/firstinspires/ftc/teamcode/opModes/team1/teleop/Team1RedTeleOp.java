package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

@TeleOp(name="Team 1 red", group="Team 1") // Registers TeleOp OpMode Entrypoint
public class Team1RedTeleOp extends TeleOpModeBase { // Ensure you extend the base
    Team1GenericTeleOp teleop;

    DifferentialDrive drive;

    @Override
    public void setup() {
        // TODO: Move to own OpMode

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
        Joystick leftJoystick = new Joystick(Inputs.gamepad1.getLeftX(), Inputs.gamepad1.getLeftY());
//            Joystick rightJoystick = new Joystick(gamepad.getRightX(), gamepad.getRightY());

        drive.arcadeDrive(leftJoystick.y, leftJoystick.x);
        TelemetryContainer.getTelemetry().addData("Joystick Y", Inputs.gamepad1.getLeftY());
        TelemetryContainer.getTelemetry().addData("Left Joystick - uses Coordinates class - x", leftJoystick.x);
        TelemetryContainer.getTelemetry().addData("Left Joystick - uses coordinates class - y", leftJoystick.y);
    }
}

class Joystick {
    double x;
    double y;
    public Joystick(double x1, double y1) {
        x = x1;
        y = y1;
    }

    /**
     *
     * @param min1 Min of the first range
     * @param max1 Max of the first range
     * @param min2 min of the first range
     * @param max2 max of the first range
     */
    public void Transpose(int min1, int max1, int min2, int max2) {
        x = (((x - (min1))/(max1 - min1)) * (max2 - min2)) - (min2);
        y = (((y - (min1))/(max1 - min1)) * (max2 - min2)) - (min2);
    }
}
