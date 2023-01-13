package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;

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
        if (gamepad.getLeftX() == 1) {
            Motor m_motor_1 = new Motor(hardwareMap, "motorOne");
        }
    }
}
