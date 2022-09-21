package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowAllianceColor;
import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowTeamColors;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;

@Deprecated
@Disabled
@TeleOp(name="Telop: FTCLib BlinkinExample", group="FTCLib")
public class FTCLibBlinkinTeleop extends CommandOpMode {

    private LEDSubsystem m_leds;
    private ShowAllianceColor m_showAllianceBlueColorCommand;
    private ShowAllianceColor m_showAllianceRedColorCommand;
    private ShowTeamColors m_showTeamColorsCommand;

    @Override
    public void initialize() {

        //set up Game pad 1
        GamepadEx toolOp = new GamepadEx(gamepad1);

        //X (Blue) button
        Button blueAlliance = new GamepadButton(toolOp, GamepadKeys.Button.X);
        //B (Red) button
        Button redAlliance = new GamepadButton(toolOp, GamepadKeys.Button.B);
        //A (Team) button
        Button teamColors = new GamepadButton(toolOp, GamepadKeys.Button.A);

        //create LED SubSystem
        m_leds = new LEDSubsystem(hardwareMap,"blinkin", 10);

        //create commands
        m_showAllianceBlueColorCommand = new ShowAllianceColor(m_leds, ShowAllianceColor.AllianceColor.BLUE);
        m_showAllianceRedColorCommand = new ShowAllianceColor(m_leds, ShowAllianceColor.AllianceColor.RED);
        m_showTeamColorsCommand = new ShowTeamColors(m_leds);

        //assign buttons to commands
        blueAlliance.whenPressed(m_showAllianceBlueColorCommand);
        redAlliance.whenPressed(m_showAllianceRedColorCommand);
        teamColors.whenPressed(m_showTeamColorsCommand);


    }
}
