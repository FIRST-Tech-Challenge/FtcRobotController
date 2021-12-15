package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowAllianceColor;
import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowTeamColors;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;

public class CreateLEDs {

    private final GamepadEx op;
    private LEDSubsystem m_leds;
    private final HardwareMap hwMap;
    private final String deviceName;

    public CreateLEDs (final HardwareMap hwMap, final String ledName, final GamepadEx op){
        this.op = op;
        deviceName = ledName;
        this.hwMap = hwMap;
    }

    public CreateLEDs (final HardwareMap hwMap, final String ledName, final GamepadEx op, Boolean autoCreate){
        this.op = op;
        deviceName = ledName;
        this.hwMap = hwMap;

        if (autoCreate) create();
    }

    public void create(){
        ShowAllianceColor m_showAllianceBlueColorCommand;
        ShowAllianceColor m_showAllianceRedColorCommand;
        ShowTeamColors m_showTeamColorsCommand;

        //X (Blue) button
        Button blueAlliance = new GamepadButton(op, GamepadKeys.Button.X);
        //B (Red) button
        Button redAlliance = new GamepadButton(op, GamepadKeys.Button.B);
        //A (Team) button
        Button teamColors = new GamepadButton(op, GamepadKeys.Button.A);

        //create LED SubSystem
        m_leds = new LEDSubsystem(hwMap,deviceName, 10);

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
