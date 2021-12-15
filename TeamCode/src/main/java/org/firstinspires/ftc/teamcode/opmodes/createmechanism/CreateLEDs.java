package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowColor;
//import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowTeamColors;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;

public class CreateLEDs {

    private final GamepadEx op;
    private LEDSubsystem leds;
    private final HardwareMap hwMap;
    private final String deviceName;
    private static final int LED_PERIOD = 10;

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
        ShowColor m_showAllianceBlueColorCommand;
        ShowColor m_showAllianceRedColorCommand;
        ShowColor m_showTeamColorsCommand;

        //X (Blue) button
        Button blueAlliance = new GamepadButton(op, GamepadKeys.Button.X);
        //B (Red) button
        Button redAlliance = new GamepadButton(op, GamepadKeys.Button.B);
        //A (Team) button
        Button teamColors = new GamepadButton(op, GamepadKeys.Button.A);

        //create LED SubSystem
        leds = new LEDSubsystem(hwMap,deviceName, LED_PERIOD);

        //create commands
        m_showAllianceBlueColorCommand = new ShowColor(leds, RevBlinkinLedDriver.BlinkinPattern.BLUE);
        m_showAllianceRedColorCommand = new ShowColor(leds, RevBlinkinLedDriver.BlinkinPattern.RED);
        m_showTeamColorsCommand = new ShowColor(leds, RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);

        //assign buttons to commands
        blueAlliance.whenPressed(m_showAllianceBlueColorCommand);
        redAlliance.whenPressed(m_showAllianceRedColorCommand);
        teamColors.whenPressed(m_showTeamColorsCommand);
    }
}
