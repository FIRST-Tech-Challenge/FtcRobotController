package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowAllianceColor;
import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowTeamColors;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;


@TeleOp(name="Telop: FTCLib BlinkinExample", group="FTCLib")
public class FTCLibBlinkinTeleop extends CommandOpMode {

    private LEDSubsystem m_leds;
    private ShowAllianceColor m_showAllianceColorCommand;
    private ShowTeamColors m_showTeamColorsCommand;

    @Override
    public void initialize() {

        m_leds = new LEDSubsystem(hardwareMap,"blinkin");

    }
}
