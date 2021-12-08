package org.firstinspires.ftc.teamcode.commands.leds.blinkin;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;

public class ShowTeamColors extends CommandBase {

    private final LEDSubsystem m_ledSubsytem;

    public ShowTeamColors(LEDSubsystem subsystem){
        m_ledSubsytem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES;
        m_ledSubsytem.setPattern(pattern);
    }

}
