package org.firstinspires.ftc.teamcode.commands.leds.blinkin;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;

public class ShowAllianceColor extends CommandBase {

    private final LEDSubsystem m_ledSubsytem;

    private AllianceColor allianceColor;

    protected enum AllianceColor {
        RED,
        BLUE
    }

    public ShowAllianceColor(LEDSubsystem subsystem,AllianceColor ac){
        m_ledSubsytem = subsystem;
        allianceColor = ac;
        addRequirements(m_ledSubsytem);
    }

    @Override
    public void initialize(){
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;

        if(allianceColor == AllianceColor.BLUE){
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        }
        else if (allianceColor == AllianceColor.RED){
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }

        m_ledSubsytem.setPattern(pattern);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
