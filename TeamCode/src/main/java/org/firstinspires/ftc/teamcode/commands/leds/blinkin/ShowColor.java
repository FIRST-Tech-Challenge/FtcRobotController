package org.firstinspires.ftc.teamcode.commands.leds.blinkin;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;

public class ShowColor extends CommandBase {

    private final LEDSubsystem ledSubsytem;
    private final RevBlinkinLedDriver.BlinkinPattern pattern;


    public ShowColor(LEDSubsystem subsystem, RevBlinkinLedDriver.BlinkinPattern pat){
        ledSubsytem = subsystem;
        pattern = pat;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

        ledSubsytem.setPattern(pattern);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
