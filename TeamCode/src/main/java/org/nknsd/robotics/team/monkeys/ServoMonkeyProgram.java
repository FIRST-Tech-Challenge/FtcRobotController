package org.nknsd.robotics.team.monkeys;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.IntakeSpinnerHandler;
import org.nknsd.robotics.team.components.testfiles.ServoMonkey;

import java.util.List;

public class ServoMonkeyProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        IntakeSpinnerHandler intakeSpinnerHandler = new IntakeSpinnerHandler("intakeServo");
        components.add(intakeSpinnerHandler);
        telemetryEnabled.add(intakeSpinnerHandler);

        ServoMonkey servoMonkey = new ServoMonkey();
        servoMonkey.link(intakeSpinnerHandler);
        components.add(servoMonkey);
        telemetryEnabled.add(servoMonkey);
    }
}
