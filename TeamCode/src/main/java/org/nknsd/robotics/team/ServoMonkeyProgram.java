package org.nknsd.robotics.team;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.IntakeServoHandler;
import org.nknsd.robotics.team.components.testfiles.ExtensionMonkey;
import org.nknsd.robotics.team.components.testfiles.ServoMonkey;

import java.util.List;

public class ServoMonkeyProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        IntakeServoHandler intakeServoHandler = new IntakeServoHandler("intakeServo");
        components.add(intakeServoHandler);
        telemetryEnabled.add(intakeServoHandler);

        ServoMonkey servoMonkey = new ServoMonkey();
        servoMonkey.link(intakeServoHandler);
        components.add(servoMonkey);
        telemetryEnabled.add(servoMonkey);
    }
}
