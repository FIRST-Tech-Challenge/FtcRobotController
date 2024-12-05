package org.nknsd.teamcode.programs.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.components.handlers.IntakeSpinnerHandler;
import org.nknsd.teamcode.components.testfiles.ServoMonkey;
import org.nknsd.teamcode.frameworks.NKNProgramTrue;

import java.util.List;

@TeleOp(name = "Servo Monkey", group="Tests") @Disabled
public class ServoMonkeyProgram extends NKNProgramTrue {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        IntakeSpinnerHandler intakeSpinnerHandler = new IntakeSpinnerHandler();
        components.add(intakeSpinnerHandler);
        telemetryEnabled.add(intakeSpinnerHandler);

        ServoMonkey servoMonkey = new ServoMonkey();
        servoMonkey.link(intakeSpinnerHandler);
        components.add(servoMonkey);
        telemetryEnabled.add(servoMonkey);
    }
}
