package org.nknsd.teamcode.programs.tests.thisYear;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.handlers.ConveyorHandler;
import org.nknsd.teamcode.components.handlers.artifact.MicrowavePositions;
import org.nknsd.teamcode.components.handlers.artifact.MicrowaveScoopHandler;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;
import org.nknsd.teamcode.programs.tests.thisYear.microwave.MicrowaveTester;

import java.util.List;

public class ConveyorTest extends NKNProgram {

    static class StartConveyorsState extends StateMachine.State {

        final ConveyorHandler conveyorHandler;

        StartConveyorsState(ConveyorHandler conveyorHandler) {
            this.conveyorHandler = conveyorHandler;
        }

        @Override
        protected void run(ElapsedTime runtime, Telemetry telemetry) {

        }

        @Override
        protected void started() {
            conveyorHandler.spinUp();
        }

        @Override
        protected void stopped() {

        }
    }

    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        ConveyorHandler conveyorHandler = new ConveyorHandler();
        components.add(conveyorHandler);
        telemetryEnabled.add(conveyorHandler);

        components.add(StateMachine.INSTANCE);

        StateMachine.INSTANCE.startAnonymous(new StartConveyorsState(conveyorHandler));
    }
}