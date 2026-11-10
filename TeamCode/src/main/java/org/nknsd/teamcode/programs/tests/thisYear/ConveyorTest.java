package org.nknsd.teamcode.programs.tests.thisYear;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.handlers.ConveyorHandler;
import org.nknsd.teamcode.components.handlers.IntakeHandler;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;

import java.util.List;

@TeleOp(name = "ConveyorServoTest", group = "Tests")
public class ConveyorTest extends NKNProgram {

    static class StartConveyorsState extends StateMachine.State {

        final ConveyorHandler conveyorHandler;
        final IntakeHandler intakeHandler;

        StartConveyorsState(ConveyorHandler conveyorHandler, IntakeHandler intakeHandler) {
            this.conveyorHandler = conveyorHandler;
            this.intakeHandler = intakeHandler;
        }

        @Override
        protected void run(ElapsedTime runtime, Telemetry telemetry) {

        }

        @Override
        protected void started() {
            conveyorHandler.startUp();
            intakeHandler.setIntakePower(.5);
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

        IntakeHandler intakeHandler = new IntakeHandler();
        components.add(intakeHandler);
        telemetryEnabled.add(intakeHandler);

        components.add(StateMachine.INSTANCE);

        StateMachine.INSTANCE.startAnonymous(new StartConveyorsState(conveyorHandler, intakeHandler));
    }
}