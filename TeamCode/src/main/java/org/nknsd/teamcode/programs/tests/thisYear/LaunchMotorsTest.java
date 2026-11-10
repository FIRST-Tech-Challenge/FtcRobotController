package org.nknsd.teamcode.programs.tests.thisYear;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.handlers.launch.LaunchHandler;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;

import java.util.List;

@TeleOp(name = "Launch Motors Test", group = "Tests")
public class LaunchMotorsTest extends NKNProgram {

    static class StartLaunchState extends StateMachine.State {

        final LaunchHandler launchHandler;

        StartLaunchState(LaunchHandler launchHandler) {
            this.launchHandler = launchHandler;
        }
        boolean notStupid1 = true;
        boolean notStupid2 = true;
        boolean notStupid3 = true;
        boolean notStupid4 = true;
        @Override
        protected void run(ElapsedTime runtime, Telemetry telemetry) {

            if(notStupid3 && runtime.milliseconds() > 5000){
                launchHandler.setTargetTicks(2000);
                notStupid3 = false;
            }

            if(notStupid4 && runtime.milliseconds() > 10000){
                launchHandler.setTargetTicks(100000);
                notStupid4 = false;
            }
        }

        @Override
        protected void started() {
            launchHandler.setTargetTicks(1500);
            launchHandler.setEnabled(true);
        }

        @Override
        protected void stopped() {

        }
    }
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        LaunchHandler launchHandler = new LaunchHandler(0.95, 1.05);
        components.add(launchHandler);

        telemetryEnabled.add(launchHandler);

        components.add(StateMachine.INSTANCE);

        StateMachine.INSTANCE.startAnonymous(new StartLaunchState(launchHandler));
    }
}
