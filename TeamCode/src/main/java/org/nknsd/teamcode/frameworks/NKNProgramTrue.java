package org.nknsd.teamcode.frameworks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.nknsd.teamcode.programs.tests.EACmonkeyProgram;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class NKNProgramTrue extends OpMode {
    private final List<NKNComponent> componentList = new LinkedList<>();
    private final List<NKNComponent> enabledTelemetryList = new LinkedList<>();
    private final ElapsedTime runtime = new ElapsedTime();
    private final long TELEMETRY_DELAY = 200;
    // We use these two to delay telemetry outputs to ~200 milliseconds
    //Time counters are in milliseconds
    private long lastTelemetryCall = 0;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        runtime.reset();
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        createComponents(componentList, enabledTelemetryList);

        // Report on the success of the component's initialization
        for (NKNComponent component:componentList){
            if (!component.init(telemetry,hardwareMap,gamepad1,gamepad2)){
                telemetry.addData("Status", "Failed on "+component.getName());
                telemetry.update();
                throw new NullPointerException("Failed to init "+component.getName());
            }
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Code to run REPEATEDLY after hitting INIT but before hitting PLAY
    @Override
    public void init_loop() {
        for (NKNComponent component:componentList){
            component.init_loop(runtime,telemetry);
        }
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
        for (NKNComponent component:componentList){
            component.start(runtime,telemetry);
        }
    }

    // Code to run REPEATEDLY after the driver hits PLAY
    // Does NOT handle telemetry
    @Override
    public void loop() {
        for (NKNComponent component:componentList){
            component.loop(runtime, telemetry);
        }

        if (runtime.now(TimeUnit.MILLISECONDS) - TELEMETRY_DELAY > lastTelemetryCall) {
            doTelemetry();
        }
    }

    // Runs once every ~500 milliseconds
    // ONLY calls for components to do telemetry
    public void doTelemetry() {
        for (NKNComponent component:componentList){
            if (isInTelemetryEnabledList(component)) {
                component.doTelemetry(telemetry);
            }
        }
        telemetry.update();
        lastTelemetryCall = runtime.now(TimeUnit.MILLISECONDS);
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        for (NKNComponent component:componentList){
            component.stop(runtime,telemetry);
        }
    }

    private boolean isInTelemetryEnabledList(NKNComponent component) {
        for (NKNComponent c: enabledTelemetryList) {
            if (c.equals(component)) {
                return true;
            }
        }
        return false;
    }

    // Fill in this function with your components to create the main list
    public abstract void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled);
}
