package org.nknsd.robotics.framework;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

public abstract class NKNAutoProgram extends NKNProgram{
    private final List<NKNComponent> componentList = new LinkedList<>();
    private final List<NKNComponent> enabledTelemetryList = new LinkedList<>();
    private final List<NKNAutoStep> stepList = new LinkedList<>();

    abstract void createSteps(List<NKNAutoStep> stepList);
    private int currentStep = 0;
    private boolean done = false; //Better ways probably exist

    private void initSteps(List<NKNAutoStep> stepList, List<NKNComponent> componentList) {
        HashMap<String, NKNComponent> componentHashMap = new HashMap<>();
        for (NKNComponent c : componentList) {
            componentHashMap.put(c.getName(), c);
        }

        for (NKNAutoStep a : stepList) {
            a.link(componentHashMap);
        }
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        createComponents(componentList, enabledTelemetryList);
        createSteps(stepList);
        initSteps(stepList, componentList);

        // Report on the success of the component's initialization
        for (NKNComponent component:componentList){
            if (!component.init(telemetry,hardwareMap,gamepad1,gamepad2)){
                telemetry.addData("Status", "Failed on "+component.getName());
                telemetry.update();
                throw new NullPointerException("Failed to init "+component.getName());
            }
        }
    }

    // Code to run REPEATEDLY after the driver hits PLAY
    // Does NOT handle telemetry
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        if (done) {
            stop(runtime, telemetry);
            return;
        }

        for (NKNComponent component:componentList){
            component.loop(runtime, telemetry);
        }

        NKNAutoStep step = stepList.get(currentStep);
        step.run();

        if (step.isDone()) {
            currentStep ++;
        }

        if (currentStep >= stepList.size()) {
            done = true;
        }
    }
}
