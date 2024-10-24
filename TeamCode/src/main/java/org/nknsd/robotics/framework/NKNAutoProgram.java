package org.nknsd.robotics.framework;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.team.autonomous.AutoSkeleton;

import java.util.LinkedList;
import java.util.List;

public abstract class NKNAutoProgram extends NKNProgram{
    private final List<NKNComponent> componentList = new LinkedList<>();
    private final List<NKNComponent> enabledTelemetryList = new LinkedList<>();
    private final List<NKNAutoStep> stepList = new LinkedList<>();

    private AutoSkeleton autoSkeleton;
    private int currentStep = 0;
    private boolean done = false; //Better ways probably exist

    public abstract void createSteps(List<NKNAutoStep> stepList);

    private void initSteps(List<NKNAutoStep> stepList) {
        for (NKNAutoStep a : stepList) {
            a.link(autoSkeleton);
        }
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        createComponents(componentList, enabledTelemetryList);
        createSteps(stepList);
        initSteps(stepList);

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
        step.run(telemetry);

        if (step.isDone(runtime)) {
            currentStep ++;
        }

        if (currentStep >= stepList.size()) {
            done = true;
        }
    }
}
