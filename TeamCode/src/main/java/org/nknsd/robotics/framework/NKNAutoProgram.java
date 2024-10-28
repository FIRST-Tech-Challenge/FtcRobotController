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

    private int currentStep = 0;
    private boolean done = false; //Better ways probably exist

    public abstract void createSteps(List<NKNAutoStep> stepList); // NEEDS to run initSteps on the program's end

    static public void initSteps(List<NKNAutoStep> stepList, AutoSkeleton autoSkeleton) {
        for (NKNAutoStep a : stepList) {
            a.link(autoSkeleton);
            //telemetry.addData(a.getName(), "Setup!");
        }
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        createComponents(componentList, enabledTelemetryList);
        createSteps(stepList);
        telemetry.update();

        // Report on the success of the component's initialization
        for (NKNComponent component:componentList){
            if (!component.init(telemetry,hardwareMap,gamepad1,gamepad2)){
                telemetry.addData("Status", "Failed on "+component.getName());
                telemetry.update();
                throw new NullPointerException("Failed to init "+component.getName());
            }
        }
    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        stepList.get(0).begin(runtime, telemetry);
    }

    // Code to run REPEATEDLY after the driver hits PLAY
    // Does NOT handle telemetry
    @Override
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
            if (currentStep >= stepList.size()) {
                done = true;
                return;
            }
            stepList.get(currentStep).begin(runtime, telemetry);
        }
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        for (NKNComponent component:componentList){
            if (isInTelemetryEnabledList(component)) {
                component.doTelemetry(telemetry);
            }
        }

        telemetry.addData("Current step", stepList.get(currentStep).getName());
    }
}
