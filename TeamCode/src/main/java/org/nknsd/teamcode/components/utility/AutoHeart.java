package org.nknsd.teamcode.components.utility;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNAutoStep;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.helperClasses.AutoSkeleton;

import java.util.List;

public class AutoHeart implements NKNComponent {
    private final List<NKNAutoStep> stepList;
    private int currentStep = 0;
    private boolean done = false; //Better ways probably exist

    public AutoHeart(List<NKNAutoStep> stepList) {
        this.stepList = stepList;
    }

    public void linkSteps(List<NKNAutoStep> stepList, AutoSkeleton autoSkeleton) {
        for (NKNAutoStep a : stepList) {
            a.link(autoSkeleton);
        }
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        stepList.get(0).begin(runtime, telemetry);
    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "AutoHeart";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        if (done) {
            stop(runtime, telemetry);
            return;
        }

        NKNAutoStep step = stepList.get(currentStep);
        step.run(telemetry, runtime);

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
        telemetry.addData("Current step", stepList.get(currentStep).getName());
    }
}
