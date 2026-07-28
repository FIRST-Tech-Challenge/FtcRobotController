package org.firstinspires.ftc.teamcode.common.autonomous;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs an ordered list of non-blocking autonomous steps one at a time.
 *
 * <p>Empty sequences finish immediately. Repeated start calls do nothing. Calling stop before
 * completion stops the active step and finishes the sequence. Updates before start or after
 * completion do nothing. The current step name is null when no step is active.</p>
 */
public class AutoSequence {
    private final List<AutoStep> steps = new ArrayList<>();
    private int currentStepIndex = -1;
    private boolean started;
    private boolean finished;

    public AutoSequence() {
        // Steps may be added before starting.
    }

    public AutoSequence(AutoStep... steps) {
        if (steps == null) {
            throw new IllegalArgumentException("An autonomous sequence needs a step array.");
        }
        for (AutoStep step : steps) {
            addStep(step);
        }
    }

    public void addStep(AutoStep step) {
        if (step == null) {
            throw new IllegalArgumentException("An autonomous sequence cannot add a null step.");
        }
        if (started) {
            throw new IllegalStateException("Add autonomous steps before starting the sequence.");
        }
        steps.add(step);
    }

    public void start() {
        if (started) {
            return;
        }
        started = true;
        if (steps.isEmpty()) {
            finished = true;
            return;
        }
        currentStepIndex = 0;
        getCurrentStep().start();
    }

    public void update() {
        if (!started || finished) {
            return;
        }
        AutoStep currentStep = getCurrentStep();
        currentStep.update();
        if (!currentStep.isFinished()) {
            return;
        }
        currentStep.stop();
        currentStepIndex++;
        if (currentStepIndex >= steps.size()) {
            currentStepIndex = -1;
            finished = true;
            return;
        }
        getCurrentStep().start();
    }

    public void stop() {
        if (!started || finished) {
            return;
        }
        getCurrentStep().stop();
        currentStepIndex = -1;
        finished = true;
    }

    public boolean isFinished() {
        return finished;
    }

    public String getCurrentStepName() {
        return currentStepIndex < 0 ? null : getCurrentStep().getName();
    }

    private AutoStep getCurrentStep() {
        return steps.get(currentStepIndex);
    }
}
