package org.nknsd.robotics.team.autonomous;

import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.autoSteps.AutoStepMove;
import org.nknsd.robotics.team.autoSteps.AutoStepMoveNRotate;
import org.nknsd.robotics.team.autoSteps.AutoStepSleep;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.FlowSensorHandler;
import org.nknsd.robotics.team.components.IMUComponent;
import org.nknsd.robotics.team.components.IntakeSpinnerHandler;
import org.nknsd.robotics.team.components.PotentiometerHandler;
import org.nknsd.robotics.team.components.RotationHandler;
import org.nknsd.robotics.team.components.WheelHandler;
import org.nknsd.robotics.team.components.autonomous.AutoHeart;

import java.util.LinkedList;
import java.util.List;

public class TestingAuto extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Step List
        List<NKNAutoStep> stepList = new LinkedList<NKNAutoStep>();


        // Core mover
        AutoSkeleton autoSkeleton = new AutoSkeleton(0.7, 0.2, 1.5);

        AutoHeart autoHeart = new AutoHeart(stepList);
        components.add(autoHeart);
        //telemetryEnabled.add(autoHeart);


        // Sensors
        FlowSensorHandler flowSensorHandler = new FlowSensorHandler();
        components.add(flowSensorHandler);
        //telemetryEnabled.add(flowSensorHandler);

        IMUComponent imuComponent = new IMUComponent();
        components.add(imuComponent);
        //telemetryEnabled.add(imuComponent);

        PotentiometerHandler potentiometerHandler = new PotentiometerHandler();
        components.add(potentiometerHandler);


        // Wheel Handler
        WheelHandler wheelHandler = new WheelHandler();
        components.add(wheelHandler);


        // Arm Stuff
        RotationHandler rotationHandler = new RotationHandler ();
        components.add(rotationHandler);

        ExtensionHandler extensionHandler = new ExtensionHandler();
        components.add(extensionHandler);

        IntakeSpinnerHandler intakeSpinnerHandler = new IntakeSpinnerHandler();
        components.add(intakeSpinnerHandler);


        // Linking
        rotationHandler.link(potentiometerHandler, extensionHandler);
        extensionHandler.link(rotationHandler);
        autoSkeleton.link(wheelHandler, rotationHandler, extensionHandler, intakeSpinnerHandler, flowSensorHandler, imuComponent);
        assembleList(stepList, autoHeart, autoSkeleton);
    }

    private void assembleList(List<NKNAutoStep> stepList, AutoHeart autoHeart, AutoSkeleton autoSkeleton) {
        AutoStepMove right1 = new AutoStepMove(1, 0);
        AutoStepMove left1 = new AutoStepMove(-1, 0);
        AutoStepMove up1 = new AutoStepMove(0, 1);
        AutoStepMove down1 = new AutoStepMove(0, -1);
        AutoStepMoveNRotate turn = new AutoStepMoveNRotate(0, 0, 90);
        AutoStepSleep sleep = new AutoStepSleep(500);

        stepList.add(up1);
        stepList.add(turn);
        stepList.add(sleep);
        stepList.add(right1);
        stepList.add(turn);

        autoHeart.linkSteps(stepList, autoSkeleton);
    }
}
