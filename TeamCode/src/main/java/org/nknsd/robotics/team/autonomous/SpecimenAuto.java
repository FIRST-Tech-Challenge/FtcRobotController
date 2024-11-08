package org.nknsd.robotics.team.autonomous;

import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.autoSteps.AutoStepAbsoluteControl;
import org.nknsd.robotics.team.autoSteps.AutoStepExtendArm;
import org.nknsd.robotics.team.autoSteps.AutoStepMove;
import org.nknsd.robotics.team.autoSteps.AutoStepMoveNRotate;
import org.nknsd.robotics.team.autoSteps.AutoStepRotateArm;
import org.nknsd.robotics.team.autoSteps.AutoStepServo;
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

public class SpecimenAuto extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Step List
        List<NKNAutoStep> stepList = new LinkedList<NKNAutoStep>();


        // Core mover
        AutoSkeleton autoSkeleton = new AutoSkeleton(0.3, 0.8, 1.5);

        AutoHeart autoHeart = new AutoHeart(stepList);
        components.add(autoHeart);
        telemetryEnabled.add(autoHeart);


        // Sensors
        FlowSensorHandler flowSensorHandler = new FlowSensorHandler();
        components.add(flowSensorHandler);
        telemetryEnabled.add(flowSensorHandler);

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
        // Declare steps
        AutoStepSleep sleep = new AutoStepSleep(200);

        AutoStepMove moveToBar = new AutoStepMove(0, 1.1);

        AutoStepRotateArm rotateToSpecimen = new AutoStepRotateArm(RotationHandler.RotationPositions.SPECIMEN);

        AutoStepExtendArm extendToSpecimen = new AutoStepExtendArm(ExtensionHandler.ExtensionPositions.SPECIMEN);

        AutoStepServo grip = new AutoStepServo(IntakeSpinnerHandler.HandStates.GRIP, 0);
        AutoStepServo release = new AutoStepServo(IntakeSpinnerHandler.HandStates.RELEASE, 500);



        // Create path
        // Approach bar and align arm
        stepList.add(moveToBar);
        stepList.add(rotateToSpecimen);
        stepList.add(grip);

        // Deposit specimen
        stepList.add(extendToSpecimen);
        stepList.add(sleep);
        stepList.add(release);

        autoHeart.linkSteps(stepList, autoSkeleton);
    }
}
