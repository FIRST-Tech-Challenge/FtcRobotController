package org.nknsd.teamcode.programs.autos;

import org.nknsd.teamcode.frameworks.NKNAutoStep;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;
import org.nknsd.teamcode.autoSteps.AutoStepMove;
import org.nknsd.teamcode.autoSteps.AutoStepMoveNRotate;
import org.nknsd.teamcode.autoSteps.AutoStepRotateArm;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.sensors.IMUSensor;
import org.nknsd.teamcode.components.handlers.IntakeSpinnerHandler;
import org.nknsd.teamcode.components.sensors.PotentiometerSensor;
import org.nknsd.teamcode.components.handlers.RotationHandler;
import org.nknsd.teamcode.components.handlers.WheelHandler;
import org.nknsd.teamcode.components.utility.AutoHeart;

import java.util.LinkedList;
import java.util.List;

public class PushAuto extends NKNProgram {

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
        FlowSensor flowSensor = new FlowSensor();
        components.add(flowSensor);
        telemetryEnabled.add(flowSensor);

        IMUSensor imuSensor = new IMUSensor();
        components.add(imuSensor);
        //telemetryEnabled.add(imuComponent);

        PotentiometerSensor potentiometerSensor = new PotentiometerSensor();
        components.add(potentiometerSensor);


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
        rotationHandler.link(potentiometerSensor, extensionHandler);
        extensionHandler.link(rotationHandler);
        autoSkeleton.link(wheelHandler, rotationHandler, extensionHandler, intakeSpinnerHandler, flowSensor, imuSensor);
        assembleList(stepList, autoHeart, autoSkeleton);
    }

    private void assembleList(List<NKNAutoStep> stepList, AutoHeart autoHeart, AutoSkeleton autoSkeleton) {
        //Move forward
        AutoStepMove step0 = new AutoStepMove(0, 0.2);
        stepList.add(step0);

        //Deposit blue sample
        AutoStepMove step1 = new AutoStepMove(-0.45, 0);
        stepList.add(step1);


        //Head to c5
        AutoStepMove step2 = new AutoStepMove(0.62, 0);
        stepList.add(step2);

        AutoStepMove step3 = new AutoStepMove(0, 1.5);
        stepList.add(step3);


        //Align with middle sample and bring it down
        AutoStepMove step4 = new AutoStepMove(-0.66, 0);
        stepList.add(step4);

        AutoStepMove step5 = new AutoStepMove(0, -1.4);
        stepList.add(step5);


//        //Head up and bring right sample down
//        AutoStepU step6 = new AutoStepU(1.6);
//        stepList.add(step6);
//
//        AutoStepR step7 = new AutoStepR(0.35);
//        stepList.add(step7);
//
//        AutoStepAdjustTarget step7_5 = new AutoStepAdjustTarget(-0.2, 0);
//        stepList.add(step7_5);
//
//        AutoStepD step8 = new AutoStepD(1.5);
//        stepList.add(step8);


        //Head to observation zone
        AutoStepMoveNRotate step9 = new AutoStepMoveNRotate(0, 1.45, 90);
        stepList.add(step9);

        AutoStepRotateArm lowerArm = new AutoStepRotateArm(RotationHandler.RotationPositions.PREPICKUP);
        stepList.add(lowerArm);

        AutoStepMove step11 = new AutoStepMove(1.1, 0);
        stepList.add(step11);

        AutoStepRotateArm raiseArm = new AutoStepRotateArm(RotationHandler.RotationPositions.HIGH);
        stepList.add(raiseArm);


        autoHeart.linkSteps(stepList, autoSkeleton);
    }
}
