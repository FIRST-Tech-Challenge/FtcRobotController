package org.nknsd.robotics.team.autonomous;

import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.autoSteps.AutoStepMove;
import org.nknsd.robotics.team.autoSteps.AutoStepMoveNRotate;
import org.nknsd.robotics.team.autoSteps.AutoStepRotateArm;
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
