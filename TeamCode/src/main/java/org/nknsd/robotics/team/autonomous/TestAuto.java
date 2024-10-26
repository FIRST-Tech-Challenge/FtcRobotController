package org.nknsd.robotics.team.autonomous;

import org.nknsd.robotics.framework.NKNAutoProgram;
import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.autonomous.steps.AutoStepAdjustTarget;
import org.nknsd.robotics.team.autonomous.steps.AutoStepD;
import org.nknsd.robotics.team.autonomous.steps.AutoStepL;
import org.nknsd.robotics.team.autonomous.steps.AutoStepMoveNRotate;
import org.nknsd.robotics.team.autonomous.steps.AutoStepR;
import org.nknsd.robotics.team.autonomous.steps.AutoStepRotateArm;
import org.nknsd.robotics.team.autonomous.steps.AutoStepServo;
import org.nknsd.robotics.team.autonomous.steps.AutoStepU;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.FlowSensorHandler;
import org.nknsd.robotics.team.components.IMUComponent;
import org.nknsd.robotics.team.components.IntakeServoHandler;
import org.nknsd.robotics.team.components.PotentiometerHandler;
import org.nknsd.robotics.team.components.RotationHandler;
import org.nknsd.robotics.team.components.WheelHandler;

import java.util.List;

public class TestAuto extends NKNAutoProgram {
    private AutoSkeleton autoSkeleton;

    @Override
    public void createSteps(List<NKNAutoStep> stepList) {
        //Move forward
        AutoStepU step0 = new AutoStepU(0.2);
        stepList.add(step0);


        //Deposit blue sample
        AutoStepL step1 = new AutoStepL(1.7);
        stepList.add(step1);


        //Head to c5
        AutoStepR step2 = new AutoStepR(0.95);
        stepList.add(step2);

        AutoStepU step3 = new AutoStepU(1.7);
        stepList.add(step3);


        //Align with middle sample and bring it down
        AutoStepL step4 = new AutoStepL(0.9);
        stepList.add(step4);

        AutoStepD step5 = new AutoStepD(1.85);
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
        AutoStepU step9 = new AutoStepU(1.65);
        stepList.add(step9);

        AutoStepMoveNRotate step10 = new AutoStepMoveNRotate(1.2, 0, 90);
        stepList.add(step10);

        AutoStepRotateArm step11 = new AutoStepRotateArm(RotationHandler.RotationPositions.HIGH);
        stepList.add(step11);

        AutoStepServo step12 = new AutoStepServo(-.1, 100);
        stepList.add(step12);

        //Next bit is for pseudo battlebots code (Dillon will know)
        //AutoStepR step6 = new AutoStepR(2);
        //stepList.add(step6);

        //AutoStepD step7 = new AutoStepD(0.15);
        //stepList.add(step7);

        //AutoStepR step8 = new AutoStepR(2.5);
        //stepList.add(step8);
        // WHYYY

        NKNAutoProgram.initSteps(stepList, autoSkeleton);
    }

    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Core mover
        autoSkeleton = new AutoSkeleton(0.35, 1.5, 0.5);

        // Sensors
        FlowSensorHandler flowSensorHandler = new FlowSensorHandler("sensor_otos", 0.590551, 3.54331, 0);
        components.add(flowSensorHandler);
        telemetryEnabled.add(flowSensorHandler);

        IMUComponent imuComponent = new IMUComponent();
        components.add(imuComponent);

        PotentiometerHandler potentiometerHandler = new PotentiometerHandler("armPot");
        components.add(potentiometerHandler);

        // Wheel Handler
        WheelHandler wheelHandler = new WheelHandler(
                "motorFL", "motorFR", "motorBL", "motorBR", new String[]{"motorFL", "motorBL"}
        );
        components.add(wheelHandler);
        telemetryEnabled.add(wheelHandler);

        // Arm Stuff
        RotationHandler rotationHandler = new RotationHandler ("motorArmRotate", 0.05, 0.38, 0.005, 10, true);
        components.add(rotationHandler);

        ExtensionHandler extensionHandler = new ExtensionHandler("motorArmExtend", true, 0.35);
        components.add(extensionHandler);

        IntakeServoHandler intakeServoHandler = new IntakeServoHandler("intakeServo");
        components.add(intakeServoHandler);

        // Linking
        rotationHandler.link(potentiometerHandler, extensionHandler);
        extensionHandler.link(rotationHandler);
        autoSkeleton.link(wheelHandler, rotationHandler, intakeServoHandler, flowSensorHandler, imuComponent);
    }
}
