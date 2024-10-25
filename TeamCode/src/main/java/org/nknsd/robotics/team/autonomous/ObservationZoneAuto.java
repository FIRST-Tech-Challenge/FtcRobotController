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

public class ObservationZoneAuto extends NKNAutoProgram {
    private AutoSkeleton autoSkeleton;

    @Override
    public void createSteps(List<NKNAutoStep> stepList) {
        //Move forward a little
        AutoStepU step0 = new AutoStepU(0.2);
        stepList.add(step0);

        //Move right into the obs zone
        AutoStepR step1 = new AutoStepR(1.5);
        stepList.add(step1);

        NKNAutoProgram.initSteps(stepList, autoSkeleton);
    }

    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Core mover
        autoSkeleton = new AutoSkeleton(0.35, 1.5);

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
