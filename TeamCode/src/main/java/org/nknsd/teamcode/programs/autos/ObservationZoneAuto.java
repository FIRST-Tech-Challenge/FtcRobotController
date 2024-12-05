package org.nknsd.teamcode.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.nknsd.teamcode.frameworks.NKNAutoStep;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.autoSteps.AutoStepMove;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.sensors.IMUSensor;
import org.nknsd.teamcode.components.handlers.IntakeSpinnerHandler;
import org.nknsd.teamcode.components.sensors.PotentiometerSensor;
import org.nknsd.teamcode.components.handlers.RotationHandler;
import org.nknsd.teamcode.components.handlers.WheelHandler;
import org.nknsd.teamcode.components.utility.AutoHeart;
import org.nknsd.teamcode.frameworks.NKNProgramTrue;
import org.nknsd.teamcode.helperClasses.AutoSkeleton;

import java.util.LinkedList;
import java.util.List;

@Autonomous(name = "Observation Zone Auto")
public class ObservationZoneAuto extends NKNProgramTrue {
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
        //Move forward a little
        AutoStepMove step0 = new AutoStepMove(0, 0.2);
        stepList.add(step0);

        //Move right into the obs zone
        AutoStepMove step1 = new AutoStepMove(1.5, 0);
        stepList.add(step1);

        autoHeart.linkSteps(stepList, autoSkeleton);
    }
}
