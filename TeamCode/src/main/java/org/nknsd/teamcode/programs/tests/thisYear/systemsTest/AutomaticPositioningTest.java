package org.nknsd.teamcode.programs.tests.thisYear.systemsTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.handlers.odometry.AbsolutePosition;
import org.nknsd.teamcode.components.handlers.vision.BasketLocator;
import org.nknsd.teamcode.components.handlers.vision.ID;
import org.nknsd.teamcode.components.handlers.launch.LaunchSystem;
import org.nknsd.teamcode.components.handlers.launch.LauncherHandler;
import org.nknsd.teamcode.components.handlers.artifact.MicrowavePositions;
import org.nknsd.teamcode.components.handlers.artifact.MicrowaveScoopHandler;
import org.nknsd.teamcode.components.handlers.artifact.SlotTracker;
import org.nknsd.teamcode.components.handlers.vision.TargetingSystem;
import org.nknsd.teamcode.components.handlers.launch.TrajectoryHandler;
import org.nknsd.teamcode.components.handlers.color.BallColor;
import org.nknsd.teamcode.components.handlers.color.BallColorInterpreter;
import org.nknsd.teamcode.components.handlers.color.ColorReader;
import org.nknsd.teamcode.components.motormixers.AbsolutePowerMixer;
import org.nknsd.teamcode.components.motormixers.AutoPositioner;
import org.nknsd.teamcode.components.motormixers.MecanumMotorMixer;
import org.nknsd.teamcode.components.motormixers.PowerInputMixer;
import org.nknsd.teamcode.components.sensors.AprilTagSensor;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.sensors.LEDIndicator;
import org.nknsd.teamcode.components.utility.RobotVersion;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;

import java.util.List;

@TeleOp(name = "automatic launching tester", group = "Tests")
public class AutomaticPositioningTest extends NKNProgram {

    TargetingSystem targetingSystem = new TargetingSystem();
    PowerInputMixer powerInputMixer = new PowerInputMixer();

    class IntakeState extends StateMachine.State {

        private final MicrowavePositions slot;
        private final MicrowaveScoopHandler microwaveScoopHandler;
        private final SlotTracker slotTracker;
        private final LaunchSystem launchSystem;
        private final int slotNum;

        IntakeState(int slotNum, MicrowaveScoopHandler microwaveScoopHandler, SlotTracker slotTracker, LaunchSystem launchSystem) {
            this.launchSystem = launchSystem;
            switch (slotNum) {
                case 1:
                    slot = MicrowavePositions.LOAD1;
                    break;
                case 2:
                    slot = MicrowavePositions.LOAD2;
                    break;
                default:
                    slot = MicrowavePositions.LOAD0;
            }
            this.microwaveScoopHandler = microwaveScoopHandler;
            this.slotTracker = slotTracker;
            this.slotNum = slotNum;
        }


        @Override
        protected void run(ElapsedTime runtime, Telemetry telemetry) {
            if (microwaveScoopHandler.isDone()) {
                if (slotTracker.getSlotColor(slotNum) == BallColor.GREEN || slotTracker.getSlotColor(slotNum) == BallColor.PURPLE) {
                    StateMachine.INSTANCE.stopAnonymous(this);
                }
            }
            if (targetingSystem.getDistance() != -1) {
                launchSystem.setDistance(targetingSystem.getDistance());
            }
        }

        @Override
        protected void started() {
            microwaveScoopHandler.setMicrowavePosition(slot);
            microwaveScoopHandler.toggleIntake(true);
        }

        @Override
        protected void stopped() {
            if (slotNum != 2) {
                StateMachine.INSTANCE.startAnonymous(new IntakeState(slotNum + 1, microwaveScoopHandler, slotTracker, launchSystem));
            } else {
                StateMachine.INSTANCE.startAnonymous(new LaunchState(0, microwaveScoopHandler, launchSystem, slotTracker));
            }

        }
    }

    class LaunchState extends StateMachine.State {

        private boolean hasLaunched = false;

        private final MicrowavePositions slot;
        private final MicrowaveScoopHandler microwaveScoopHandler;
        private final LaunchSystem launchSystem;
        private final SlotTracker slotTracker;
        private final int slotNum;

        LaunchState(int slotNum, MicrowaveScoopHandler microwaveScoopHandler, LaunchSystem launchSystem, SlotTracker slotTracker) {
            this.launchSystem = launchSystem;
            this.slotTracker = slotTracker;
            switch (slotNum) {
                case 1:
                    slot = MicrowavePositions.FIRE1;
                    break;
                case 2:
                    slot = MicrowavePositions.FIRE2;
                    break;
                default:
                    slot = MicrowavePositions.FIRE0;
            }
            this.microwaveScoopHandler = microwaveScoopHandler;
            this.slotNum = slotNum;
        }

        @Override
        protected void run(ElapsedTime runtime, Telemetry telemetry) {
            if (targetingSystem.targetVisible()) {
                launchSystem.setDistance(targetingSystem.getDistance());
            } else powerInputMixer.setManualPowers(new double[]{0, 0, 0});
            if (!hasLaunched && microwaveScoopHandler.isDone() && launchSystem.isReady() && targetingSystem.targetAcquired()) {
                microwaveScoopHandler.doScoopLaunch();
                hasLaunched = true;
            }
            if (hasLaunched && microwaveScoopHandler.isDone()) {
                slotTracker.clearSlot(slotNum);
                targetingSystem.enableAutoTargeting(false);
                powerInputMixer.setManualPowers(new double[]{0, 0, 0});
                StateMachine.INSTANCE.stopAnonymous(this);
            }
        }

        @Override
        protected void started() {
            microwaveScoopHandler.toggleIntake(false);
            targetingSystem.enableAutoTargeting(true);
            microwaveScoopHandler.setMicrowavePosition(slot);
        }

        @Override
        protected void stopped() {
            if (slotNum != 2) {
                StateMachine.INSTANCE.startAnonymous(new LaunchState(slotNum + 1, microwaveScoopHandler, launchSystem, slotTracker));
            } else {
                StateMachine.INSTANCE.startAnonymous(new IntakeState(0, microwaveScoopHandler, slotTracker, launchSystem));
            }
        }
    }


    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {

        components.add(StateMachine.INSTANCE);
        telemetryEnabled.add(StateMachine.INSTANCE);


//      leds
        LEDIndicator leftLED = new LEDIndicator("ledleftgreen", "ledleftred");
        LEDIndicator rightLED = new LEDIndicator("ledrightgreen", "ledrightred");


        components.add(leftLED);
        components.add(rightLED);


        TrajectoryHandler trajectoryHandler = new TrajectoryHandler();
        components.add(trajectoryHandler);
        telemetryEnabled.add(trajectoryHandler);

        LauncherHandler launcherHandler = new LauncherHandler(0.95, 1.10);
        components.add(launcherHandler);
        telemetryEnabled.add(launcherHandler);
        launcherHandler.setEnabled(true);

        targetingSystem.addLEDs(leftLED,rightLED);

        LaunchSystem launchSystem = new LaunchSystem(RobotVersion.INSTANCE.launchSpeedInterpolater, RobotVersion.INSTANCE.launchAngleInterpolater, 3, 16, 132);


        MicrowaveScoopHandler microwaveScoopHandler = new MicrowaveScoopHandler();
        components.add(microwaveScoopHandler);

        SlotTracker slotTracker = new SlotTracker();
        components.add(slotTracker);
        telemetryEnabled.add(slotTracker);

        ColorReader colorReader = new ColorReader("ColorSensor");
        components.add(colorReader);
        BallColorInterpreter ballColorInterpreter = new BallColorInterpreter(10, 0.01);
        components.add(ballColorInterpreter);


        FlowSensor flowSensor1 = new FlowSensor("RODOS");
        components.add(flowSensor1);
        FlowSensor flowSensor2 = new FlowSensor("LODOS");
        components.add(flowSensor2);
        AbsolutePosition absolutePosition = new AbsolutePosition(flowSensor1, flowSensor2);
        components.add(absolutePosition);
        telemetryEnabled.add(absolutePosition);

        MecanumMotorMixer mecanumMotorMixer = new MecanumMotorMixer();
        components.add(mecanumMotorMixer);
        telemetryEnabled.add(mecanumMotorMixer);

        AbsolutePowerMixer absolutePowerMixer = new AbsolutePowerMixer();
        components.add(absolutePowerMixer);
        absolutePowerMixer.link(mecanumMotorMixer, absolutePosition);

        AutoPositioner autoPositioner = new AutoPositioner();
        components.add(autoPositioner);

        components.add(powerInputMixer);


        AprilTagSensor aprilTagSensor = new AprilTagSensor();
        components.add(aprilTagSensor);
        telemetryEnabled.add(aprilTagSensor);

        BasketLocator basketLocator = new BasketLocator(RobotVersion.INSTANCE.aprilDistanceInterpolater);
        components.add(basketLocator);
        telemetryEnabled.add(basketLocator);

        components.add(targetingSystem);
        telemetryEnabled.add(targetingSystem);
        targetingSystem.setTargetingColor(ID.BLUE);


        slotTracker.link(microwaveScoopHandler, ballColorInterpreter);
        targetingSystem.link(basketLocator, absolutePosition, autoPositioner);
        basketLocator.link(aprilTagSensor);
        powerInputMixer.link(absolutePowerMixer, mecanumMotorMixer);
        ballColorInterpreter.link(colorReader);
        autoPositioner.link(powerInputMixer, absolutePosition);
        launchSystem.link(trajectoryHandler, launcherHandler);


        StateMachine.INSTANCE.startAnonymous(new IntakeState(0, microwaveScoopHandler, slotTracker, launchSystem));
    }
}
