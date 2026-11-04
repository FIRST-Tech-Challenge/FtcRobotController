package org.nknsd.teamcode.programs.parts;

import org.nknsd.teamcode.components.handlers.lifting.BalancedLiftHandler;
import org.nknsd.teamcode.components.handlers.artifact.ArtifactSystem;
import org.nknsd.teamcode.components.handlers.artifact.MicrowaveScoopHandler;
import org.nknsd.teamcode.components.handlers.artifact.SlotTracker;
import org.nknsd.teamcode.components.handlers.color.BallColorInterpreter;
import org.nknsd.teamcode.components.handlers.color.ColorReader;
import org.nknsd.teamcode.components.handlers.launch.FiringSystem;
import org.nknsd.teamcode.components.handlers.launch.LaunchSystem;
import org.nknsd.teamcode.components.handlers.launch.LauncherHandler;
import org.nknsd.teamcode.components.handlers.launch.TrajectoryHandler;
import org.nknsd.teamcode.components.handlers.odometry.AbsolutePosition;
import org.nknsd.teamcode.components.handlers.srs.PeakFinder;
import org.nknsd.teamcode.components.handlers.srs.PeakPointer;
import org.nknsd.teamcode.components.handlers.srs.SRSHubHandler;
import org.nknsd.teamcode.components.handlers.vision.BasketLocator;
import org.nknsd.teamcode.components.handlers.vision.TargetingSystem;
import org.nknsd.teamcode.components.motormixers.AbsolutePowerMixer;
import org.nknsd.teamcode.components.motormixers.AutoPositioner;
import org.nknsd.teamcode.components.motormixers.MecanumMotorMixer;
import org.nknsd.teamcode.components.motormixers.PowerInputMixer;
import org.nknsd.teamcode.components.sensors.AprilTagSensor;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.sensors.IMUSensor;
import org.nknsd.teamcode.components.sensors.LEDIndicator;
import org.nknsd.teamcode.components.utility.RobotVersion;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.ProgramPart;

import java.util.List;

public class Setup extends ProgramPart {

    public LEDIndicator getLeftLED() {
        return leftLED;
    }

    public LEDIndicator getRightLED() {
        return rightLED;
    }

    private LEDIndicator leftLED, rightLED;

    private LaunchSystem launchSystem;
    private FiringSystem firingSystem;
    private ArtifactSystem artifactSystem;
    private PowerInputMixer powerInputMixer;
    private BalancedLiftHandler balancedLiftHandler;

    public PeakPointer getPeakPointer() {
        return peakPointer;
    }

    private PeakPointer peakPointer;

    public SRSHubHandler getSrsHubHandler() {
        return srsHubHandler;
    }

    private SRSHubHandler srsHubHandler;

    public SlotTracker getSlotTracker() {
        return slotTracker;
    }

    private SlotTracker slotTracker;

    public MicrowaveScoopHandler getMicrowaveScoopHandler() {
        return microwaveScoopHandler;
    }

    private MicrowaveScoopHandler microwaveScoopHandler;

    public PowerInputMixer getPowerInputMixer() {
        return powerInputMixer;
    }

    public AprilTagSensor getAprilTagSensor() {
        return aprilTagSensor;
    }

    private AprilTagSensor aprilTagSensor;

    public LaunchSystem getLaunchSystem() {
        return launchSystem;
    }

    public TargetingSystem getTargetingSystem() {
        return targetingSystem;
    }

    public AutoPositioner getAutoPositioner() {
        return autoPositioner;
    }

    public AbsolutePosition getAbsolutePosition() {
        return absolutePosition;
    }

    public ArtifactSystem getArtifactSystem() {
        return artifactSystem;
    }

    public FiringSystem getFiringSystem() {
        return firingSystem;
    }

    private AbsolutePosition absolutePosition;
    private AutoPositioner autoPositioner;
    private TargetingSystem targetingSystem;


    public BalancedLiftHandler getBalancedLiftHandler() {
        return balancedLiftHandler;
    }


    boolean scanOnStart = true, enableTelemetry = true;

    public void changeEnableSettings(boolean scanOnStart, boolean enableTelemetry) {
        this.scanOnStart = scanOnStart;
        this.enableTelemetry = enableTelemetry;
    }

    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {

//        statemachine
        components.add(StateMachine.INSTANCE);
        telemetryEnabled.add(StateMachine.INSTANCE);


//      leds
        leftLED = new LEDIndicator("ledleftgreen", "ledleftred");
        rightLED = new LEDIndicator("ledrightgreen", "ledrightred");
        components.add(leftLED);
        components.add(rightLED);


//        firing
        TrajectoryHandler trajectoryHandler = new TrajectoryHandler();
        components.add(trajectoryHandler);
        if (enableTelemetry) {
            telemetryEnabled.add(trajectoryHandler);
        }

        LauncherHandler launcherHandler = new LauncherHandler(0.95, 1.05);
        components.add(launcherHandler);
        if (enableTelemetry) {
            telemetryEnabled.add(launcherHandler);
        }
        launcherHandler.setEnabled(true);

        launchSystem = new LaunchSystem(RobotVersion.INSTANCE.launchSpeedInterpolater, RobotVersion.INSTANCE.launchAngleInterpolater, 0, 16, 132);

        firingSystem = new FiringSystem();
        components.add(firingSystem);
        if (enableTelemetry) {
            telemetryEnabled.add(firingSystem);
        }


//        microwave and artifact system
        ColorReader colorReader = new ColorReader("ColorSensor");
        components.add(colorReader);
        telemetryEnabled.add(colorReader);
        BallColorInterpreter ballColorInterpreter = new BallColorInterpreter(5, 0.005);
        components.add(ballColorInterpreter);

        slotTracker = new SlotTracker();
        components.add(slotTracker);
        telemetryEnabled.add(slotTracker);

        microwaveScoopHandler = new MicrowaveScoopHandler();
        components.add(microwaveScoopHandler);

        artifactSystem = new ArtifactSystem();


//        driving
        FlowSensor flowSensor1 = new FlowSensor("RODOS", RobotVersion.INSTANCE.rodosMult);
        components.add(flowSensor1);
        FlowSensor flowSensor2 = new FlowSensor("LODOS", RobotVersion.INSTANCE.lodosMult);
        components.add(flowSensor2);
        absolutePosition = new AbsolutePosition(flowSensor1, flowSensor2);
        components.add(absolutePosition);
        telemetryEnabled.add(absolutePosition);

        MecanumMotorMixer mecanumMotorMixer = new MecanumMotorMixer();
        components.add(mecanumMotorMixer);
        telemetryEnabled.add(mecanumMotorMixer);

        AbsolutePowerMixer absolutePowerMixer = new AbsolutePowerMixer();
        components.add(absolutePowerMixer);

        powerInputMixer = new PowerInputMixer();
        components.add(powerInputMixer);

        autoPositioner = new AutoPositioner();
        components.add(autoPositioner);


//        apriltag tracking
        aprilTagSensor = new AprilTagSensor();
        components.add(aprilTagSensor);
        if (enableTelemetry) {
            telemetryEnabled.add(aprilTagSensor);
        }

        BasketLocator basketLocator = new BasketLocator(RobotVersion.INSTANCE.aprilDistanceInterpolater);
        components.add(basketLocator);
//        basketLocator.setLEDs(leftLED, rightLED);
        basketLocator.setSkewEnabled(true);
        if (enableTelemetry) {
            telemetryEnabled.add(basketLocator);
        }

        targetingSystem = new TargetingSystem();
        components.add(targetingSystem);
        if (enableTelemetry) {
            telemetryEnabled.add(targetingSystem);
        }


//        lifting
        balancedLiftHandler = new BalancedLiftHandler();
        components.add(balancedLiftHandler);
        IMUSensor imuSensor = new IMUSensor(/*new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)*/);
        components.add(imuSensor);
        telemetryEnabled.add(balancedLiftHandler);

        if (RobotVersion.isAutonomous()) {
//        srs
        srsHubHandler = new SRSHubHandler();
        components.add(srsHubHandler);

        PeakFinder peakFinder = new PeakFinder();

        peakPointer = new PeakPointer(peakFinder, srsHubHandler, autoPositioner, absolutePosition);
        components.add(peakPointer);
        peakPointer.setLEDs(leftLED, rightLED);
        peakPointer.setPids(RobotVersion.INSTANCE.ballEatingPidH, RobotVersion.INSTANCE.ballEatingPidXY);

        telemetryEnabled.add(peakPointer);
        }


//        all links
        slotTracker.link(microwaveScoopHandler, ballColorInterpreter);
        targetingSystem.link(basketLocator, absolutePosition, autoPositioner);
        targetingSystem.addLEDs(leftLED, rightLED);
        basketLocator.link(aprilTagSensor);
        powerInputMixer.link(absolutePowerMixer, mecanumMotorMixer);
        absolutePowerMixer.link(mecanumMotorMixer, absolutePosition);
        ballColorInterpreter.link(colorReader);
        launchSystem.link(trajectoryHandler, launcherHandler);
        firingSystem.link(launchSystem, targetingSystem, artifactSystem);
        artifactSystem.link(microwaveScoopHandler, slotTracker, launchSystem);
        autoPositioner.link(powerInputMixer, absolutePosition);
        balancedLiftHandler.link(imuSensor);

        if (scanOnStart) {
            artifactSystem.scanAll();
        }

        if (RobotVersion.isAutonomous()) {
            autoPositioner.enableAutoPositioning(true, true, true);
        }
    }
}
