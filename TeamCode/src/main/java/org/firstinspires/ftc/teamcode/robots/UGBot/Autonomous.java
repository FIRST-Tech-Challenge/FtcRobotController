package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.StackHeight;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.SkystoneVisionProvider;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.VisionProvider;

/**
 * Class to keep all autonomous-related functions and state-machines in
 */
public class Autonomous {

    private PoseUG robot;
    private Telemetry telemetry;
    private Gamepad gamepad1;

    public static int sampleExtendMiddle = 2210;
    public static int sampleExtendLeft = 2200;
    public static int sampleExtendRight = 2200;
    public static boolean sampleContinue = false;

    // vision-related configuration
    public VisionProvider vp;
    public int visionProviderState = 2;
    public boolean visionProviderFinalized;
    public boolean enableTelemetry = false;
    public static final Class<? extends VisionProvider>[] visionProviders = VisionProviders.visionProviders;
    public static final Viewpoint viewpoint = Viewpoint.WEBCAM;
    public int ugState = 1;
    private MineralStateProvider ugStateProvider = () -> ugState;

    // staging and timer variables
    public float autoDelay = 0;
    public Stage autoStage = new Stage();
    public Stage autoSetupStage = new Stage();

    // auto constants
    private static final double DRIVE_POWER = .65;
    private static final float TURN_TIME = 2;
    private static final float DUCKY_TIME = 1.0f;

    public Autonomous(PoseUG robot, Telemetry telemetry, Gamepad gamepad1) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }

    public boolean sample() {
        // Turn on camera to see which is gold
        StackHeight gp = vp.detect();
        // Hold state lets us know that we haven't finished looping through detection
        if (gp != StackHeight.NONE_FOUND) {
            switch (gp) {
                case ZERO:
                    ugState = 0;
                    break;
                case FOUR:
                    ugState = 2;
                    break;
                case ONE:
                case NONE_FOUND:
                default:
                    ugState = 1;
                    break;
            }
            telemetry.addData("Vision Detection", "Stack Height: %s", gp.toString());
            vp.shutdownVision();
            return true;
        } else {
            telemetry.addData("Vision Detection", "NONE_FOUND (still looping through internally)");
            return false;
        }
    }

    public StateMachine simultaneousStateTest = getStateMachine(autoStage).addSimultaneousStates(() -> {
        robot.turret.rotateRight(0.25);
        return false;
    }, () -> {
        robot.driveMixerDiffSteer(0.25, 0.25);
        return false;
    }).build();


    public StateMachine AutoFull = getStateMachine(autoStage)
            // open and align gripper for 1st skystone
            .addState(() -> robot.driveIMUDistanceWithReset(.5,robot.getHeading(),true,.5))
            .addSingleState(() -> robot.turret.rotateCardinalTurret(true))
            .addState(() -> robot.launcher.extendToMax())
            .build();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // //
    // Old Autonomous Routines //
    // //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private StateMachine.Builder getStateMachine(Stage stage) {
        return
                StateMachine.builder().stateSwitchAction(() -> robot.launcher.setGripperPos(robot.launcher.toggleGripper())) // resetMotors(true)
                .stateEndAction(() -> robot.turret.maintainHeadingTurret(false)).stage(stage);
    }

    public void deinitVisionProvider() {
        telemetry.addData("Please wait", "Deinitializing vision");
        // telemetry.update();
        robot.ledSystem.setColor(LEDSystem.Color.CALM);
        vp.shutdownVision();
        vp = null;
        visionProviderFinalized = false;
    }

    public void initVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            // telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = visionProviders[visionProviderState].newInstance();
             vp.initializeVision(robot.hwMap, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    public void initDummyVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            // telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = VisionProviders.defaultProvider.newInstance();
             vp.initializeVision(robot.hwMap, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }
}
