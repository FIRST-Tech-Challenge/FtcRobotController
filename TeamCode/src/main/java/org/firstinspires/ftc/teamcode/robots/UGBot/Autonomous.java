package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.SkystoneVisionProvider;
import org.firstinspires.ftc.teamcode.vision.StonePos;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.VisionProvidersSkystone;

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
    public SkystoneVisionProvider vp;
    public int visionProviderState = 2;
    public boolean visionProviderFinalized;
    public boolean enableTelemetry = false;
    public static final Class<? extends SkystoneVisionProvider>[] visionProviders = VisionProvidersSkystone.visionProviders;
    public static final Viewpoint viewpoint = Viewpoint.WEBCAM;
    public int skystoneState = 1;
    private MineralStateProvider skystoneStateProvider = () -> skystoneState;

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
        StonePos gp = vp.detectSkystone().getQuarryPosition();
        // Hold state lets us know that we haven't finished looping through detection
        if (gp != StonePos.NONE_FOUND) {
            switch (gp) {
                case SOUTH:
                    skystoneState = 0;
                    break;
                case MIDDLE:
                    skystoneState = 1;
                    break;
                case NORTH:
                    skystoneState = 2;
                    break;
                case NONE_FOUND:
                case ERROR1:
                case ERROR2:
                case ERROR3:
                default:
                    skystoneState = 1;
                    break;
            }
            telemetry.addData("Vision Detection", "StonePos: %s", gp.toString());
            vp.shutdownVision();
            return true;
        } else {
            telemetry.addData("Vision Detection", "NONE_FOUND (still looping through internally)");
            return false;
        }
    }

    // public StateMachine visionTest = getStateMachine(autoStage)
    // .addState(() -> {
    // robot.xPos = robot.vps.detect();
    // return false;
    // })
    // .build();

    public StateMachine simultaneousStateTest = getStateMachine(autoStage).addSimultaneousStates(() -> {
        robot.turret.rotateRight(0.25);
        return false;
    }, () -> {
        robot.driveMixerDiffSteer(0.25, 0.25);
        return false;
    }).build();


    public StateMachine AutoFull = getStateMachine(autoStage)

            .build();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // //
    // Old Autonomous Routines //
    // //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private StateMachine.Builder getStateMachine(Stage stage) {
        return
                StateMachine.builder().stateSwitchAction(() -> robot.luncher.setGripperPos(robot.luncher.toggleGripper())) // resetMotors(true)
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
             vp.initializeVision(robot.hwMap, telemetry, enableTelemetry, viewpoint, !robot.isBlue);
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
            vp = VisionProvidersSkystone.defaultProvider.newInstance();
             vp.initializeVision(robot.hwMap, telemetry, enableTelemetry, viewpoint, !robot.isBlue);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    int stoneCount = 0;
    boolean[] quarryStones = new boolean[6];

    public int nextAutonStone(int firstStone) {
        if (stoneCount == 0) {
            quarryStones[firstStone] = true;
            stoneCount++;
            return firstStone;
        }
        if (stoneCount == 1) {
            quarryStones[firstStone + 3] = true;
            stoneCount++;
            return firstStone + 3;
        }
        if (stoneCount > 1 && stoneCount < 6) {
            for (int i = 0; i < quarryStones.length; i++) {
                if (!quarryStones[i]) // this is the first stone still false
                {
                    stoneCount++;
                    quarryStones[i] = true;
                    return i;
                }
            }

        }
        return -1; // this would be a fail
    }
}
