package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.StackHeight;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;

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
    public int visionProviderState = 0;
    public boolean visionProviderFinalized;
    public boolean enableTelemetry = true;
    public static final Class<? extends VisionProvider>[] visionProviders = VisionProviders.visionProviders;
    public static final Viewpoint viewpoint = Viewpoint.WEBCAM;
    public StackHeight height = StackHeight.HOLD_STATE;
    public int ugState = 0;
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
        height = gp;
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
//            vp.shutdownVision();
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

    private Constants.Position targetPose;

    public StateMachine AutoFullRed = getStateMachine(autoStage)
            //deploy intake without waiting on completion so gripper deploys simultaneously
            .addSingleState(() -> robot.intake.Do(Intake.Behavior.DEPLOY))

            .addState(() -> robot.deployWobbleGoalGripperAuton())
            .addState(() -> robot.launcher.wobbleGrip())
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> robot.launcher.setElbowTargetAngle(20))

            .addMineralState(ugStateProvider,
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_A_1, true, .8, .2),
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_B_1, true, .8, .2),
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_C_1, true, .8, .2))

//        .addMineralState(ugStateProvider,
//                ()-> robot.turret.setTurretAngle(90 + Constants.GRIPPER_HEADING_OFFSET),
//                ()-> robot.turret.setTurretAngle(270 + Constants.GRIPPER_HEADING_OFFSET),
//                ()-> robot.turret.setTurretAngle(35 + Constants.GRIPPER_HEADING_OFFSET))


            //release the wobble goal
            .addState(() -> robot.releaseWobbleGoalAuton(false))

            .addSingleState(() -> robot.launcher.gripperExtendABobTargetPos = Constants.GRIPPER_HALFWAY_POS)

            //.addSingleState(() -> robot.setTarget(Constants.Target.HIGH_GOAL))

            //back up
//        .addMineralState(ugStateProvider,
//                ()->true,
//                ()->true,
//                ()-> robot.driveToFieldPosition(Constants.Position.TARGET_C_3,false,  .8,.1)) //todo back up .5 meter
//                // driveIMUDistances seems broke for backwards ()-> robot.driveIMUDistance(.7,robot.getHeading(), false, -.5))

            //launch preferred since we can't seem to launch while driving away from goal at speed

            .addSingleState(() -> robot.turret.setDangerModeActive(true))

            .addMineralState(ugStateProvider,
                    () -> true,
                    () -> robot.turret.setTurretAngle(270),
                    () -> true)

            //WOBBLE TWO
            .addSimultaneousStates(
                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_APPROACH, false, .8, .2),
                    () -> robot.enterWobbleGoalMode()
            )

            .addState(() -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO, false, .8, .5))

            .addState(() -> robot.launcher.wobbleGrip())
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            .addSingleState(() -> Constants.IN_WOBBLE_MODE = false)
            .addSingleState(() -> robot.turret.setDangerModeActive(true))
//            .addSingleState(() -> robot.turret.setCurrentMode(Turret.TurretMode.fieldRelative))

            //ben is a coder 😎
//            .addMineralState(ugStateProvider,
//                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_EXIT, true, .8, .15),
//                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_EXIT_B, true, .8, .15),
//                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_EXIT, true, .8, .15))

            .addState(()-> robot.driveUntilXJank(1,true,Constants.startingXOffset,.05))


            .addState(()-> robot.launcher.setElbowTargetAngle(15))

            .addSingleState(() -> robot.launcher.gripperExtendABobTargetPos = Constants.GRIPPER_OUT_POS)

//            .addSimultaneousStates(
//                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_EXIT, true, .8, .15),
//                    () -> robot.launcher.setElbowTargetAngle(20)
//                            )

            .addSingleState(()-> robot.rotateIMU(0,1))

            //place wobble 2
            .addMineralState(ugStateProvider,
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_A_2, true, .8, .2),
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_B_2, true, .8, .2),
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_C_1, true, .8, .2))

            //spin up the flywheel
            .addSingleState(() -> robot.launcher.preSpinFlywheel(1000))

            .addState(() -> robot.releaseWobbleGoalAuton(true))

            .addSingleState(()-> robot.setTarget(Constants.Target.HIGH_GOAL))

            //lAUNCHING
            .addState(() -> robot.driveToFieldPosition(Constants.Position.LAUNCH_PREFERRED, false, 1, .1))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            .addState(()-> robot.rotateIMU(325,1))


            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            .addState(() -> robot.shootRingAuton(Constants.Target.HIGH_GOAL, 3))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            .addSingleState(()-> robot.gripperModeIsInReverse = false)
            .addSingleState(()-> robot.intake.alwaysASpinnin = false)

            //park
//            .addState(() -> robot.driveToFieldPosition(Constants.Position.NAVIGATE, true, .8, .6))
            .addSingleState(()-> robot.intake.Do(Intake.Behavior.TENT))
            //.addSingleState(() -> robot.intake.Do(Intake.Behavior.TENT))
            .addTimedState(5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .build();




    public StateMachine AutoFullBlue = getStateMachine(autoStage)
            //deploy intake without waiting on completion so gripper deploys simultaneously
            .addSingleState(() -> robot.intake.Do(Intake.Behavior.DEPLOY))
            .addSingleState(()-> robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE))

            .addState(() -> robot.deployWobbleGoalGripperAuton())
            .addState(() -> robot.launcher.wobbleGrip())
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> robot.launcher.setElbowTargetAngle(20))

            .addMineralState(ugStateProvider,
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_A_1_BLUE, true, .8, .2),
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_B_1, true, .8, .2),
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_C_1, true, .8, .2))

//        .addMineralState(ugStateProvider,
//                ()-> robot.turret.setTurretAngle(90 + Constants.GRIPPER_HEADING_OFFSET),
//                ()-> robot.turret.setTurretAngle(270 + Constants.GRIPPER_HEADING_OFFSET),
//                ()-> robot.turret.setTurretAngle(35 + Constants.GRIPPER_HEADING_OFFSET))


            //release the wobble goal
            .addState(() -> robot.releaseWobbleGoalAuton(false))
            .addSingleState(() -> robot.turret.setDangerModeActive(true))
            .addSingleState(()-> robot.turret.setHeading(270))

            //.addSingleState(() -> robot.setTarget(Constants.Target.HIGH_GOAL))

            //back up
//        .addMineralState(ugStateProvider,
//                ()->true,
//                ()->true,
//                ()-> robot.driveToFieldPosition(Constants.Position.TARGET_C_3,false,  .8,.1)) //todo back up .5 meter
//                // driveIMUDistances seems broke for backwards ()-> robot.driveIMUDistance(.7,robot.getHeading(), false, -.5))

            //launch preferred since we can't seem to launch while driving away from goal at speed

            //WOBBLE TWO
            .addSimultaneousStates(
                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_APPROACH_BLUE, false, .8, .2),
                    () -> robot.enterWobbleGoalMode()
            )

            .addState(() -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO, false, .8, .5))

            .addState(() -> robot.launcher.wobbleGrip())
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            .addSingleState(() -> Constants.IN_WOBBLE_MODE = false)
            .addSingleState(() -> robot.turret.setDangerModeActive(true))
//            .addSingleState(() -> robot.turret.setCurrentMode(Turret.TurretMode.fieldRelative))

            //ben is a coder 😎
            .addMineralState(ugStateProvider,
                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_EXIT_BLUE, true, .8, .15),
                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_EXIT_B, true, .8, .15),
                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_EXIT, true, .8, .15))

//            .addSimultaneousStates(
//                    () -> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_EXIT, true, .8, .15),
//                    () -> robot.launcher.setElbowTargetAngle(20)
//                            )

            .addSingleState(()-> robot.rotateIMU(0,1))

            //place wobble 2
            .addMineralState(ugStateProvider,
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_A_2, true, .8, .2),
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_B_2, true, .8, .2),
                    () -> robot.driveToFieldPosition(Constants.Position.TARGET_C_1, true, .8, .2))

            //spin up the flywheel
            .addSingleState(() -> robot.launcher.preSpinFlywheel(1000))

            .addState(() -> robot.releaseWobbleGoalAuton(true))

            .addSingleState(()-> robot.setTarget(Constants.Target.HIGH_GOAL))

            //lAUNCHING
            .addState(() -> robot.driveToFieldPosition(Constants.Position.LAUNCH_PREFERRED, false, 1, .1))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            .addState(() -> robot.shootRingAuton(Constants.Target.HIGH_GOAL, 3))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            .addSingleState(()-> robot.gripperModeIsInReverse = false)

            //park
            .addState(() -> robot.driveToFieldPosition(Constants.Position.NAVIGATE, true, .8, .6))

            //.addSingleState(() -> robot.intake.Do(Intake.Behavior.TENT))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .build();



    public StateMachine TurretCalibrate = getStateMachine(autoStage)
            .addSingleState(()-> robot.intake.Do(Intake.Behavior.DEPLOY))
            .addState(() -> robot.turret.calibrate())
            .build();

    public StateMachine DemoRollingRingtake = getStateMachine(autoStage)
            .addSingleState(()-> robot.intake.Do(Intake.Behavior.DEPLOY))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            //spin up the flywheel
            .addSingleState(() -> robot.launcher.preSpinFlywheel(900))
            .addState(()-> robot.driveToFieldPosition(Constants.Position.LAUNCH_ROLLERS,true,  .5,.1))
            .addSingleState(()-> robot.intake.Do(Intake.Behavior.TENT)) //initial tent to put it into rollingringtake
            .addState(() -> robot.shootRingAuton(Constants.Target.HIGH_GOAL,30))

            //the rest of this is just about safely returning proteus to starting position
            .addSingleState(()-> robot.setTarget(Constants.Target.NONE))
            .addSingleState(()-> robot.intake.setRollingRingMode(false))
            .addSingleState(()-> robot.intake.Do(Intake.Behavior.INTAKE))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(()-> robot.intake.Do(Intake.Behavior.TRAVEL))
            //todo: something still doesn't work right here. bottom servo jams. prolly need a custom RETURN_INIT behavior
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(()-> robot.intake.Do(Intake.Behavior.INITIALIZE))
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(()-> robot.driveToFieldPosition(Constants.Position.HOME, false, .4,.1))
            //todo add a robot.quiesce
            .build();

    public StateMachine AutoTest = getStateMachine(autoStage)
            .addSingleState(()-> robot.intake.Do(Intake.Behavior.DEPLOY))
            .addState(() -> robot.launcher.setElbowTargetAngle(0))
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(()-> robot.driveToFieldPosition(Constants.Position.TARGET_B_1, true, .5,.1))

            .addState(()-> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO_APPROACH, false, .5,.1))
            .addTimedState(4f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(()-> robot.driveToFieldPosition(Constants.Position.WOBBLE_TWO, false, .5,.1))
            .addTimedState(4f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(()-> robot.driveUntilXJank(1,false,Constants.startingXOffset,.05))
            .addTimedState(4f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))


            .build();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // //
    // Old Autonomous Routines //
    // //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private StateMachine.Builder getStateMachine(Stage stage) {
//        return
//                StateMachine.builder()
//                        .stateSwitchAction(() -> robot.launcher.setGripperPos(robot.launcher.toggleGripper()))
//                        .stateEndAction(() -> robot.turret.maintainHeadingTurret(false)).stage(stage);

            return StateMachine.builder()
                    .stateSwitchAction(() -> robot.returnTrue())
                    .stateEndAction(() -> {})
                    .stage(stage);

    }

    public void deinitVisionProvider() {
        telemetry.addData("Please wait", "Deinitializing vision");
        // telemetry.update();
//        robot.ledSystem.setColor(LEDSystem.Color.CALM);
        vp.shutdownVision();
        vp = null;
        visionProviderFinalized = false;
    }

    public void initVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            // telemetry.update();
//            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = visionProviders[visionProviderState].newInstance();
             vp.initializeVision(robot.hwMap, viewpoint, this.enableTelemetry);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    public void initDummyVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            // telemetry.update();
//            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = VisionProviders.defaultProvider.newInstance();
             vp.initializeVision(robot.hwMap, viewpoint, false);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }
}
