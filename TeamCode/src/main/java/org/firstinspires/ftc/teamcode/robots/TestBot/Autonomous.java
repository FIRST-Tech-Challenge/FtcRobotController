package org.firstinspires.ftc.teamcode.robots.TestBot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.vision.VisionProvidersRoverRuckus;

/**
 * Class to keep all autonomous-related functions and state-machines in
 */
public class Autonomous {

    private PoseSkystone robot;
    private Telemetry telemetry;
    private Gamepad gamepad1;

    //vision-related configuration
    public VisionProvider vp;
    public int visionProviderState;
    public boolean visionProviderFinalized;
    public boolean enableTelemetry = false;
    public static final Class<? extends VisionProvider>[] visionProviders = VisionProvidersRoverRuckus.visionProviders;
    public static final Viewpoint viewpoint = Viewpoint.WEBCAM;
    public int mineralState = 0;
    private MineralStateProvider mineralStateProvider = () -> mineralState;

    //staging and timer variables
    public float autoDelay = 0;
    public Stage autoStage = new Stage();
    public Stage autoSetupStage = new Stage();

    //auto constants
    private static final double DRIVE_POWER = .65;
    private static final float TURN_TIME = 2;
    private static final float DUCKY_TIME = 1.0f;


    public Autonomous(PoseSkystone robot, Telemetry telemetry, Gamepad gamepad1) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }


    private boolean sample() {
        //Turn on camera to see which is gold
        GoldPos gp = vp.detect();
        // Hold state lets us know that we haven't finished looping through detection
        if (gp != GoldPos.HOLD_STATE) {
            switch (gp) {
                case LEFT:
                    mineralState = 0;
                    break;
                case MIDDLE:
                    mineralState = 1;
                    break;
                case RIGHT:
                    mineralState = 2;
                    break;
                case NONE_FOUND:
                case ERROR1:
                case ERROR2:
                case ERROR3:
                default:
                    mineralState = 1;
                    break;
            }
            telemetry.addData("Vision Detection", "GoldPos: %s", gp.toString());
            vp.shutdownVision();
            return true;
        } else {
            telemetry.addData("Vision Detection", "HOLD_STATE (still looping through internally)");
            return false;
        }
    }

    public StateMachine primaryBlue = getStateMachine(autoStage)
            .addState(() -> sample())
            .build();
    public StateMachine theWalkOfShame = getStateMachine(autoStage)//auto for an "in case all else fails"
            .addState(() -> (robot.driveForward(true, .7, .60)))//moves up
            .build();
    public StateMachine primaryBlueMec = getStateMachine(autoStage)
            .addState(() -> (robot.crane.setElbowTargetPos(2715,1.0)))//sets the gripper on the block
            .addState(() -> (robot.driveForward(true, .463, .60)))//moves to quarry line
            .addState(() -> (robot.crane.setElbowTargetPos(3200,1.0)))//sets the gripper on the block
            .addTimedState(3,//starts the gripper and pciks up the block
                    () -> robot.crane.grabStone(),
                    () -> robot.crane.stopGripper())
            .addState(() -> (robot.crane.setElbowTargetPos(3100,1.0)))//goes back up
            .addState(() -> (robot.rotateIMU(270, 12))) //rotate back toward building zone - stay in 2nd column
            .addState(() -> (robot.driveForward(true, 1.3, .60))) //todo: should continue until upward sensor detect transit of sky bridge - for now continuing until we are past bridge under odometry
            .addState(() ->robot.crane.setElbowTargetPos(2700,1.0)) //todo: should be an articulation to raise crane to current tower level - for now just raising a fair bit
            .addState(() -> (robot.driveForward(true, .6, .60))) //todo: should continue until proximity from back wall detected
            .addState(() -> (robot.rotateIMU(0, 9))) //turn to foundation
            .addState(() -> (robot.driveForward(true, .11, .60)))//goes towards the foundation
            .addTimedState(3,//starts the gripper and pciks up the block
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopGripper())
            .addState(() -> (robot.driveForward(false, .11, .60)))
            .addState(() -> (robot.rotateIMU(90, 9)))
            .addState(() -> (robot.crane.setElbowTargetPos(0,1.0)))//goes back up
            .addState(() -> (robot.driveForward(true, .9, .60)))
            //.addState(() -> (robot.rotateIMU(0, 9)))
            //.addState(() -> (robot.crane.setElbowTargetPos(2715,1.0)))//sets the gripper on the block
            //.addState(() -> (robot.driveForward(true, .463, .60)))//moves to quarry line
            //.addState(() -> (robot.crane.setElbowTargetPos(3200,1.0)))//sets the gripper on the block
            //.addTimedState(3,//starts the gripper and pciks up the block
                    //() -> robot.crane.grabStone(),
                    //() -> robot.crane.stopGripper())
            //.addState(() -> (robot.crane.setElbowTargetPos(3100,1.0)))
            .build();

    public StateMachine primaryRedMec = getStateMachine(autoStage)
            .addState(() -> (robot.crane.resetCraneEncoders()))
            .addState(() -> (robot.crane.setElbowTargetPos(2715,1.0)))//sets the gripper on the block
            .addState(() -> (robot.driveForward(true, .463, .60)))//moves to quarry line
            .addState(() -> (robot.crane.setElbowTargetPos(3200,1.0)))//sets the gripper on the block
            .addTimedState(3,//starts the gripper and pciks up the block
                    () -> robot.crane.grabStone(),
                    () -> robot.crane.stopGripper())
            .addState(() -> (robot.crane.setElbowTargetPos(3100,1.0)))//goes back up
            .addState(() -> (robot.rotateIMU(90, 12))) //rotate back toward building zone - stay in 2nd column
            .addState(() -> (robot.driveForward(true, 1.3, .60))) //todo: should continue until upward sensor detect transit of sky bridge - for now continuing until we are past bridge under odometry
            .addState(() ->robot.crane.setElbowTargetPos(2700,1.0)) //todo: should be an articulation to raise crane to current tower level - for now just raising a fair bit
            .addState(() -> (robot.driveForward(true, .6, .60))) //todo: should continue until proximity from back wall detected
            .addState(() -> (robot.rotateIMU(0, 9))) //turn to foundation
            .addState(() -> (robot.driveForward(true, .11, .60)))//goes towards the foundation
            .addTimedState(3,//starts the gripper and pciks up the block
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopGripper())
            .addState(() -> (robot.driveForward(false, .11, .60)))
            .addState(() -> (robot.rotateIMU(270, 9)))
            .addState(() -> (robot.crane.setElbowTargetPos(0,1.0)))//goes back up
            .addState(() -> (robot.driveForward(true, .9, .60)))
            //.addState(() -> (robot.rotateIMU(0, 9)))
            //.addState(() -> (robot.crane.setElbowTargetPos(2715,1.0)))//sets the gripper on the block
            //.addState(() -> (robot.driveForward(true, .463, .60)))//moves to quarry line
            //.addState(() -> (robot.crane.setElbowTargetPos(3200,1.0)))//sets the gripper on the block
            //.addTimedState(3,//starts the gripper and pciks up the block
            //() -> robot.crane.grabStone(),
            //() -> robot.crane.stopGripper())
            //.addState(() -> (robot.crane.setElbowTargetPos(3100,1.0)))
            .build();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                                  Old Autonomous Routines                                   //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public StateMachine autoSetupReverse = getStateMachine(autoSetupStage)
            .addTimedState(autoDelay, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.setAutonSingleStep(false)) //turn off autonSingleState
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.RED)) //red color
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reversedeploying)) //start deploy
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.reversedeployed) //wait until robot articulation in progress
            .addState(() -> sample()) //detect the mineral
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.reverseDriving) //wait until done
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE)) //purple color
            .addState(() -> robot.rotateIMU(0, 1)) //turn back to center
            .build();

    public StateMachine depotSide_worlds = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))
            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMid(1,10);})
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(80, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(135, 3)) //turn to crater
            .addState(() -> robot.crane.extendToMax(1,10))
            //.addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .build();

    public StateMachine depotSample_worlds = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))

            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .build();

    public StateMachine craterSide_worlds = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .02, .40)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMin(1,10);})
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(80, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(135, 3)) //turn to depot
            .addState(() -> robot.crane.extendToMax(1,10))
            .addState(() -> robot.crane.setElbowTargetPos(robot.crane.autodepotthingy, 1))
            .addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin())
            .addState(() -> robot.driveForward(false, .8, DRIVE_POWER))
            .build();

    public StateMachine depotSide_deposit = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .234, .40)))

            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .224, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(36, TURN_TIME),
                    () -> robot.rotateIMU(354, TURN_TIME),
                    () -> robot.rotateIMU(318, TURN_TIME))

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1350, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+900, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,10))
            .addState(() -> {robot.articulate(PoseSkystone.Articulation.reverseDepositAssisted); return robot.rotateIMU(0, 3);})
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addState(() -> robot.crane.extendToPosition(robot.crane.extendMid-50, 1, 10))
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addTimedState(1,
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addSingleState(() -> robot.crane.extendToMin(1,10))
            .addState(() -> robot.rotateIMU(80, 3)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, .8)) //drive to wall
            .addState(() -> robot.rotateIMU(135, 2)) //turn to crater
            .addState(() -> robot.crane.extendToMax(1,10))
            .build();

    public StateMachine craterSide_cycle = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .020, .40)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(37, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(323, TURN_TIME))

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.crane.stopIntake())
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(0, 1)) //turn to crater
            .addSingleState(() -> robot.crane.ejectStone())
            .addState(() -> robot.driveForward(true, .1, DRIVE_POWER))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseIntake))
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addState(() -> robot.crane.extendToMax())
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation. prereversedeposit))
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addState(() -> robot.rotateIMU(345, 3))
            .addState(() -> robot.driveForward(false, .1, DRIVE_POWER))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseDeposit))
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addTimedState(2,
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.rotateIMU(0, 1)) //turn to crater
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving))
//            .addSingleState(() -> robot.crane.eject())
//            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseIntake))
//            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
//            .addState(() -> robot.crane.extendToMax())
//            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation. prereversedeposit))
//            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
//            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseDeposit))
//            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
//            .addState(() -> robot.rotateIMU(345, 3))
//            .addTimedState(2,
//                    () -> robot.crane.eject(),
//                    () -> robot.crane.stopIntake())
//            .addState(() -> robot.rotateIMU(0, 4)) //turn to crater
            .build();


    public StateMachine autoSetup = getStateMachine(autoSetupStage)
            .addTimedState(autoDelay, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.setAutonSingleStep(false)) //turn off autonSingleState
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.RED)) //red color
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.deploying)) //start deploy
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.driving) //wait until done
            .addState(() -> robot.articulate(PoseSkystone.Articulation.driving, true))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE)) //purple color
            .addState(() -> robot.rotateIMU(0, 1)) //turn back to center
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            .addState(() -> robot.driveForward(false, .05, DRIVE_POWER)) //move back to see everything
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            .addState(() -> sample()) //detect the mineral
            .addState(() -> robot.driveForward(true, .05, DRIVE_POWER)) //move forward again
            .build();

    public StateMachine depotSide_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))

            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(80, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(120, 3)) //turn to crater
            .addState(() -> robot.crane.extendToMax(1,10))
            //.addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .build();

    public StateMachine depotSample_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))

            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .build();

    public StateMachine craterSide_extend_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            .addTimedState(.5f, () -> {}, () -> {})
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(85, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.1, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(120, 3)) //turn to depot
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))
            .addState(() -> robot.crane.setElbowTargetPos(10,1))
//            .addState(() -> robot.driveForward(true, .4, DRIVE_POWER))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
//            .addState(() -> robot.crane.extendToMax(1,10))
            .addSingleState(() -> robot.crane.setExtendABobTargetPos(robot.crane.extendMax))
            .addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .addState(() -> robot.crane.nearTargetExtend())
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .4, DRIVE_POWER))
            .addState(() -> robot.crane.nearTargetElbow())
            .addState(() -> robot.rotateIMU(34, 0.6))
            .addState(() -> robot.rotateIMU(310, 4))
            .addState(() -> robot.driveForward(true, 0.2, .8))
            .addState(() -> robot.crane.extendToMax())
            .build();

    public StateMachine craterSide_extend_reverse_team_marker = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> robot.rotateIMU(90, 4)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(128, 3)) //turn to depot
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))
            .addState(() -> robot.crane.setElbowTargetPos(10,1))
            .addState(() -> robot.driveForward(true, .4, DRIVE_POWER))
            .addState(() -> robot.crane.extendToPosition(robot.crane.extendMid+300,1,10))//extendToMin(1,10))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .4, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(90, 4)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(false, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.4, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            .addState(() -> robot.crane.extendToPosition(robot.crane.extendMid+400,1,10))
            .build();

    public StateMachine depotSide = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .604, DRIVE_POWER),
                    () -> robot.driveForward(true, .47, DRIVE_POWER),
                    () -> robot.driveForward(true, .604, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //turn to depot
                    () -> robot.rotateIMU(345, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(15, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to depot
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .762, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.crane.setElbowTargetPos(618, 1))
            .addState(() -> robot.crane.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addMineralState(mineralStateProvider, //turn to wall
                    () -> true,
                    () -> robot.rotateIMU(225, 4),
                    () -> robot.rotateIMU(225, 4))
            .addMineralState(mineralStateProvider, //move forward a little
                    () -> true,
                    () -> robot.driveForward(false, .090, DRIVE_POWER),
                    () -> robot.driveForward(false, .160, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(303, 5)) //turn to crater
            .addState(() -> robot.driveForward(false, 1.05, DRIVE_POWER)) //go to crater
            .addState(() -> robot.rotateIMU(310, 1.5)) //turn to crater
            .addState(() -> robot.driveForward(false, .80, DRIVE_POWER)) //go to grater
            .addSingleState(() -> robot.crane.setElbowTargetPos(robot.crane.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.crane.getElbowCurrentPos() - robot.crane.pos_AutoPark) < 20) //wait until done
            .build();

    public StateMachine depotSample = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .604, DRIVE_POWER),
                    () -> robot.driveForward(true, .47, DRIVE_POWER),
                    () -> robot.driveForward(true, .604, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //turn to depot
                    () -> robot.rotateIMU(345, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(15, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to depot
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .762, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.crane.setElbowTargetPos(618, 1))
            .addState(() -> robot.crane.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .build();

    public StateMachine craterSide_extend = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .70, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move back
                    () -> robot.driveForward(false, .440, DRIVE_POWER),
                    () -> robot.driveForward(false, .35, DRIVE_POWER),
                    () -> robot.driveForward(false, .445, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(270, 3)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(false, 1.43344, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.48988, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.9, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(310, 3)) //turn to depot
            .addState(() -> robot.articulate(PoseSkystone.Articulation.preIntake, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))
            .addState(() -> robot.crane.extendToMax(1,10))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.driving, true))
            .addState(() -> robot.crane.nearTargetElbow())
            .addState(() -> robot.rotateIMU(100, 0.6))
            .addState(() -> robot.rotateIMU(130, 3))
            .addState(() -> robot.driveForward(false, .5, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.preIntake, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))


            /*.addState(() -> robot.driveForward(true, 1.2, DRIVE_POWER)) //move to depot
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.crane.setElbowTargetPos(618, 1))
            .addState(() -> robot.crane.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.eject(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.driveForward(false, 2, DRIVE_POWER))
            .addSingleState(() -> robot.crane.setElbowTargetPos(robot.crane.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.crane.getElbowCurrentPos() - robot.crane.pos_AutoPark) < 20) //wait until done*/
            .build();

    private boolean resetIMUBool() {
        robot.resetIMU();
        return true;
    }

    public StateMachine craterSide = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .70, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move back
                    () -> robot.driveForward(false, .440, DRIVE_POWER),
                    () -> robot.driveForward(false, .35, DRIVE_POWER),
                    () -> robot.driveForward(false, .445, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(80, TURN_TIME)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.5, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.9, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(135, TURN_TIME)) //turn to depot
            .addState(() -> robot.driveForward(true, 1.2, DRIVE_POWER)) //move to depot
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.crane.setElbowTargetPos(618, 1))
            .addState(() -> robot.crane.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.driveForward(false, 2, DRIVE_POWER))
            .addSingleState(() -> robot.crane.setElbowTargetPos(robot.crane.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.crane.getElbowCurrentPos() - robot.crane.pos_AutoPark) < 20) //wait until done
            .build();

    public StateMachine craterSide_worlds_old = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            .addTimedState(.5f, () -> {}, () -> {})
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(85, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.5, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(135, 3)) //turn to depot
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))
            .addState(() -> robot.crane.setElbowTargetPos(10,1))
//            .addState(() -> robot.driveForward(true, .4, DRIVE_POWER))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
//            .addState(() -> robot.crane.extendToMax(1,10))
            .addSingleState(() -> robot.crane.setExtendABobTargetPos(robot.crane.extendMax))
            .addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .addState(() -> robot.crane.nearTargetExtend())
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .4, DRIVE_POWER))
            .addState(() -> robot.crane.nearTargetElbow())
            .addState(() -> robot.rotateIMU(34, 0.6))
            .addState(() -> robot.rotateIMU(315, 4))
            .addState(() -> robot.driveForward(true, 0.2, .8))
            .addState(() -> robot.crane.extendToMax())
            .build();

    public StateMachine driveStraight = getStateMachine(autoStage)
            .addState(() -> robot.driveForward(false, 4, DRIVE_POWER))
            .build();






    private StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> robot.resetMotors(true))
                .stateEndAction(() -> {})
                .stage(stage);
    }

    public void deinitVisionProvider() {
        telemetry.addData("Please wait", "Deinitializing vision");
        //telemetry.update();
        robot.ledSystem.setColor(LEDSystem.Color.CALM);
        vp.shutdownVision();
        vp = null;
        visionProviderFinalized = false;
    }

    public void initVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            //telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = visionProviders[visionProviderState].newInstance();
            vp.initializeVision(robot.hwMap, telemetry, enableTelemetry, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    public void initDummyVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            //telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = VisionProvidersRoverRuckus.defaultProvider.newInstance();
            vp.initializeVision(robot.hwMap, telemetry, enableTelemetry, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

}
