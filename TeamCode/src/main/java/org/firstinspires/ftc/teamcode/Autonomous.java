package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.SkystoneGripPipeline;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.vision.VisionProvidersRoverRuckus;

/**
 * Class to keep all autonomous-related functions and state-machines in
 */
public class Autonomous {

    private PoseBigWheel robot;
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


    public Autonomous(PoseBigWheel robot, Telemetry telemetry, Gamepad gamepad1) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }


    public StateMachine autoSetupReverse = getStateMachine(autoSetupStage)
            .addTimedState(autoDelay, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.setAutonSingleStep(false)) //turn off autonSingleState
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.RED)) //red color
            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation.reversedeploying)) //start deploy
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.reversedeployed) //wait until robot articulation in progress
            .addState(() -> sample()) //detect the mineral
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.reverseDriving) //wait until done
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE)) //purple color
            .addState(() -> robot.rotatePIDIMU(0, 1)) //turn back to center
            .build();

    public StateMachine depotSide_worlds = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> robot.collector.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMin(1,15))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,0,1,1))
            //.addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+800, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> {robot.collector.stopIntake(); return robot.collector.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.pos_reverseSafeDrive,1,1))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotatePIDIMU(80, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotatePIDIMU(135, 3)) //turn to crater
            .addState(() -> robot.collector.extendToMax(1,10))
            //.addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .build();

    public StateMachine depotSample_worlds = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> robot.collector.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMin(1,15))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,0,1,1))
            //.addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+800, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> {robot.collector.stopIntake(); return robot.collector.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.pos_reverseSafeDrive,1,1))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .build();

    public StateMachine craterSide_worlds = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .02, .40)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,0,1,1))
            //.addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10),
                    () -> robot.collector.extendToPosition(robot.collector.extendMid+800, 1, 10),
                    () -> robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10))
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> {robot.collector.stopIntake(); return robot.collector.extendToMin(1,10);})
            //.addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.pos_reverseSafeDrive,1,1))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotatePIDIMU(80, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotatePIDIMU(135, 3)) //turn to depot
            .addState(() -> robot.collector.extendToMax(1,10))
            .addState(() -> robot.collector.setElbowTargetPos(robot.collector.autodepotthingy, 1))
            .addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMin())
            .addState(() -> robot.driveForward(false, .8, DRIVE_POWER))
            .build();

    public StateMachine depotSide_deposit = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .234, .40)))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> robot.collector.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMin(1,15))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> (robot.driveForward(false, .224, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(36, TURN_TIME),
                    () -> robot.rotatePIDIMU(354, TURN_TIME),
                    () -> robot.rotatePIDIMU(318, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,0,1,1))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1350, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+900, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMin(1,10))
            .addState(() -> {robot.articulate(PoseBigWheel.Articulation.reverseDepositAssisted); return robot.rotatePIDIMU(0, 3);})
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addState(() -> robot.collector.extendToPosition(robot.collector.extendMid-50, 1, 10))
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.pos_reverseSafeDrive,1,1))
            .addTimedState(1,
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addSingleState(() -> robot.collector.extendToMin(1,10))
            .addState(() -> robot.rotatePIDIMU(80, 3)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, .8)) //drive to wall
            .addState(() -> robot.rotatePIDIMU(135, 2)) //turn to crater
            .addState(() -> robot.collector.extendToMax(1,10))
            .build();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                                  Old Autonomous Routines                                   //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public StateMachine craterSide_cycle = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .020, .40)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(37, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(323, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,0,1,1))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+800, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.collector.stopIntake())
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotatePIDIMU(0, 1)) //turn to crater
            .addSingleState(() -> robot.collector.eject())
            .addState(() -> robot.driveForward(true, .1, DRIVE_POWER))
            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation.reverseIntake))
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addState(() -> robot.collector.extendToMax())
            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation. prereversedeposit))
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addState(() -> robot.rotatePIDIMU(345, 3))
            .addState(() -> robot.driveForward(false, .1, DRIVE_POWER))
            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDeposit))
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addTimedState(2,
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.rotatePIDIMU(0, 1)) //turn to crater
            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving))
//            .addSingleState(() -> robot.collector.eject())
//            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation.reverseIntake))
//            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
//            .addState(() -> robot.collector.extendToMax())
//            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation. prereversedeposit))
//            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
//            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDeposit))
//            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
//            .addState(() -> robot.rotatePIDIMU(345, 3))
//            .addTimedState(2,
//                    () -> robot.collector.eject(),
//                    () -> robot.collector.stopIntake())
//            .addState(() -> robot.rotatePIDIMU(0, 4)) //turn to crater
            .build();


    public StateMachine lagTest = getStateMachine(autoStage)
            .addState(() -> {
                robot.driveMixerTank(.65,0);
                return gamepad1.a;
            })
            .addState(() -> robot.driveForward(false, 1, .65))
            .addState(() -> resetIMUBool())
            .build();

    public StateMachine autoSetup = getStateMachine(autoSetupStage)
            .addTimedState(autoDelay, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.setAutonSingleStep(false)) //turn off autonSingleState
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.RED)) //red color
            .addSingleState(() -> robot.articulate(PoseBigWheel.Articulation.deploying)) //start deploy
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.driving) //wait until done
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.driving, true))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE)) //purple color
            .addState(() -> robot.rotatePIDIMU(0, 1)) //turn back to center
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            .addState(() -> robot.driveForward(false, .05, DRIVE_POWER)) //move back to see everything
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            .addState(() -> sample()) //detect the mineral
            .addState(() -> robot.driveForward(true, .05, DRIVE_POWER)) //move forward again
            .build();

    public StateMachine depotSide_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> robot.collector.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMin(1,15))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,0,1,1))
            //.addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+800, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> {robot.collector.stopIntake(); return robot.collector.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.pos_reverseSafeDrive,1,1))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotatePIDIMU(80, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotatePIDIMU(120, 3)) //turn to crater
            .addState(() -> robot.collector.extendToMax(1,10))
            //.addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .build();

    public StateMachine depotSample_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> robot.collector.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMin(1,15))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.autodepotthingy,1,1))
            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,0,1,1))
            //.addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+800, 1, 10);},
                    () -> { robot.collector.eject(); return robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> {robot.collector.stopIntake(); return robot.collector.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseDeposit, robot.collector.pos_reverseSafeDrive,1,1))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .build();

    public StateMachine craterSide_extend_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,0,1,1))
            //.addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10),
                    () -> robot.collector.extendToPosition(robot.collector.extendMid+800, 1, 10),
                    () -> robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10))
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving,true))
            .addTimedState(.5f, () -> {}, () -> {})
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotatePIDIMU(85, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.1, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotatePIDIMU(120, 3)) //turn to depot
            //.addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true))
            .addState(() -> robot.collector.setElbowTargetPos(10,1))
//            .addState(() -> robot.driveForward(true, .4, DRIVE_POWER))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
//            .addState(() -> robot.collector.extendToMax(1,10))
            .addSingleState(() -> robot.collector.setExtendABobTargetPos(robot.collector.extendMax))
            .addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .addState(() -> robot.collector.nearTargetExtend())
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .4, DRIVE_POWER))
            .addState(() -> robot.collector.nearTargetElbow())
            .addState(() -> robot.rotatePIDIMU(34, 0.6))
            .addState(() -> robot.rotatePIDIMU(310, 4))
            .addState(() -> robot.driveForward(true, 0.2, .8))
            .addState(() -> robot.collector.extendToMax())
            .build();

    public StateMachine craterSide_extend_reverse_team_marker = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> robot.rotatePIDIMU(90, 4)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER))
            .addState(() -> robot.rotatePIDIMU(128, 3)) //turn to depot
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true))
            .addState(() -> robot.collector.setElbowTargetPos(10,1))
            .addState(() -> robot.driveForward(true, .4, DRIVE_POWER))
            .addState(() -> robot.collector.extendToPosition(robot.collector.extendMid+300,1,10))//extendToMin(1,10))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMid(1,10))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .4, DRIVE_POWER))
            .addState(() -> robot.rotatePIDIMU(90, 4)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(false, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.4, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,40,1,1))
            .addState(() -> robot.collector.extendToPosition(robot.collector.extendMid+400,1,10))
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
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.collector.setElbowTargetPos(618, 1))
            .addState(() -> robot.collector.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
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
            .addSingleState(() -> robot.collector.setElbowTargetPos(robot.collector.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.collector.getElbowCurrentPos() - robot.collector.pos_AutoPark) < 20) //wait until done
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
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.collector.setElbowTargetPos(618, 1))
            .addState(() -> robot.collector.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .build();

    public StateMachine craterSide_extend = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .70, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move back
                    () -> robot.driveForward(false, .440, DRIVE_POWER),
                    () -> robot.driveForward(false, .35, DRIVE_POWER),
                    () -> robot.driveForward(false, .445, DRIVE_POWER))
            .addState(() -> robot.rotatePIDIMU(270, 3)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(false, 1.43344, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.48988, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.9, DRIVE_POWER))
            .addState(() -> robot.rotatePIDIMU(310, 3)) //turn to depot
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.preIntake, true))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true))
            .addState(() -> robot.collector.extendToMax(1,10))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMid(1,10))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.driving, true))
            .addState(() -> robot.collector.nearTargetElbow())
            .addState(() -> robot.rotatePIDIMU(100, 0.6))
            .addState(() -> robot.rotatePIDIMU(130, 3))
            .addState(() -> robot.driveForward(false, .5, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.preIntake, true))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true))


            /*.addState(() -> robot.driveForward(true, 1.2, DRIVE_POWER)) //move to depot
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.collector.setElbowTargetPos(618, 1))
            .addState(() -> robot.collector.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.driveForward(false, 2, DRIVE_POWER))
            .addSingleState(() -> robot.collector.setElbowTargetPos(robot.collector.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.collector.getElbowCurrentPos() - robot.collector.pos_AutoPark) < 20) //wait until done*/
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
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.collector.setElbowTargetPos(618, 1))
            .addState(() -> robot.collector.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.eject(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.driveForward(false, 2, DRIVE_POWER))
            .addSingleState(() -> robot.collector.setElbowTargetPos(robot.collector.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.collector.getElbowCurrentPos() - robot.collector.pos_AutoPark) < 20) //wait until done
            .build();

    public StateMachine craterSide_worlds_old = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotatePIDIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotatePIDIMU(321, TURN_TIME))
            .addState(() -> robot.goToPosition(robot.superman.pos_reverseIntake,0,1,1))
            //.addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10),
                    () -> robot.collector.extendToPosition(robot.collector.extendMid+800, 1, 10),
                    () -> robot.collector.extendToPosition(robot.collector.extendMid+1300, 1, 10))
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving,true))
            .addTimedState(.5f, () -> {}, () -> {})
            .addState(() -> robot.getArticulation() == PoseBigWheel.Articulation.manual)
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotatePIDIMU(85, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.5, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotatePIDIMU(135, 3)) //turn to depot
            //.addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.manual, true))
            .addState(() -> robot.collector.setElbowTargetPos(10,1))
//            .addState(() -> robot.driveForward(true, .4, DRIVE_POWER))
            .addSingleState(() -> robot.collector.setBeltToElbowModeEnabled())
//            .addState(() -> robot.collector.extendToMax(1,10))
            .addSingleState(() -> robot.collector.setExtendABobTargetPos(robot.collector.extendMax))
            .addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .addState(() -> robot.collector.nearTargetExtend())
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.collector.collect(),
                    () -> robot.collector.stopIntake())
            .addState(() -> robot.collector.extendToMid(1,10))
            .addSingleState(() -> robot.collector.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseBigWheel.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .4, DRIVE_POWER))
            .addState(() -> robot.collector.nearTargetElbow())
            .addState(() -> robot.rotatePIDIMU(34, 0.6))
            .addState(() -> robot.rotatePIDIMU(315, 4))
            .addState(() -> robot.driveForward(true, 0.2, .8))
            .addState(() -> robot.collector.extendToMax())
            .build();

    public StateMachine driveStraight = getStateMachine(autoStage)
            .addState(() -> robot.driveForward(false, 4, DRIVE_POWER))
            .build();




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
