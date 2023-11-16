package org.firstinspires.ftc.teamcode.Robots;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.Components.Arm.ArmStates.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.Arm.ArmStates;
import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Clamp;
import org.firstinspires.ftc.teamcode.Components.FutureComponents.Extendo;
import org.firstinspires.ftc.teamcode.Components.FutureComponents.FlippingIntake;
import org.firstinspires.ftc.teamcode.Components.Hanger;
import org.firstinspires.ftc.teamcode.Components.Hopper;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.Preloader;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Components.Ultrasonics;
import org.firstinspires.ftc.teamcode.Components.Wrist;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/** Warren Robot class to contain all the season's functions */
public class BradBot extends BasicRobot {
  Arm arm;
  Clamp clamp;
  CVMaster cv;
  Hanger hanger;
  Intake intake;
  Launcher launcher;
  Lift lift;
  Preloader preloader;
  public SampleMecanumDrive roadrun;
  Ultrasonics ultras;
  Wrist wrist;

  /**
   * Instatiates all the hardware and sets up initial states of some software Logs that this
   * function is being called to general surface
   *
   * @param p_op opMode
   * @param p_is_Teleop is the program a teleop program
   */
  public BradBot(LinearOpMode p_op, boolean p_is_Teleop) {
    super(p_op, p_is_Teleop);
    LOGGER.setLogLevel(RFLogger.Severity.INFO);
    LOGGER.log("Initializing Components!");
    arm = new Arm();
    if (!isTeleop) {
      cv = new CVMaster();
    }
    clamp = new Clamp();
    hanger = new Hanger();
    intake = new Intake();
    launcher = new Launcher();
    lift = new Lift();
    preloader = new Preloader();
    roadrun = new SampleMecanumDrive(p_op.hardwareMap);
    //        ultras = new Ultrasonics();
    wrist = new Wrist();
  }

  public int getSpikePos() {
    return cv.getPosition() - 1;
  }

  public int getRightSpikePos() {
    return cv.getRightPosition() - 1;
  }

  public int getBlueSpikePos(){
    return cv.getBluePosition() -1;
  }
  public int getBlueRightSpikePos(){
    return cv.getBlueRightPosition()-1;
  }

  /**
   * starts the intake, for autonomous, intake.update() will handle the rest Logs that this function
   * called to general surface
   */
  public void startIntakeAuto() {
    if (queuer.queue(true, Intake.IntakeStates.STOPPED.getState())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("intaking until 2 pixels are stored");
      }
    }
  }

  public void preloadAuto() {
    if (queuer.queue(false, !preloader.getLoaded())) {
      if (!queuer.isExecuted()) {
        preloader.deposit();
        LOGGER.log("depositing preload");
      }
    }
  }

  public void loadAuto() {
    if (queuer.queue(true, preloader.getLoaded())) {
      if (!queuer.isExecuted()) {
        preloader.load();
        LOGGER.log("loading preload");
      }
    }
  }

  public void flipAuto() {
    if (queuer.queue(true, ArmStates.FLIPPED.getState())) {
      if (!queuer.isExecuted()) {
        clamp.clamp();
        arm.flipTo(FLIPPED);
        wrist.flipTo(Wrist.WristTargetStates.HOLD);
        lift.iterateUp();
      }
    }
  }

  public void resetAuto() {
    if (queuer.queue(true, !FLIPPED.getState())) {
      if (!queuer.isExecuted()) {
        lift.setPosition(Lift.LiftPositionStates.LOW_SET_LINE);
        arm.flipTo(UNFLIPPED);
        wrist.update();
        wrist.flipTo(Wrist.WristTargetStates.FLAT);
      }
    }
  }

  public void resetLift() {
    if (queuer.queue(true, lift.getCurrentPosition() < 5)) {
      lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
      lift.setPosition(0);
    }
  }

  public void dropWrist() {
    if (queuer.queue(false, Wrist.WristStates.DROP.getState())) {
      if (!queuer.isExecuted()) {
        wrist.flipTo(Wrist.WristTargetStates.DROP);
        //        clamp.unclamp();
      }
    }
  }

  public void drop() {
    if (queuer.queue(false, !clamp.getClamped())) {
      if (!queuer.isExecuted()) {
        //        wrist.flipTo(Wrist.WristTargetStates.DROP);
        clamp.unclamp();
      }
    }
  }

  /**
   * Empties the hopper in auto, hopper.update() will handle the rest Logs that this function called
   * to general surface
   */
  public void depositAuto() {
    if (queuer.queue(true, Hopper.HopperStates.ZERO.getState())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("depositing until 0 pixels remain");
      }
    }
  }

  /**
   * Calls other lift auto Logs that function is called
   *
   * @param p_liftPosition target position
   */
  public void liftAuto(Lift.LiftPositionStates p_liftPosition) {
    if (queuer.queue(true, lift.atTargetPosition())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("lifting to: " + p_liftPosition.name());
        liftAuto(p_liftPosition.getPosition());
      }
    }
  }

  /**
   * Auto lifts lift to this position, lift.update() will handle the rest Logs that this function
   * called to general surface
   *
   * @param p_position what position to go to
   */
  public void liftAuto(double p_position) {
    if (queuer.queue(true, lift.atTargetPosition())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("lifting to: " + p_position);
        liftAuto(p_position);
      }
    }
  }

  /**
   * follows inputted trajectory Logs that this function is called as well as initial and target
   * pose to general surface
   *
   * @param p_traj inputted trajectory
   */
  public void followTrajSeq(TrajectorySequence p_traj) {
    if (queuer.queue(false, !roadrun.isBusy())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("going from: " + p_traj.start() + " to: " + p_traj.end());
        roadrun.followTrajectorySequenceAsync(p_traj);
      }
    }
  }

  /** What is run each loop in teleOp Logs that this function is being called to general surface */
  public void teleOp() {
    boolean isA = gampad.readGamepad(op.gamepad2.a, "gamepad1_a", "resetOuttake");
    boolean rightBumper =
        gampad.readGamepad(op.gamepad1.right_bumper, "gamepad1_right_bumper", "startIntake");
    boolean leftBumper =
        gampad.readGamepad(op.gamepad1.left_bumper, "gamepad1_left_bumper", "reverseIntake");
    boolean isB = gampad.readGamepad(op.gamepad1.x, "gamepad1_x", "shoot");
    boolean isB2 = gampad.readGamepad(op.gamepad2.b, "gamepad2_b", "lockPower");

    boolean isX = gampad.readGamepad(op.gamepad1.x, "gamepad1_x", "toggleFieldCentricSlow");
    boolean isY = gampad.readGamepad(op.gamepad1.y, "gamepad1_y", "deposit");
    boolean up = gampad.readGamepad(op.gamepad2.dpad_up, "gamepad2_dpad_up", "lift Up");
    boolean down = gampad.readGamepad(op.gamepad2.dpad_down, "gamepad2_dpad_down", "lift down");
    boolean right =
        gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "toggleButterfly");
    boolean left = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "toggleClamp");

    boolean isX2 = gampad.readGamepad(op.gamepad2.x, "gamepad2_x", "toggleFieldCentricSlow");
    boolean isY2 = gampad.readGamepad(op.gamepad2.y, "gamepad2_y", "toggleFieldCentricSlow");

    float manualUp = op.gamepad1.right_trigger;
    float manualDown = op.gamepad1.left_trigger;
    float hangUp = op.gamepad2.right_trigger;
    float hangDown = op.gamepad2.left_trigger;
    if (isA) {
      arm.flipTo(UNFLIPPED);
      lift.update();
      wrist.update();
      wrist.flipTo(Wrist.WristTargetStates.FLAT);
      lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
      intake.stopIntake();
    }
    if (isB) {
      if (launcher.getLoaded()) launcher.shoot();
      else launcher.load();
    }
    if (isX2) {
      lift.resetPosition();
    }
    if (rightBumper) {
      if (Intake.IntakeStates.STOPPED.getState()) {
        if (Wrist.WristTargetStates.FLAT.state && Lift.LiftMovingStates.AT_ZERO.getState()) {
          wrist.flatten();
          arm.flatten();
          clamp.unclamp();
        }
        intake.intake();
      } else {
        if (Wrist.WristTargetStates.FLAT.state && Lift.LiftMovingStates.AT_ZERO.getState()) {
          clamp.clamp();
          wrist.unflatten();
          arm.unflatten();
        }
        intake.stopIntake();
      }
    }
    if (left) {
      if (clamp.getClamped()) clamp.unclamp();else clamp.clamp();
    }
    if (leftBumper) {
      intake.reverseIntake();
      clamp.clamp();
    }
    if (up) {
      if (Wrist.WristTargetStates.FLAT.state) {
        arm.flipTo(FLIPPED);
        wrist.flipTo(Wrist.WristTargetStates.HOLD);
      }
      lift.iterateUp();
      intake.stopIntake();
    }
    if (down) {
      if (Wrist.WristTargetStates.FLAT.state) {
        arm.flipTo(FLIPPED);
        wrist.flipTo(Wrist.WristTargetStates.HOLD);
      }
      lift.iterateDown();
      intake.stopIntake();
    }
    if (abs(op.gamepad2.left_stick_y) > 0.05) {
      lift.manualExtend(-op.gamepad2.left_stick_y);
    }
    //    if (abs(hangUp - hangDown) > 0.05) {
    hanger.setPower(hangUp - hangDown);
    //    }
    if (isY) {
      wrist.flipTo(Wrist.WristTargetStates.DROP);
    }
    if (isB2) {
      hanger.setPermaPower(hangUp - hangDown);
    }
    if (right) {
      roadrun.toggleButtered();
    }
    if (isX) {
      roadrun.toggleFieldCentric();
    }
    if (isX2) {
      preloader.deposit();
    }
    if (isY2) {
      preloader.load();
    }

    roadrun.setWeightedDrivePower(
        new Pose2d(
            -op.gamepad1.left_stick_y, -op.gamepad1.left_stick_x, -op.gamepad1.right_stick_x));
    update();
  }

  public void done() {
    queuer.done();
  }

  /**
   * updates the states of all the following Logs that this function is being called to surface
   * general log All else is logged in each respective function
   */
  public void update() {
    LOGGER.setLogLevel(RFLogger.Severity.FINER);
    LOGGER.log("updating each component");
    super.update();
    arm.update();
    if (!isTeleop) {
      cv.update();
    }
    intake.update();
    lift.update();
    roadrun.update();
    wrist.update();
  }

  public void stop() {
    LOGGER.log("the program has stopped normally");
    cv.stop();
  }
}
