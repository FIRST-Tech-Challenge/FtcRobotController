package org.firstinspires.ftc.teamcode.Robots;

import static org.apache.commons.math3.util.FastMath.PI;
import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.cos;
import static org.apache.commons.math3.util.FastMath.floor;
import static org.apache.commons.math3.util.FastMath.max;
import static org.apache.commons.math3.util.FastMath.min;
import static org.firstinspires.ftc.teamcode.Components.Arm.ArmStates.*;
import static org.firstinspires.ftc.teamcode.Components.Arm.DROP_POS;
import static org.firstinspires.ftc.teamcode.Components.Magazine.pixels;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.types.PathType;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.Arm.ArmStates;
import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.Hanger;
import org.firstinspires.ftc.teamcode.Components.Hopper;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.Preloader;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFGamepad;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Components.Twrist;
import org.firstinspires.ftc.teamcode.Components.Ultrasonics;
import org.firstinspires.ftc.teamcode.Components.Wrist;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.secret.PPUI;
import org.firstinspires.ftc.teamcode.roadrunner.secret.Waypoint;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

/** Warren Robot class to contain all the season's functions */
public class BradBot extends BasicRobot {
  Arm arm;
  Claw claw;
  CVMaster cv;
  Hanger hanger;
  Intake intake;
  Launcher launcher;
  Lift lift;
  Magazine magazine;

  boolean purped = false;
  boolean pathFin = false;
  boolean gapped = false;
  boolean brokenFollowing = false;
  //  double startIntake = -100;
  //  Preloader preloader;
  public SampleMecanumDrive roadrun;
  Twrist twrist;
  Ultrasonics ultras;
  Wrist wrist;
  Path path;

  PPUI ppui;
  double voltage = 12;
  boolean intaked = false;

  MecanumDrive drive;

  /**
   * Instatiates all the hardware and sets up initial states of some software Logs that this
   * function is being called to general surface
   *
   * @param p_op opMode
   * @param p_is_Teleop is the program a teleop program
   */
  public BradBot(LinearOpMode p_op, boolean p_is_Teleop, boolean isLogi) {
    this(p_op, p_is_Teleop, isLogi, false);
  }
  public BradBot(LinearOpMode p_op, boolean p_is_Teleop, boolean isLogi, boolean isFlipped){
      super(p_op, p_is_Teleop, isFlipped);
      LOGGER.setLogLevel(RFLogger.Severity.INFO);
      LOGGER.log("Initializing Components!");
      arm = new Arm();
      if (!isTeleop) {
          cv = new CVMaster(isLogi);
      }
      claw = new Claw();
      magazine = new Magazine();
      hanger = new Hanger();
      intake = new Intake();
      launcher = new Launcher();
      lift = new Lift();
      //    preloader = new Preloader();
      roadrun = new SampleMecanumDrive(p_op.hardwareMap, Tracker.TrackType.ROADRUN_ODOMETRY);
      twrist = new Twrist();
      ultras = new Ultrasonics();
      wrist = new Wrist();
      //    path = new Path();
      purped = false;
      //    ppui = new PPUI(roadrun);
      voltage = voltageSensor.getVoltage();
      brokenFollowing = false;

      MAX_ACCEL*=voltage / 15;
      kV *= 12.89/voltage;
      // 12.4,12.2
      update();
  }

  public BradBot(LinearOpMode p_op, boolean p_isTeleop) {
    this(p_op, p_isTeleop, false);
  }

  public int getSpikePos() {
    return cv.getPosition();
  }

  public int getRightSpikePos() {
    return cv.getRightPosition() - 1;
  }

  public int getBlueSpikePos() {
    return cv.getBluePosition() - 1;
  }

  public int getBlueRightSpikePos() {
    return cv.getBlueRightPosition() - 1;
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
    if (queuer.queue(false, true)) {
      if (!queuer.isExecuted()) {
        //        preloader.deposit();
        LOGGER.log("depositing preload");
      }
    }
  }

  public void purpurAuto() {
    if (queuer.queue(true, purped)) {
      if (lift.getCurrentPosition() > 600) {
        arm.purpurPigzl();
      }
      if (DROP.state) {
        Lift.LiftMovingStates.LOW.state = false;
        lift.setPosition(400);
        wrist.purpur();
        purped = true;
        LOGGER.log("purpuring");
      }
    }
  }

  public void purpurAuto2() {
    if (queuer.queue(true, purped)) {
      if (lift.getCurrentPosition() > 600) {
        arm.purpurPigzl2();
      }
      if (DROP.state) {
        Lift.LiftMovingStates.LOW.state = false;
        lift.setPosition(400);
        wrist.purpur();
        purped = true;
        LOGGER.log("purpuring");
      }
    }
  }

  public void dropServo(int servo) {
    if (servo == 1) {
      claw.moveOne(false);
    } else {
      claw.moveTwo(false);
    }
  }

  public void yellowAuto(boolean left) {
    if (queuer.queue(
        true, Twrist.twristStates.DROP.getState() || Twrist.twristStates.OT.getState())) {
      if (currentPose.getX() > -10 && DROP.getState()) {
        if (left) {
          lift.setPosition(530);
        } else {
          lift.setPosition(530);
        }
        if (left) {
          twrist.flipTo(Twrist.twristTargetStates.OT);
        } else {
          twrist.flipTo(Twrist.twristTargetStates.DROP);
        }
        //        if(Wrist.WristStates.LOCK.getState()){
        //          wrist.flipTo(Wrist.WristTargetStates.GRAB);
        //        }
        wrist.flipTo(Wrist.WristTargetStates.DROP);
        LOGGER.log("ocook");
      }
    }
  }

  public void setRight(boolean right) {
    cv.setRight(right);
  }

  public void observeSpike() {
    cv.observeSpike();
  }

  public void dropAuto(int servo) {
    if (queuer.queue(false, queuer.isNextExecuted())) {
      //      if (!queuer.isExecuted()) {
      if (servo == 1) {
        claw.moveOne(false);
      } else {
        claw.moveTwo(false);
      }
      Claw.clawStates.CLOSE.setStateTrue();
      gapped = false;

      LOGGER.log("dropping claw " + servo);
    }
    //    }
  }

  public void hoverArm() {
    arm.flipTo(HOVER);
  }

  public void upAuto() {
    if (queuer.queue(true, lift.getCurrentPosition() > 500)) {
      if (!Lift.LiftMovingStates.LOW.state) {
        lift.setPosition(Lift.LiftPositionStates.LOW_SET_LINE);
        intake.stopIntake();
        if (Wrist.WristStates.LOCK.getState()) {
          wrist.flipTo(Wrist.WristTargetStates.GRAB);
        }
        arm.flipTo(DROP);
        wrist.flipTo(Wrist.WristTargetStates.DROP);
        LOGGER.log("ocrud");
      }
    }
  }

  public void veryLowAuto() {
    if (queuer.queue(true, Wrist.WristStates.DROP.getState())) {
      if (currentPose.getX() > 9) {
        lift.setPosition(860);
        intake.stopIntake();
        arm.flipTo(DROP);
        //        if(Wrist.WristStates.LOCK.getState()){
        //          wrist.flipTo(Wrist.WristTargetStates.GRAB);
        //        }
        wrist.flipTo(Wrist.WristTargetStates.DROP);
        LOGGER.log("ocook");
      }
    }
  }

  public void outtakePreload() {
    if (queuer.queue(true, intake.getHeight() == 5)) {}
  }

  public void lowAuto(boolean left) {
    if (queuer.queue(true, Wrist.WristStates.DROP.getState() || lift.getTarget() == 970)) {
      LOGGER.log(
          "armor"
              + Claw.clawStates.GRAB.getState()
              + " "
              + Arm.ArmTargetStates.HOVER.getState()
              + " "
              + (currentPose.getX() > -15));
      if (currentPose.getX() > -17 && Claw.clawStates.GRAB.getState()) {
        lift.setPosition(970);
        intake.stopIntake();
        arm.flipTo(DROP);
        wrist.flipTo(Wrist.WristTargetStates.DROP);
        twrist.flipTo(Twrist.twristTargetStates.DROP);
        LOGGER.log("ocook");
      }
    }
  }
  public void lessLowAuto(boolean left) {
    if (queuer.queue(true, Wrist.WristStates.DROP.getState() || lift.getTarget() == 1200)) {
      LOGGER.log(
              "armor"
                      + Claw.clawStates.GRAB.getState()
                      + " "
                      + Arm.ArmTargetStates.HOVER.getState()
                      + " "
                      + (currentPose.getX() > -15));
      if (currentPose.getX() > -20 && Claw.clawStates.GRAB.getState()) {
        lift.setPosition(1200);
        intake.stopIntake();
        arm.flipTo(DROP);
        wrist.flipTo(Wrist.WristTargetStates.DROP);
        twrist.flipTo(Twrist.twristTargetStates.DROP);
        LOGGER.log("ocook");
      }
    }
  }

  public void grabAuto() {
    if (queuer.queue(
        true,
        /*(Claw.clawStates.GRAB.getState()
        && Arm.ArmTargetStates.HOVER.getState()
        && GRAB.getState()
        && !Intake.IntakeStates.INTAKING.getState())||*/ lift.getCurrentPosition() > 30
            && !Intake.IntakeStates.INTAKING.getState()
            && Claw.clawStates.GRAB.getState()
            && HOVER.getState())) {
      if ((pixels == 2 || !Intake.IntakeStates.INTAKING.getState())
          && !GRAB.state
          && !Claw.clawStates.GRAB.getState()) {
        arm.flipTo(GRAB);
        wrist.flipTo(Wrist.WristTargetStates.LOCK);
        claw.flipTo(Claw.clawTargetStates.CLOSE);
        gapped = true;
      }
      if (GRAB.state && !Arm.ArmTargetStates.HOVER.getState() && lift.getCurrentPosition() < 20) {
        claw.flipTo(Claw.clawTargetStates.GRAB);
      }
      if (Claw.clawStates.GRAB.getState()) {
        arm.flipTo(HOVER);
      }
    }
  }

  public void grabSupAuto() {
    if (queuer.queue(
        true, Claw.clawStates.GRAB.getState() && Arm.ArmTargetStates.HOVER.getState())) {
      if (!GRAB.state && lift.getCurrentPosition() < 10 && Intake.IntakeStates.STOPPED.getState()) {
        arm.flipTo(GRAB);
        wrist.flipTo(Wrist.WristTargetStates.LOCK);
        claw.flipTo(Claw.clawTargetStates.CLOSE);
      }
      if (GRAB.state && !Arm.ArmTargetStates.HOVER.getState()) {
        claw.flipTo(Claw.clawTargetStates.GRAB);
      }
      if (Claw.clawStates.GRAB.getState()) {
        arm.flipTo(HOVER);
      }
    }
  }

  public void grabAuto(int servo) {
    if (queuer.queue(false, Claw.clawStates.GRAB.getState())) {
      if (!queuer.isExecuted()) {
        arm.flipTo(GRAB);
        wrist.flipTo(Wrist.WristTargetStates.LOCK);
        LOGGER.log("grabbing");
      }
      if (GRAB.getState() && !Claw.clawStates.GRAB.getState()) {
        claw.moveTwo(true);
        claw.moveOne(true);
        Claw.clawTargetStates.GRAB.setStateTrue();
      }
    }
  }

  public void loadAuto() {
    if (queuer.queue(true, true)) {
      if (!queuer.isExecuted()) {
        //        preloader.load();
        LOGGER.log("loading preload");
      }
    }
  }

  public void flipAuto() {
    if (queuer.queue(true, ArmStates.DROP.getState())) {
      if (!queuer.isExecuted()) {
        arm.flipTo(DROP);
        lift.iterateUp();
      }
    }
  }

  public void resetAuto() {
    if (queuer.queue(true, lift.getCurrentPosition() < 50 || HOVER.state)) {
      if (!queuer.isExecuted()) {
        twrist.flipTo(Twrist.twristTargetStates.GRAB);
        wrist.flipTo(Wrist.WristTargetStates.GRAB);
        arm.flipTo(HOVER, -.04, false);
        lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
        claw.moveOne(false);
          claw.moveTwo(false);
          Claw.clawStates.CLOSE.setStateTrue();
        LOGGER.log("buh");
      }
      //      lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
      //      Lift.LiftMovingStates.AT_ZERO.clearTargets();
    }
  }

  public void intakeAuto(int height) {
    if (queuer.queue(
        true,
        ((pixels == 1
                && time - intake.getStartIntakeTime() > 0.4
                && magazine.solidTwoPixels()
                && abs(intake.getVelocity()) < 1400)
            || pixels == 2 && intaked
            || Intake.IntakeStates.REVERSING.getState()))) {
      LOGGER.log(String.valueOf(intake.getVelocity()));
      //      packet.put("pixel", pixels);
      //      packet.put("timeInt",time-intake.getStartIntakeTime());
      //      packet.put("solidTwoPixels", magazine.solidTwoPixels());
      //      packet.put("intakeVel", intake.getVelocity());


      if (currentPose.getX() < 0) {
        int minus=0;
        intaked = true;
//        if(abs(intake.getVelocity()) < 1300 && time - intake.getStartIntakeTime()>0.8 && intake.getIntakePower()!=0){
//            minus=-1;
//        }

        magazine.updateSensors();
        intake.intakeAutoHeight(max(height - pixels, height - 1));
      }
    }
  }

  public void resetLift() {
    if (queuer.queue(true, lift.getCurrentPosition() < 5)) {
      lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
      lift.setPosition(0);
    }
  }

  public void breakAtX(double pos, boolean greater) {
    if (queuer.queue(
        true, !roadrun.isBusy()&& (greater && currentPose.getX() > pos) || (!greater && currentPose.getX() < pos))) {
      if (greater) {
        if (currentPose.getX() > pos) {
          queuer.done();
        }
      } else {
        if (currentPose.getX() < pos) {
          queuer.done();
        }
      }
    }
  }
  public void breakAtY(double pos, boolean greater) {
    if (queuer.queue(
            true, !roadrun.isBusy()&&(greater && currentPose.getY() > pos) || (!greater && currentPose.getY() < pos))) {
      if (greater) {
        if (currentPose.getY() > pos) {
          queuer.done();
        }
      } else {
        if (currentPose.getY() < pos) {
          queuer.done();
        }
      }
    }
  }

  public void dropWrist() {
    if (queuer.queue(false, Wrist.WristStates.DROP.getState())) {
      if (!queuer.isExecuted()) {
        wrist.flipTo(Wrist.WristTargetStates.DROP);
      }
    }
  }

  public void loopPPUI(ArrayList<Waypoint> path, int sign) {
    ppui.followPath(path, sign);
  }

  public void drop() {
    if (queuer.queue(true, Claw.clawStates.CLOSE.getState() && lift.getCurrentPosition() > 100)) {
      //      if (!queuer.isExecuted()) {
      if (currentPose.getX() > 47) {
        claw.moveTwo(false);
        claw.moveOne(false);
        Claw.clawStates.CLOSE.setStateTrue();
        gapped = false;
        intaked = false;
        queuer.done();
      }
      //      }
    }
  }
  public void drop2() {
    if (queuer.queue(true, Claw.clawStates.CLOSE.getState() && lift.getCurrentPosition() > 100)) {
      //      if (!queuer.isExecuted()) {
      if (currentPose.getX()  > 49) {
        claw.moveTwo(false);
        claw.moveOne(false);
        Claw.clawStates.CLOSE.setStateTrue();
        gapped = false;
        intaked = false;
        queuer.done();
      }
      //      }
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

  public boolean checkAlliance() {
//    if (currentPose.getX() > 5 && currentPose.getX() < 30) {
//      LOGGER.log("XPosition: "+currentPose.getX());
      //      if (ultras.checkAlliance()) {
      //        roadrun.setMotorPowers(0, 0, 0, 0);
      //      }
      return ultras.checkAlliance();
//    }
//    return false;
  }

  public boolean checkMovingCloser() {
    return ultras.movingCloser();
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
  public void followTrajSeq(TrajectorySequence p_traj, double pos, boolean greater,boolean isX) {
    if (queuer.queue(false, !roadrun.isBusy())) {
      if (!queuer.isExecuted()) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("going from: " + p_traj.start() + " to: " + p_traj.end());
        roadrun.followTrajectorySequenceAsync(p_traj);
      }
      packet.put("greater", currentPose.getY()>pos);
      if(isX){
        if(greater){
          if(currentPose.getX()>pos){
            queuer.done();
          }
        }
        else{
          if(currentPose.getX()<pos){
            queuer.done();
          }
        }
      } else{
        if(greater){
          if(currentPose.getY()>pos){
            queuer.done();
          }
        }
        else{
          if(currentPose.getY()<pos){
            queuer.done();
          }
        }
      }
    }
  }

  public void followPPPath(Path p_path) {
    boolean equals = false;
    if (path.size() > 1) {
      equals = path.get(1).getPose().equals(p_path.get(1).getPose());
      if (equals) {
        equals = path.get(path.size() - 1).getPose().equals(p_path.get(path.size() - 1).getPose());
      }
    }
    if (queuer.queue(false, equals && (path.isFinished() || path.timedOut()))) {
      if (!queuer.isExecuted()) {
        if (!p_path.equals(path)) path = p_path;
        path.setPathType(PathType.WAYPOINT_ORDERING_CONTROLLED);
        path.init();
        pathFin = false;
      }
      if (queuer.isExecuted() && !equals) {
        queuer.done();
      }
      double[] speeds =
          path.loop(currentPose.getX(), currentPose.getY(), -currentPose.getHeading());
      packet.put("xSpeed", speeds[0]);
      packet.put("ySpeed", speeds[1]);
      packet.put("aSpeed", speeds[2]);
      packet.put("timedOUt", path.timedOut());
      packet.put("isFinished", path.isFinished());
      packet.put("equals", equals);
      var powers =
          MecanumKinematics.robotToWheelVelocities(
                  new Pose2d(speeds[0], speeds[1], speeds[2]), 12, 14, 1.2)
              .toArray();
      roadrun.setMotorPowers(
          (double) powers[0] * 12.5 / voltage,
          (double) powers[1] * 12.5 / voltage,
          (double) powers[2] * 12.5 / voltage,
          (double) powers[3] * 12.5 / voltage);
      if ((double) powers[0] == 0
          && (double) powers[1] == 0
          && (double) powers[2] == 0
          && (double) powers[3] == 0) {
        queuer.done();
        pathFin = true;
      }
      if (path.get(path.size() - 1)
                  .getPose()
                  .getTranslation()
                  .getDistance(new Translation2d(currentPose.getX(), currentPose.getY()))
              < 4
          && currentVelocity.vec().norm() < 0.1) {
        queuer.done();
        pathFin = true;
      }
    } else if (equals) {
      pathFin = true;
    }
  }

  public void loopPPPath(Path p_path, boolean change) {
    if (!path.isEmpty() || change) {
      if (change) {
        path = p_path;
        path.setPathType(PathType.WAYPOINT_ORDERING_CONTROLLED);
        path.init();
        pathFin = false;
      }
      double[] speeds =
          path.loop(currentPose.getX(), currentPose.getY(), -currentPose.getHeading());
      packet.put("xSpeed", speeds[0]);
      packet.put("ySpeed", speeds[1]);
      packet.put("aSpeed", speeds[2]);
      packet.put("timedOUt", path.timedOut());
      packet.put("isFinished", path.isFinished());
      var powers =
          MecanumKinematics.robotToWheelVelocities(
                  new Pose2d(speeds[0], speeds[1], speeds[2]), 12, 8, 1.2)
              .toArray();
      roadrun.setMotorPowers(
          (double) powers[0], (double) powers[1], (double) powers[2], (double) powers[3]);
    }
  }

  public void setBlue(boolean blue) {
    cv.setBlue(blue);
  }

  public void followTrajSeq(TrajectorySequence p_traj, boolean isOptional) {
    if (queuer.isFirstLoop()) {
      queuer.queue(false, false, true);
    } else {
      if (queuer.queue(false, (!roadrun.isBusy()), isOptional)) {
        if (!roadrun.isBusy() && !queuer.isExecuted()) {
          roadrun.followTrajectorySequenceAsync(p_traj);
          LOGGER.log("ULTRA: start: " + p_traj.start() + " end: " + p_traj.end());
        }
      }
    }
  }

  public void followTrajSeqUltra(boolean check, Pose2d startPose) {
    //    if (queuer.isFirstLoop()) {
    //      queuer.queue(true, false, true);
    //    } else {
    if (queuer.queue(true, !roadrun.getCurrentTraj().start().equals(startPose) || (!roadrun.isBusy()&&!check&&roadrun.getCurrentTraj().start().equals(startPose)))) {
      LOGGER.log("STOP IF ULTRA" + roadrun.getCurrentTraj().start() +"<  >" + startPose + "NI" + roadrun.getCurrentTraj().start().equals(startPose));
      if (check) {
        op.telemetry.addData("UTLRADETECT", "ULTRADETECT");
        //          op.telemetry.update();

        brokenFollowing = true;
        roadrun.breakFollowing();
        roadrun.setMotorPowers(0, 0, 0, 0);
        LOGGER.log("ULTRA: STOPPED");
      }
    }
    //    }
  }

  public void hangerTele() {
    float hangUp = op.gamepad2.right_trigger;
    float hangDown = op.gamepad2.left_trigger;
    hanger.setRawPower(hangDown - hangUp);
  }

  /** What is run each loop in teleOp Logs that this function is being called to general surface */
  public void teleOp() {
    boolean isA = gampad.readGamepad(op.gamepad2.a, "gamepad1_a", "resetOuttake");
    boolean rightBumper =
        gampad.readGamepad(op.gamepad1.right_bumper, "gamepad1_right_bumper", "startIntake");
    boolean leftBumper = gampad.readGamepad(op.gamepad1.a, "gamepad1_left_bumper", "reverseIntake");
    boolean isB = gampad.readGamepad(op.gamepad1.y, "gamepad1_y", "shoot");
    boolean isB2 = gampad.readGamepad(op.gamepad2.b, "gamepad2_b", "lockPower");

    boolean isX = gampad.readGamepad(op.gamepad1.x, "gamepad1_x", "toggleFieldCentricSlow");
    boolean up = gampad.readGamepad(op.gamepad2.dpad_up, "gamepad2_dpad_up", "lift Up");
    boolean down = gampad.readGamepad(op.gamepad2.dpad_down, "gamepad2_dpad_down", "lift down");
    boolean right2 =
        gampad.readGamepad(op.gamepad2.dpad_right, "gamepad2_dpad_right", "tilt right");
    boolean left2 = gampad.readGamepad(op.gamepad2.dpad_left, "gamepad2_dpad_left", "tilt left");
    boolean right =
        gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "toggleButterfly");
    boolean left = gampad.readGamepad(op.gamepad1.left_bumper, "gamepad1_dpad_left", "toggleClamp");

    boolean isX2 = gampad.readGamepad(op.gamepad2.x, "gamepad2_x", "toggleFieldCentricSlow");
    boolean isY2 = gampad.readGamepad(op.gamepad2.y, "gamepad2_y", "hanger up");

    float manualUp = op.gamepad1.right_trigger;
    float manualDown = op.gamepad1.left_trigger;
    float hangUp = op.gamepad2.right_trigger;
    float hangDown = op.gamepad2.left_trigger;
    boolean intUp =
        gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "intake iterate up");
    boolean intDown =
        gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "intake iterate down");
    boolean isRightBumper2 =
        gampad.readGamepad(op.gamepad2.right_bumper, "gamepad2_right_bumper", "startIntake");
    boolean isLeftBumper2 =
            gampad.readGamepad(op.gamepad2.left_bumper, "gamepad2_left_bumper", "startIntake");
    ;
    if (isA) {
      twrist.flipTo(Twrist.twristTargetStates.GRAB);
      wrist.flipTo(Wrist.WristTargetStates.GRAB);
      arm.flipTo(HOVER, -.04, false);
      lift.setPosition(Lift.LiftPositionStates.AT_ZERO);
      claw.flipTo(Claw.clawTargetStates.CLOSE);
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
        //        magazine.clampTo(Magazine.MagazineTargetStates.CLOSE);
        intake.intake();
        intake.uppies();

      } else {
        //        magazine.clampTo(Magazine.MagazineTargetStates.OPEN);
        intake.stopIntake();
        intake.downy();
      }
    }
    if (left) {
      intake.stopIntake();
      if (HOVER.getState()
          && !Arm.ArmTargetStates.GRAB.getState()
          && !DROP.getState()
          && lift.getCurrentPosition() < 40) {
        arm.flipTo(GRAB);
        wrist.flipTo(Wrist.WristTargetStates.LOCK);
      }
      if (Claw.clawStates.CLOSE.getState()) {
        claw.flipTo(Claw.clawTargetStates.GRAB);
      } else {
        gapped = false;
        claw.flipTo(Claw.clawTargetStates.CLOSE);
      }
    }
    if (leftBumper) {
      intake.reverseIntake();
    }
    if (left2) {
      if (Twrist.twristStates.LEFT_TILT.getState()) {
        twrist.flipTo(Twrist.twristTargetStates.SLEFT_TILTY);
      } else if (Twrist.twristStates.DROP.getState()) {
        twrist.flipTo(Twrist.twristTargetStates.LEFT_TILT);
      } else if (Twrist.twristStates.VERT.getState()) {
        twrist.flipTo(Twrist.twristTargetStates.SRIGHT_TILTY);
      } else if(Twrist.twristStates.SRIGHT_TILT.getState()){
        twrist.flipTo(Twrist.twristTargetStates.RIGHT_TILT);
      }else if (Twrist.twristStates.RIGHT_TILT.getState()) {
        twrist.flipTo(Twrist.twristTargetStates.OT);
        }else if(Twrist.twristStates.SLEFT_TILT.getState()){
        twrist.flipTo(Twrist.twristTargetStates.VERT);
      } else {
        twrist.flipTo(Twrist.twristTargetStates.DROP);
      }
    }
    if (right2) {
      if (Twrist.twristStates.LEFT_TILT.getState()) {
        twrist.flipTo(Twrist.twristTargetStates.DROP);
      } else if (Twrist.twristStates.DROP.getState()) {
        twrist.flipTo(Twrist.twristTargetStates.OT);
      } else if (Twrist.twristStates.OT.getState()) {
        twrist.flipTo(Twrist.twristTargetStates.RIGHT_TILT);
      } else if (Twrist.twristStates.RIGHT_TILT.getState()) {
        twrist.flipTo(Twrist.twristTargetStates.SRIGHT_TILTY);
      } else if(Twrist.twristStates.SRIGHT_TILT.getState()){
        twrist.flipTo(Twrist.twristTargetStates.VERT);
      } else if(Twrist.twristStates.VERT.getState()){
        twrist.flipTo(Twrist.twristTargetStates.SLEFT_TILTY);
      }
      else {
        twrist.flipTo(Twrist.twristTargetStates.LEFT_TILT);
      }
    }
    if (up) {
      lift.iterateUp();
      intake.stopIntake();
      arm.flipTo(DROP);
      if (Wrist.WristStates.LOCK.getState()) {
        wrist.flipTo(Wrist.WristTargetStates.GRAB);
      }
      wrist.flipTo(Wrist.WristTargetStates.DROP);
      if (Twrist.twristStates.GRAB.getState()) twrist.flipTo(Twrist.twristTargetStates.VERT);
    }
    if (intUp) {
      intake.toggleIntakeHeight();
    }
    if (intDown) intake.toggleIntakeHeightDown();
    if (down) {
      arm.flipTo(DROP);
      wrist.flipTo(Wrist.WristTargetStates.DROP);
//      twrist.flipTo(Twrist.twristTargetStates.DROP);
      lift.iterateDown();
      intake.stopIntake();
    }
    if (abs(op.gamepad2.left_stick_y) > 0.05) {
      lift.manualExtend(-op.gamepad2.left_stick_y);
    }
    //    if (abs(hangUp - hangDown) > 0.05) {
    hanger.setPower(hangUp - hangDown);
    //    }
    if (isB2) {
      hanger.setPermaPower(hangUp - hangDown);
    }
    if(isRightBumper2){
      twrist.flipTo(Twrist.twristTargetStates.RIGHT_TILT);
    }
    if(isLeftBumper2){
      twrist.flipTo(Twrist.twristTargetStates.LEFT_TILT);
    }
    if (right) {
      roadrun.toggleButtered();
      intake.superYuppers();
    }
    if (isX) {
      intake.setHeight(1);
    }
    if (isX2) {
      //      preloader.deposit();
    }

    roadrun.setWeightedDrivePower(
        new Pose2d(
            -op.gamepad1.left_stick_y, -op.gamepad1.left_stick_x, -op.gamepad1.right_stick_x));
    update();
    if(isY2){
      lift.iterateUp();
      lift.iterateUp();
      lift.iterateUp();
    }
    if (pixels == 2
        && !GRAB.state
        && !Arm.ArmTargetStates.HOVER.getState()
        && lift.getCurrentPosition() < 10
        && (Intake.IntakeStates.REVERSING.getState() || Intake.IntakeStates.STOPPED.getState())
        && !gapped) {
      arm.flipTo(GRAB);
      wrist.flipTo(Wrist.WristTargetStates.LOCK);
      claw.flipTo(Claw.clawTargetStates.CLOSE);
      gapped = true;
    }
    if (GRAB.state && !Arm.ArmTargetStates.HOVER.getState() && lift.getCurrentPosition() < 20) {
      claw.flipTo(Claw.clawTargetStates.GRAB);
    }
    if (Claw.clawStates.GRAB.getState() && GRAB.state) {
      arm.flipTo(HOVER);
    }
  }

  public void done() {
    queuer.done();
  }

  /**
   * updates the states of all the following Logs that this function is being called to surface
   * general log All else is logged in each respective function
   */
  public void update() {
    //    LOGGER.setLogLevel(RFLogger.Severity.FINER);

    //    LOGGER.log("updating each component");
    super.update();
    arm.update();
        if (!isTeleop) {
//          if(intake.getIntakePower()==0&&Magazine.pixels==2){
//            Magazine.pixels =0;
//            intaked=false;
//          }
          cv.update();
        }
    if (!isTeleop && abs(intake.getIntakePower()) > 0) {
      magazine.updateSensors();
    }
    intake.update();
    lift.update();
    roadrun.update();
    wrist.update();
    ultras.update();
    if (isTeleop) {
      magazine.update();
      hanger.update();
    }
    twrist.update();
    claw.update();
  }

  public void stop() {
    LOGGER.log("the program has stopped normally");
    cv.stop();
  }
}
