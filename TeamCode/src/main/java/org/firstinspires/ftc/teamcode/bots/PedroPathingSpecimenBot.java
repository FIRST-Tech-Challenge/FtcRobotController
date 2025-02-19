package org.firstinspires.ftc.teamcode.bots;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.ThreeWheelIMULocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.bots.DifferentialWristBot;
public class PedroPathingSpecimenBot extends FSMBot{

     private Follower follower;
     private Timer pathTimer, actionTimer, opmodeTimer;

     /** This is the variable where we store the state of our auto.
      * It is used by the pathUpdate method. */
     public int pathState;

     private final Pose startPose = new Pose(7.7, 53.89, 0);// starting position of robot
     private final Pose scoreSpecimenPose = new Pose(40, 66, Math.toRadians(180));// position where specimen is scored on submersible, robot is aligned to submerisble with back facing it

     //    private final Pose sample1 = new Pose(35, 23,0); //these three poses are just behind the samples
     private final Pose samplePivotPose = new Pose(35, 12.7,0); //pivot from one point to grab all 3 samples
//    private final Pose sample3 = new Pose(35, 6,0);

     private final Pose dropSamplePose = new Pose(28, 17, Math.toRadians(180));

     private final Pose loadSpecimenPose = new Pose(7.9, 23.6, 0);

     private final Pose scoreSamplePose = new Pose(17.7, 122.3, Math.toRadians(-45));

     private final Pose sampleCurveControlPoint = new Pose(21.8, 36.7, Math.toRadians(-36));

     /** Park Pose for our robot, after we do all of the scoring. */
     private final Pose parkPose = new Pose(60, 46, Math.toRadians(90));

     /** coordinate to control bezier curve for parking, to go around the submersible must use bezier curve, this is mid point.*/
     private final Pose parkControl = new Pose (37, 25, 0);



     private Path scorePreload, park;

     private PathChain pickup, dropoff, loadSpecimen, scoreSpecimen;

//     private HardwareMap testMap = new HardwareMap();

     public void buildPaths(){
          follower = new Follower(hardwareMap);
          scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSpecimenPose)));
          scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scoreSpecimenPose.getHeading());

          pickup = follower.pathBuilder()
                  .addPath(new BezierLine(new Point(dropSamplePose), new Point (samplePivotPose)))
                  .setLinearHeadingInterpolation(scoreSpecimenPose.getHeading(), samplePivotPose.getHeading())
                  .build();

          dropoff = follower.pathBuilder()
                  .addPath(new BezierLine(new Point(samplePivotPose), new Point(dropSamplePose)))
                  .setLinearHeadingInterpolation(samplePivotPose.getHeading(), dropSamplePose.getHeading())
                  .build();

          loadSpecimen = follower.pathBuilder()
                  .addPath(new BezierLine(new Point(dropSamplePose), new Point(loadSpecimenPose)))
                  .setLinearHeadingInterpolation(dropSamplePose.getHeading(), loadSpecimenPose.getHeading())
                  .build();

          scoreSpecimen = follower.pathBuilder()
                  .addPath(new BezierLine(new Point(loadSpecimenPose), new Point(scoreSpecimenPose)))
                  .setLinearHeadingInterpolation(loadSpecimenPose.getHeading(), scoreSpecimenPose.getHeading())
                  .build();

          park = new Path(new BezierCurve(new Point(scoreSpecimenPose), /* Control Point */ new Point(parkControl), new Point(parkPose)));
          park.setLinearHeadingInterpolation(scoreSpecimenPose.getHeading(), parkPose.getHeading());

     }

     public PedroPathingSpecimenBot(LinearOpMode opMode) {
          super(opMode);
     }
@Override
     public void init(HardwareMap ahwMap) {
          super.init(ahwMap);
     Constants.setConstants(FollowerConstants.class, ThreeWheelIMULocalizer.class);
     pathTimer = new Timer();
     opmodeTimer = new Timer();
     opmodeTimer.resetTimer();
          buildPaths();
          follower = new Follower(ahwMap);
          follower.setStartingPose(startPose);
     }

     protected void onTick() {
          super.onTick();
          // These loop the movements of the robot
          follower.update();
          autonomousPathUpdate();

          // Feedback to Driver Hub
          telemetry.addData("path state", EVENT_NAMES[pathState], pathState);
          telemetry.addData("x", follower.getPose().getX());
          telemetry.addData("y", follower.getPose().getY());
          telemetry.addData("heading", follower.getPose().getHeading());
          telemetry.update();
     }

     private int AT_PRELOAD_POSITION= 1;

     public int getPathState(){
          return pathState;
     }


     public void autonomousPathUpdate() {
          switch (pathState) {
               case 0:
                    follower.followPath(scorePreload);

                    setPathState(AT_PRELOAD_POSITION);
                    //Goes to submersible, in position to score preload
                    break;
               case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if(!follower.isBusy()/**||pathTimer.getElapsedTimeSeconds() > 2*/) {
                         /* Score Preload */


                         currentState = gameState.SPECIMEN_SCORING_HIGH;
                         //INSERT 3DOF CODE HERE TO SCORE SPECIMEN
                         /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                         follower.followPath(grabPickup1,true);
                         triggerEvent(EVENT_PRELOAD_SCORED, 2);
                         setPathState(EVENT_PRELOAD_SCORED);
                    }
               break;

               case 2:

                    if(!follower.isBusy()){
                         follower.followPath(pickup, true);
                         //pickup first sample
                         triggerEvent(EVENT_SAMPLE_1_PICKEDUP, 3);
                         setPathState(EVENT_SAMPLE_1_PICKEDUP);

                    }

                    break;

               case 3:

                    if(!follower.isBusy()){
                         follower.followPath(dropoff, true);
                         //release sample with claw
                         triggerEvent(EVENT_SAMPLE_1_DROPPEDOFF, 4);
                         setPathState(EVENT_SAMPLE_1_DROPPEDOFF);
                    }

                    break;

               case 4:

                    if(!follower.isBusy()){
                         follower.followPath(pickup, true);
                         //pickup first sample
                         triggerEvent(EVENT_SAMPLE_2_PICKEDUP, 5);
                         setPathState(EVENT_SAMPLE_2_PICKEDUP);
                    }


                    break;

               case 5:

                    if(!follower.isBusy()){
                         follower.followPath(dropoff, true);
                         //release sample with claw
                         triggerEvent(EVENT_SAMPLE_2_DROPPEDOFF, 6);
                         setPathState(EVENT_SAMPLE_2_DROPPEDOFF);
                    }

                    break;

               case 6:

                    if(!follower.isBusy()){
                         follower.followPath(pickup, true);
                         //pickup first sample
                         triggerEvent(EVENT_SAMPLE_3_PICKEDUP, 7);
                         setPathState(EVENT_SAMPLE_3_PICKEDUP);
                    }


                    break;

               case 7:

                    if(!follower.isBusy()){
                         follower.followPath(dropoff, true);
                         //release sample with claw
                         triggerEvent(EVENT_SAMPLE_3_DROPPEDOFF, 8);
                         setPathState(EVENT_SAMPLE_3_DROPPEDOFF);
                    }

                    break;
               case 8:

                    if(!follower.isBusy()){
                         follower.followPath(loadSpecimen, true);
                         //RAISE PIVOT FOR SPECIMEN CLAW LOAD, THEN DROP
                     triggerEvent(EVENT_SPECIMEN_1_LOADED);
                     setPathState(9);

                    }
                    break;
               case 9:

                    if(!follower.isBusy()){
                         follower.followPath(scoreSpecimen, true);
                         //RAISE PIVOT TO SCORE HIGH SPECIMEN, THEN DROP
                         triggerEvent(EVENT_SPECIMEN_1_SCORED);
                         setPathState(EVENT_SPECIMEN_1_SCORED);
                    }
                    break;
               case 10:

                    if(!follower.isBusy()){
                         follower.followPath(loadSpecimen, true);
                         //RAISE PIVOT FOR SPECIMEN CLAW LOAD, THEN DROP
                         triggerEvent(EVENT_SPECIMEN_2_LOADED);
                         setPathState(EVENT_SPECIMEN_2_LOADED);

                    }
                    break;
               case 11:

                    if(!follower.isBusy()){
                         follower.followPath(scoreSpecimen, true);
                         //RAISE PIVOT TO SCORE HIGH SPECIMEN, THEN DROP
                         triggerEvent(EVENT_SPECIMEN_2_SCORED);
                         setPathState(EVENT_SPECIMEN_2_SCORED);
                    }
                    break;
               case 12:

                    if(!follower.isBusy()){
                         follower.followPath(loadSpecimen, true);

                         //RAISE PIVOT FOR SPECIMEN CLAW LOAD, THEN DROP
                         triggerEvent(EVENT_SPECIMEN_3_SCORED);
                         setPathState(EVENT_SPECIMEN_3_SCORED);

                    }
                    break;
               case 13:

                    if(!follower.isBusy()){
                         follower.followPath(scoreSpecimen, true);
                         //RAISE PIVOT TO SCORE HIGH SPECIMEN, THEN DROP
                         triggerEvent(EVENT_SPECIMEN_3_SCORED);
                         setPathState(EVENT_SPECIMEN_3_SCORED);
                    }
                    break;
               case 14:

                    if(!follower.isBusy()){
                         follower.followPath(park, true);
                         triggerEvent(EVENT_PARKED);
                         setPathState(EVENT_PARKED);
                    }

                    break;

               case 15:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if(!follower.isBusy()) {
                         /* Level 1 Ascent */

                         /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                         setPathState(-1);
                    }
                    break;
          }
     }
     /** These change the states of the paths and actions
      * It will also reset the timers of the individual switches **/


     private void setPathState(int pState) {
          pathState = pState;
          pathTimer.resetTimer();
     }


     protected void onEvent(int event, int data) {
          super.onEvent(event, data);
//          if (event == EVENT_SAMPLE_PICKED_UP) {
//              // Do something when the sample is picked up
//              telemetry.addData("Sample picked up - Pedro Pathing", data);
//          }

          telemetry.update();
     }
}
