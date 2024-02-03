package org.firstinspires.ftc.teamcode.team.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.CSVP;
import org.firstinspires.ftc.teamcode.team.PoseStorage;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.OuttakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.LiftStateMachine;
import org.firstinspires.ftc.teamcode.team.states.HangStateMachine;

@Autonomous(name = "Red Right CS", group = "RoarAuto")
//right justified
public class RedRightCS extends LinearOpMode {
    CSBaseLIO drive;

    private static double dt;
    private static TimeProfiler updateRuntime;

    //Pose2d startPose = new Pose2d(-36.875, -63.5, Math.toRadians(360));
    //Traj0 is spikeLeft, Traj1 is spikeCenter, Traj2 is spikeRight.
    static final Vector2d TrajL0 = new Vector2d(34.325, -42.595); //Goes to place the pixel on spike mark
    static final Vector2d TrajL0S = new Vector2d(33.325, -42.595); //Goes to place the pixel on spike mark
    static final Vector2d TrajL1 = new Vector2d(45, -60);           //Goes Back to start Pose
    static final Vector2d TrajL2 = new Vector2d(3,-75);           //turns left to face the backstage goes straight to park

    static final Vector2d TrajC0 = new Vector2d(36.01, -37); //Goes to place the pixel on spike mark
    static final Vector2d TrajC1 = new Vector2d(39.125, -60);   //
    static final Vector2d TrajC2 = new Vector2d(48, -61);        //

    static final Vector2d TrajR0 = new Vector2d(42.27, -42.595); //Goes to place the pixel on spike mark
    static final Vector2d TrajR1 = new Vector2d(39.125, -60);          //
    static final Vector2d TrajR2 = new Vector2d(50,-80);          //

    static final Vector2d TrajS0 = new Vector2d(0,0);            //
    static final Vector2d TrajS1 = new Vector2d(0,0);            //
    static final Vector2d TrajS2 = new Vector2d(0,0);            //



    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime detectTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        IDLE,
        WAIT0,
        FORWARD,
        DROP,
        MOVEBACK,
        TOSTAGE,
        LIFTUP,
        OUTTAKE,
        LIFTDOWN,
        TOPARK,
        TOSTACK
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(15.125, -63.5, Math.toRadians(90));

    CSVP CSVP;
    int placement = 3;// default to right
    private static final double HIGH = 26d;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new CSBaseLIO(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.robot.getLiftSubsystem().getStateMachine().updateState(LiftStateMachine.State.IDLE);
        drive.robot.getHangSubsystem().getStateMachine().updateState(HangStateMachine.State.IDLE);
        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.PICKUP);
        drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);

        TrajectorySequence trajL0 = drive.trajectorySequenceBuilder(startPose) //Start pose to spike mark
                .lineTo(TrajL0)
                .turn(Math.toRadians(38))
                .lineTo(TrajL0S)
                .build();


        TrajectorySequence trajL1 = drive.trajectorySequenceBuilder(trajL0.end()) //Turns to normal % comes Back to start Pose
                .lineTo(TrajL1)
                .turn(Math.toRadians(-38))
                .build();

        TrajectorySequence trajL2 = drive.trajectorySequenceBuilder(trajL1.end()) //Turns towards the backboard & parks Right of the backboard
                .turn(Math.toRadians(90))
                .lineTo(TrajL2)
                .build();

        TrajectorySequence trajC0 = drive.trajectorySequenceBuilder(startPose) //Start pose to spike mark
                .lineTo(TrajC0)
                .build();

        TrajectorySequence trajC1 = drive.trajectorySequenceBuilder(trajC0.end()) //Comes Back and Turns towards the backboard
                .lineTo(TrajC1)
                .turn(Math.toRadians(89))
                .build();

        TrajectorySequence trajC2 = drive.trajectorySequenceBuilder(trajC1.end()) //Goes to the Left of the back board turn around 180
                .turn(Math.toRadians(180))
                .lineTo(TrajC2)
                .build();

        TrajectorySequence trajR0 = drive.trajectorySequenceBuilder(startPose) //Start pose to spike mark
                .strafeTo(TrajR0)
                .turn(Math.toRadians(-22))
                // .lineTo(TrajR0)
                .build();

        TrajectorySequence trajR1 = drive.trajectorySequenceBuilder(trajR0.end()) //Comes Back and Turns towards the backboard
                .turn(Math.toRadians(22))
                .lineTo(TrajR1)
                .build();

        TrajectorySequence trajR2 = drive.trajectorySequenceBuilder(trajR1.end()) //Parks to the Right of the backboard
                //  .turn(Math.toRadians(90))
                .lineTo(TrajR2)
                .build();

        TrajectorySequence trajS0 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(TrajS0)
//                .turn(Math.toRadians(70))
                .build();

        TrajectorySequence trajS1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(TrajS1)
//                .turn(Math.toRadians(70))
                .build();

        TrajectorySequence trajS2 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(TrajS2)
//                .turn(Math.toRadians(70))
                .build();



        drive.getExpansionHubs().update(getDt());

        drive.robot.getLiftSubsystem().update(getDt());
        drive.robot.getOuttakeSubsystem().update(getDt());
        drive.robot.getIntakeSubsystem().update(getDt());

        double t1 = waitTimer.milliseconds();

        CSVP = new CSVP();
        CSVP.initTfod(hardwareMap, "Red");

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        int recog = 0;
        int oldrecog = 0;
        int count = 0;
        int total = 0;

        recog = CSVP.rightDetect(); //numerical value

        telemetry.update();
        waitForStart();
        waitTimer.reset();

        if (isStopRequested()) return;
        currentState = State.WAIT0;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case WAIT0:
                    telemetry.addLine("in the wait0 state");

                    if (detectTimer.milliseconds() > 150) {
                        total++;
                        recog = CSVP.rightDetect(); //numerical value
                        detectTimer.reset();

                        if (recog != oldrecog) {
                            oldrecog = recog;
                            count = 1;
                            //telemetry.addLine("located" + recog);
                        } else{
                            count++;
                            //telemetry.addLine("located" + recog + " many times: " + count);
                        }
                    }

                    if (waitTimer.milliseconds() > 4000 ) {
                        if (count > 4) {
                            telemetry.addLine("detections: "+total);
                            telemetry.addLine(" located: " + recog + " many times: " + count);
                        } else {
                            telemetry.addLine("more then than 4 seconds");
                            telemetry.addLine("located" + recog + " many times: " + count);

                        }
                        if (count > 4){
                            currentState = RedRightCS.State.FORWARD;
                            //CSVP.closeVP();
                        }

                        placement = recog;
                    }

                    break;

                case FORWARD:
                    if (placement == 1) {
                        telemetry.addLine("Left");
                        drive.followTrajectorySequenceAsync(trajL0);

                    } else if (placement == 2) {
                        telemetry.addLine("Center");
                        drive.followTrajectorySequenceAsync(trajC0);
                    } else {
                        telemetry.addLine("Right");
                        drive.followTrajectorySequenceAsync(trajR0);
                    }
                    currentState = State.DROP;
                    break;

                case DROP:
                    if(!drive.isBusy()){
                        if(recog!=1) {
                            drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.DROP);
                            currentState = State.MOVEBACK;
                            waitTimer.reset();
                        }//for if
                        else{//when left
                            drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.FASTDROP);
                            currentState = State.MOVEBACK;
                            waitTimer.reset();
                        }//for else
                    }
                    break;

                case MOVEBACK:
                    //give it .5 seconds to drop before moving back
                    if(waitTimer.milliseconds() >= 500) {
                        //stop the intake first
                        drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);
                        if (placement == 1) {
                            drive.followTrajectorySequenceAsync(trajL1);
                        } else if (placement == 2) {
                            drive.followTrajectorySequenceAsync(trajC1);
                        } else {
                            drive.followTrajectorySequenceAsync(trajR1);
                        }
                        currentState = State.IDLE;
                        waitTimer.reset();
                    }
                    break;

                case TOSTAGE:
                    if(!drive.isBusy()) {
                        if (placement == 1) {
                            drive.followTrajectorySequenceAsync(trajL2);
                        } else if (placement == 2) {
                            drive.followTrajectorySequenceAsync(trajC2);
                        } else {
                            drive.followTrajectorySequenceAsync(trajR2);
                        }
                        currentState = State.IDLE;
                    }
                    break;

                case LIFTUP:
                    //lift up at the same time without waiting
                    drive.robot.getLiftSubsystem().extend(HIGH);
                    currentState = State.OUTTAKE;
                    waitTimer.reset();
                    break;



                case OUTTAKE:
                    if(!drive.isBusy() && waitTimer.milliseconds() > 1000){
                        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.RELEASE);
                        currentState = State.LIFTDOWN;
                        waitTimer.reset();
                    }
                    break;

                case LIFTDOWN:
                    //lift up at the same time without waiting
                    drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.PICKUP);
                    drive.robot.getLiftSubsystem().retract();
                    currentState = State.IDLE;
                    waitTimer.reset();
                    break;

                case TOPARK:
                    //add code to park
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(trajL2);
                    }
                    currentState = State.IDLE;

                    break;

                case TOSTACK:
                    //add code to stack
                    break;

                case IDLE:
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    break;
            }

            drive.update();

            //The following code ensure state machine updates i.e. parallel execution with drivetrain
            drive.getExpansionHubs().update(getDt());
            drive.robot.getLiftSubsystem().update(getDt());
            drive.robot.getOuttakeSubsystem().update(getDt());
            drive.robot.getIntakeSubsystem().update(getDt());
            telemetry.update();
        }

        drive.setMotorPowers(0.0,0.0,0.0,0.0);
    }
    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }

    public static void setUpdateRuntime(TimeProfiler updaRuntime) {
        updateRuntime = updaRuntime;
    }

    public static double getDt() {
        return dt;
    }

    public static void setDt(double pdt) {
        dt = pdt;
    }
}