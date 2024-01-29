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

@Autonomous(name = "Blue Right CS", group = "RoarAuto")
//right justified
public class BlueRightCS extends LinearOpMode {
    CSBaseLIO drive;

    private static double dt;
    private static TimeProfiler updateRuntime;

    //Traj0 is spikeLeft, Traj1 is spikeCenter, Traj2 is spikeRight.
    static final Vector2d TrajL0 = new Vector2d(27.3, -1.5);
    static final Vector2d TrajL1 = new Vector2d(3, 0);
    static final Vector2d TrajL2 = new Vector2d(3,75);

    static final Vector2d TrajC0 = new Vector2d(27,.8);
    static final Vector2d TrajC1 = new Vector2d(3, 0);
    static final Vector2d TrajC2 = new Vector2d(3,75);//left parking

    static final Vector2d TrajR0 = new Vector2d(20.7,-4.5);
    static final Vector2d TrajR1 = new Vector2d(50, 3);
    static final Vector2d TrajR2 = new Vector2d(50,80); //right parking

    static final Vector2d TrajS0 = new Vector2d(23,-20);
    static final Vector2d TrajS1 = new Vector2d(44,-20);
    static final Vector2d TrajS2 = new Vector2d(65,-20);



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

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(360));

    CSVP CSVP;
    int placement = 3;// default to right
    private static final double MID = 18d;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new CSBaseLIO(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.robot.getLiftSubsystem().getStateMachine().updateState(HangStateMachine.State.IDLE);
        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.PICKUP);
        drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);

        TrajectorySequence trajL0 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(TrajL0)
                .turn(Math.toRadians(70))
                .build();

        TrajectorySequence trajL1 = drive.trajectorySequenceBuilder(trajL0.end())
                .turn(Math.toRadians(-70))
                .lineTo(TrajL1)
                .build();

        TrajectorySequence trajL2 = drive.trajectorySequenceBuilder(trajL1.end())
                .turn(Math.toRadians(90))
                .lineTo(TrajL2)
                .build();

        TrajectorySequence trajC0 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(8))
                .lineTo(TrajC0)
                .build();

        TrajectorySequence trajC1 = drive.trajectorySequenceBuilder(trajC0.end())
                .lineTo(TrajC1)
                .turn(Math.toRadians(-8))
                .build();

        TrajectorySequence trajC2 = drive.trajectorySequenceBuilder(trajC1.end())
                .turn(Math.toRadians(90))
                .lineTo(TrajC2)
                .build();

        TrajectorySequence trajR0 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(TrajR0)
                .turn(Math.toRadians(-47))
                .build();

        TrajectorySequence trajR1 = drive.trajectorySequenceBuilder(trajR0.end())
                .turn(Math.toRadians(47))
                .lineTo(TrajR1)
                .build();

        TrajectorySequence trajR2 = drive.trajectorySequenceBuilder(trajR1.end())
                .turn(Math.toRadians(90))
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
        CSVP.initTfod(hardwareMap, "Blue");

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        int recog = 0;
        int oldrecog = 0;
        int count = 0;
        int total = 0;

        recog = CSVP.leftDetect(); //numerical value

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
                       recog = CSVP.leftDetect(); //numerical value
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
                            currentState = BlueRightCS.State.IDLE;
                        //CSVP.closeVP();
                    }

                        placement = 3;
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
                        drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.DROP);
                        currentState = State.MOVEBACK;
                        waitTimer.reset();
                    }
                    break;

                case MOVEBACK:
                    //give it 2 seconds to drop before moving back
                    if(waitTimer.milliseconds() >= 1000) {
                        //stop the intake first
                        drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);
                        if (placement == 1) {
                            drive.followTrajectorySequenceAsync(trajL1);
                        } else if (placement == 2) {
                            drive.followTrajectorySequenceAsync(trajC1);
                        } else {
                            drive.followTrajectorySequenceAsync(trajR1);
                        }
                        currentState = State.TOSTAGE;
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
                    drive.robot.getLiftSubsystem().extend(MID);
                    currentState = State.OUTTAKE;
                    waitTimer.reset();
                    break;



                case OUTTAKE:
                    if(!drive.isBusy() && waitTimer.milliseconds() > 3000){
                        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.RELEASE);
                        currentState = State.OUTTAKE;
                        waitTimer.reset();
                    }
                    break;

                case LIFTDOWN:
                    //lift up at the same time without waiting
                    drive.robot.getLiftSubsystem().retract();
                    currentState = State.IDLE;
                    waitTimer.reset();
                    break;

                case TOPARK:
                    //add code to park
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