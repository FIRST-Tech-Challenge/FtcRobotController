package org.firstinspires.ftc.teamcode.team.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.CSVP;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.OuttakeStateMachine;

@Autonomous(name = "Stack test", group = "RoarAuto")
//right justified
public class Stacktest extends LinearOpMode {
    CSBaseLIO drive;

    private static double dt;
    private static TimeProfiler updateRuntime;

    //Traj0 is spikeLeft, Traj1 is spikeCenter, Traj2 is spikeRight.
    static final Vector2d Traj0 = new Vector2d(20, -10);
    static final Vector2d Traj1 = new Vector2d(20, 0);
    static final Vector2d Traj2 = new Vector2d(0, 0);



    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime detectTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        IDLE,
        FORWARD,
        DROP,
        STAFE,
        MOVEBACK
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(360));

    CSVP CSVP;
    int placement = 3;
    private static final double MID = 18d;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new CSBaseLIO(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.robot.getLiftSubsystem().getStateMachine().updateState(HangStateMachine.State.IDLE);
        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.PICKUP);
        drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);

        TrajectorySequence traj0 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(Traj0)
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(traj0.end())
                .lineTo(Traj1)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineTo(Traj2)
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

        int recog=0;

        telemetry.update();
        waitForStart();
        waitTimer.reset();

        if (isStopRequested()) return;
        currentState = State.IDLE;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case IDLE:
                    if (waitTimer.milliseconds() > 500) {
                        currentState = State.FORWARD;
                        waitTimer.reset();
                    }
                    break;

                case FORWARD:
                    if (waitTimer.milliseconds() > 4000) {
                        drive.followTrajectorySequenceAsync(traj0);

                        currentState = State.DROP;
                        waitTimer.reset();
                    }
                    break;

                case DROP:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(traj1);
                        drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.INTAKE);
                        currentState = State.MOVEBACK;
                        waitTimer.reset();
                    }
                    break;

                case MOVEBACK:
                    //give it 1 seconds to drop before moving back
                    if(waitTimer.milliseconds() >= 2000) {
                        drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);
                        drive.followTrajectorySequenceAsync(traj2);
                        currentState = State.IDLE;
                        waitTimer.reset();
                    }
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