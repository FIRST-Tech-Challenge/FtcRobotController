package org.firstinspires.ftc.teamcode.team.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.CSVP;
import org.firstinspires.ftc.teamcode.team.PoseStorage;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.OuttakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DroneStateMachine;
import org.firstinspires.ftc.teamcode.team.states.LiftStateMachine;

@Autonomous(name = "Blue Right Pixel", group = "Pixel")
public class BlueRightFTC extends LinearOpMode {
    CSBaseLIO drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final Vector2d Traj0 = new Vector2d(-38,-26);
    static final Vector2d Traj1 = new Vector2d(-38, -29);
    static final Vector2d Traj2 = new Vector2d(-38,-43);
    static final Vector2d Traj3 = new Vector2d(-12, -26);
    static final Vector2d Traj4 = new Vector2d(-12, 72);
//    static final Vector2d Location2 = new Vector2d(-34, 13);
//    static final Vector2d Location3 = new Vector2d(-12,13);

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        WAIT0,
        CLAWCLOSE,
        INITSTRAFE,
        LIFTUP,
        FORWARD,
        PRELOAD,
        MOVEARM,
        CLAWOPEN,
        MOVEBACK,
        LIFTDOWN,
        TODROP,
        TOSTACK,
        IDLE,
        PARK,
        GRAB
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-60, -36, Math.toRadians(90));


    //these are based on LiftTest
//    private static final double HIGH = 24d;
    private static final double MID = 23.5d;
//    private static final double LOW = 14d;


    CSVP CSVP;
    boolean hasCVInit = false;
    int placement = 1;
    float confidence = 0;

    boolean tf = false;

    int counter = 0;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new CSBaseLIO(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.robot.getLiftSubsystem().getStateMachine().updateState(LiftStateMachine.State.IDLE);
//        drive.robot.getDroneSubsystem().getStateMachine().updateState(DroneStateMachine.State.OPEN);
        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.INIT);
        //drive.robot.getCappingArmSubsystem().getStateMachine().updateState(ArmStateMachine.State.REST);

        TrajectorySequence traj0 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(Traj0)
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(traj0.end())
                .lineTo(Traj1)
                .turn(Math.toRadians(-101))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineTo(Traj2)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineTo(Traj3)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineTo(Traj2)
                .build();

//        TrajectorySequence location1 = drive.trajectorySequenceBuilder(traj4.end())
//                .lineTo(Traj4)
//                .build();
//
//        TrajectorySequence location2 = drive.trajectorySequenceBuilder(traj4.end())
//                .lineTo(Location2)
//                .build();
//
//        TrajectorySequence location3 = drive.trajectorySequenceBuilder(traj4.end())
//                .lineTo(Location3)
//                .build();

        drive.getExpansionHubs().update(getDt());

        drive.robot.getLiftSubsystem().update(getDt());
        drive.robot.getOuttakeSubsystem().update(getDt());
        drive.robot.getDroneSubsystem().update(getDt());

        double t1 = waitTimer.milliseconds();

        CSVP = new CSVP();
        CSVP.initTfod(hardwareMap);

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        int detectCounter = 0;
        double confidence = 0;
        String label = "NONE";
        int oldRecog = 0;
        int recog;

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;



        currentState = State.WAIT0;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case WAIT0:
                    telemetry.addLine("in the wait0 state");
                    recog = CSVP.detect();
                    detectCounter++;
                    if (recog != 0){
                        if(oldRecog != 0) {
                            if (CSVP.detect() == recog){
                                //confidence = recog.getConfidence();
                                //label = recog.getLabel();
                                oldRecog = recog;
                            }
                        }
                        else{
                            oldRecog = recog;
                        }
                    }
                    else {
                        telemetry.addLine("NULL");
                    }
                    if (waitTimer.milliseconds() > 3000 && confidence > 0.01){
                        currentState = State.CLAWCLOSE;
                        placement = recog;
                    }
                    break;
                case CLAWCLOSE:
                    telemetry.addData("Location: ", placement); // added to return if left center or right is detected
                    //telemetry.addData("Label: ", label);
                    //telemetry.addData("Confidence: ", confidence);
                    telemetry.addData("#: ", detectCounter);
                    if(waitTimer.milliseconds() >= 1000){
                        drive.robot.getDroneSubsystem().getStateMachine().updateState(DroneStateMachine.State.CLOSE);
                        currentState = State.INITSTRAFE;
                        waitTimer.reset();
                    }
                    break;

                case INITSTRAFE:
                    if(waitTimer.milliseconds() >= 1000){
                        drive.followTrajectorySequenceAsync(traj0);
                        currentState = State.LIFTUP;
                        waitTimer.reset();
                    }
                    break;

                case LIFTUP:
                    if(!drive.isBusy() && waitTimer.milliseconds() >= 750){
                        drive.robot.getLiftSubsystem().extend(MID);
                        if(tf){
                            currentState = State.TODROP;
                        }
                        else {
                            currentState = State.FORWARD;
                            tf = true;
                        }
                        waitTimer.reset();
                    }
                    break;

                case FORWARD:
                    if (waitTimer.milliseconds() >= 2000) {
                        drive.followTrajectorySequenceAsync(traj1);
                        currentState = State.PRELOAD;
                        waitTimer.reset();
                    }
                    break;

                case PRELOAD:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(traj2);
                        currentState = State.MOVEARM;
                        waitTimer.reset();
                    }
                    break;

                case MOVEARM:
                    if(!drive.isBusy()){
                        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.RELEASE);
                        currentState = State.CLAWOPEN;
                        waitTimer.reset();
                    }
                    break;

//                case CLAWOPEN:
//                    if(waitTimer.milliseconds() >= 1000){
//                        drive.robot.getDroneSubsystem().getStateMachine().updateState(DroneStateMachine.State.OPEN);
//                        currentState = State.MOVEBACK;
//                        waitTimer.reset();
//                    }
//                    break;

                case MOVEBACK:
                    if(waitTimer.milliseconds() >= 1000){
                        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.INIT);
                        currentState = State.LIFTDOWN;
                        waitTimer.reset();
                    }
                    break;

//                case LIFTDOWN:
//                    if(waitTimer.milliseconds() >= 1000){
//                      if(counter < 1) {
//                            drive.robot.getLiftSubsystem().extend(6d);
//                           currentState = State.TOSTACK;
//                       }
//                       else{
//                            drive.robot.getLiftSubsystem().retract();
//                            currentState = State.PARK;
//                        }
//                        waitTimer.reset();
//                    }
//                    break;
//
//                case TOSTACK:
//                    if(waitTimer.milliseconds() >= 2000){
//                        drive.followTrajectorySequenceAsync(traj3);
//                        currentState = State.GRAB;
//                        waitTimer.reset();
//                    }
//                    break;
//
//                case GRAB:
//                    if(!drive.isBusy()){
//                        drive.robot.getDroneSubsystem().getStateMachine().updateState(DroneStateMachine.State.CLOSE);
//                        currentState = State.LIFTUP;
//                        waitTimer.reset();
//                    }
//                    break;

                case TODROP:
                    if(waitTimer.milliseconds() >= 2000){
                        drive.followTrajectorySequenceAsync(traj4);
                        if(counter < 1){
                            currentState = State.MOVEARM;
                            waitTimer.reset();
                            counter++;
                        }
                        else {
                            currentState = State.LIFTDOWN;
                            waitTimer.reset();
                        }
                    }
                    break;

                case PARK:
//                    if(waitTimer.milliseconds() >= 2000){
//                        if(placement == "ONE"){
//                            drive.followTrajectorySequenceAsync(location1);
//                        }
//                        else if(placement == "TWO"){
//                            drive.followTrajectorySequenceAsync(location2);
//                        }
//                        else if(placement == "THREE"){
//                            drive.followTrajectorySequenceAsync(location3);
//                        }
//                        currentState = State.IDLE;
//                        waitTimer.reset();
//                    }
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
            drive.robot.getDroneSubsystem().update(getDt());
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