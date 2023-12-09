package org.firstinspires.ftc.teamcode.team.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.DbgLog;
import org.firstinspires.ftc.teamcode.team.CSVP;
import org.firstinspires.ftc.teamcode.team.PoseStorage;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.FeederConeGripperStateMachine;
import org.firstinspires.ftc.teamcode.team.states.FeederExtensionStateMachine;
import org.firstinspires.ftc.teamcode.team.states.VirtualFourBarStateMachine;
import org.firstinspires.ftc.teamcode.team.subsystems.Feeder;

@Autonomous(name = "Red Right Pixel", group = "Pixel")
public class RedRightFTC extends LinearOpMode {
    PPBaseFeeder drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final Vector2d Traj0 = new Vector2d(36.0,-63.5);
    static final Vector2d Traj1 = new Vector2d(36.0, -10.0);
    static final Vector2d Traj2 = new Vector2d(29.25,-10.0);
    static final Vector2d Traj3 = new Vector2d(60.5, -17);
    static final Vector2d Traj4 = new Vector2d(52, -13.5);
    static final Vector2d Location1 = new Vector2d(55, -13);
    static final Vector2d Location2 = new Vector2d(36, -13);
    static final Vector2d Location3 = new Vector2d(14,-13);

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        WAIT0,
        CLAWCLOSE,
        INITSTRAFE,
        STACK_OR_PARK,
        FORWARD,
        PRELOAD,
        MOVEARM,
        CLAWOPEN,
        MOVEARMBACK,
        LIFTDOWN,
        TODROP,
        TOSTACK,
        IDLE,
        PARK,
        GRAB
    }

    State currentState = State.IDLE;

//    Pose2d startPose = new Pose2d(37, -65.5, Math.toRadians(90));
    Pose2d startPose = new Pose2d(39.25, -63.5, Math.toRadians(90));


    //these are based on LiftTest
    private static final double HIGH = 24d;
    private static final double MID = 23.5d;
    private static final double LOW = 14d;

    CSVP CSVP;
    boolean hasCVInit = false;
    String placement = "ONE";
    float confidence = 0;

    boolean tf = false;

    int counter = 0;

    private final double numOfCycles = 2;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new PPBaseFeeder(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.robot.getFeeder().getFeederExtensionStateMachine().updateState(FeederExtensionStateMachine.State.IDLE);
        drive.robot.getFeeder().getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.OPEN);
        drive.robot.getFeeder().getVirtualFourBarStateMachine().updateState(VirtualFourBarStateMachine.State.INIT);

        //Added 12/30
        drive.robot.getFeeder().getLeftExtension().setPower(0d);
        drive.robot.getFeeder().getRightExtension().setPower(0d);
        drive.robot.getFeeder().getLeftExtension().resetEncoder();
        drive.robot.getFeeder().getRightExtension().resetEncoder();
        drive.robot.getFeeder().setSetpoint(0d, false);
        drive.robot.getFeeder().setDesiredSetpoint(0d);

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
                .lineTo(Traj4)
                .build();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end()) //Added because traj3 uses the ending of traj2, but we call traj3 twice: after traj2 AND after traj4
                .lineTo(Traj3) //This is the same as traj3 except it starts at the ending of traj4 because we need to call it after traj4 as well for cycles
                .build();

        TrajectorySequence location1 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(Location1)
                .build();

        TrajectorySequence location2 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(Location2)
                .build();

        TrajectorySequence location3 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(Location3)
                .build();

        drive.getExpansionHubs().update(getDt());

        drive.robot.getFeeder().update(getDt());

        double t1 = waitTimer.milliseconds();

        CSVP = new CSVP();
        //ppcv.initVuforia(hardwareMap);
        CSVP.initTfod(hardwareMap);

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        int detectCounter = 0;
        double confidence = 0;
        String label = "NONE";
        Recognition oldRecog = null;
        Recognition recog;

        DbgLog.msg("HI: " + traj1.end().toString());

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
                    if (recog != null){
                        if(oldRecog != null) {
                            if (CSVP.detect() == recog){
                                confidence = recog.getConfidence();
                                label = recog.getLabel();
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
                        placement = label;
                    }
                    break;

                case CLAWCLOSE:
                    telemetry.addData("Label: ", label);
                    telemetry.addData("Confidence: ", confidence);
                    telemetry.addData("#: ", detectCounter);
                    if(waitTimer.milliseconds() >= 250){
                    //    drive.robot.getFeeder().getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.CLOSE);
                        currentState = State.INITSTRAFE;
                        waitTimer.reset();
                    }
                    break;

                case INITSTRAFE:
                    if(waitTimer.milliseconds() >= 250){
                        drive.followTrajectorySequenceAsync(traj0);
//                        drive.robot.getStackTracker().setPoleTargetType(2);
//                        drive.robot.getFeeder().extendPoles();
                        currentState = State.STACK_OR_PARK;
                        waitTimer.reset();
                    }
                    break;

                case STACK_OR_PARK:
                    if(tf){
                        if (waitTimer.milliseconds() >= 750) {
                            drive.robot.getStackTracker().setPoleTargetType(1);
                            drive.robot.getFeeder().extendPoles();
                            currentState = State.TODROP;
                            waitTimer.reset();
                        }
                    }
                    else {
                        currentState = State.FORWARD;
                        tf = true;
                    }
                    break;

                case FORWARD:
                    if (!drive.isBusy() && waitTimer.milliseconds() >= 1000) {
                        drive.robot.getStackTracker().setPoleTargetType(2);
                        drive.robot.getFeeder().extendPoles();
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
                        drive.robot.getFeeder().getVirtualFourBarStateMachine().updateState(VirtualFourBarStateMachine.State.LEFT);
                        currentState = State.CLAWOPEN;
                        waitTimer.reset();
                    }
                    break;

                case CLAWOPEN:
                    if(waitTimer.milliseconds() >= 750){
                        drive.robot.getFeeder().getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.OPEN);
                        currentState = State.MOVEARMBACK;
                        waitTimer.reset();
                    }
                    break;

                case MOVEARMBACK:
                    if(waitTimer.milliseconds() >= 750){
                        drive.robot.getFeeder().getVirtualFourBarStateMachine().updateState(VirtualFourBarStateMachine.State.INIT);
                        currentState = State.LIFTDOWN;
                        waitTimer.reset();
                    }
                    break;

                case LIFTDOWN:
                    if(waitTimer.milliseconds() >= 1000){
                        if(counter < numOfCycles) {
                            drive.robot.getFeeder().extend();
                            currentState = State.TOSTACK;
                        }
                        else{
                            drive.robot.getFeeder().retract();
                            currentState = State.PARK;
                        }
                        waitTimer.reset();
                    }
                    break;

                case TOSTACK:
                    if(waitTimer.milliseconds() >= 2000){
                        if (counter == 0) {
                            drive.followTrajectorySequenceAsync(traj3); //If we haven't cycled yet
                        }
                        else {
                            drive.followTrajectorySequenceAsync(traj5); //If we have cycled
                        }
                        //Remember, traj3 and traj5 are the same except for their starting positions - one starts at traj2's ending (preload drop) and the other at traj4's ending (low junction cycle drop)
                        currentState = State.GRAB;
                        waitTimer.reset();
                    }
                    break;

                case GRAB:
                    if(!drive.isBusy()){
                        drive.robot.getFeeder().getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.CLOSE);
                        drive.robot.getStackTracker().takeConeFromStack();
                        currentState = State.STACK_OR_PARK;
                        waitTimer.reset();
                    }
                    break;

                case TODROP:
                    if(waitTimer.milliseconds() >= 100){
                        drive.followTrajectorySequenceAsync(traj4);
                        if(counter < numOfCycles){
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
                    if(waitTimer.milliseconds() >= 2000){
                        if(placement == "ONE"){
                            drive.followTrajectorySequenceAsync(location1);
                        }
                        else if(placement == "TWO"){
                            drive.followTrajectorySequenceAsync(location2);
                        }
                        else if(placement == "THREE"){
                            drive.followTrajectorySequenceAsync(location3);
                        }
                        currentState = State.IDLE;
                        waitTimer.reset();
                    }
                    break;

                case IDLE:
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    break;
            }


            drive.update();
            telemetry.addLine(drive.getPoseEstimate().toString());
            telemetry.addLine("Stacked Height: " + drive.robot.getStackTracker().getExtensionHeight());
            telemetry.addLine("Extension Setpoint: " + Feeder.getSetpoint());
            telemetry.addLine("Extension Desired Setpoint: " + Feeder.getDesiredSetpoint());
            telemetry.addLine("Extension Height: " + drive.robot.getFeeder().getLeftExtension().getPosition());
            telemetry.addLine("Extension State: " + Feeder.getFeederExtensionStateMachine().getState().getName());
            telemetry.addLine("Claw state:" + Feeder.getFeederConeGripperStateMachine().getState().getName());
            telemetry.addLine("Arm State: " + Feeder.getVirtualFourBarStateMachine().getState().getName());
            telemetry.addLine("Left Extension Power: " + drive.robot.getFeeder().getLeftExtension().getLastPower());
            telemetry.addLine("Right Extension Power: " + drive.robot.getFeeder().getRightExtension().getLastPower());
            telemetry.addLine("Time seen cone: " + drive.robot.getFeeder().getTimeProfilerConeDetection().getDeltaTime(TimeUnits.SECONDS, false));
            telemetry.addLine("Cone distance: " + drive.robot.getFeeder().getConeDetector().getDistance(DistanceUnit.INCH));
            telemetry.addLine("Time threshold: " + drive.robot.getFeeder().getConeInRobotTimeThreshold().getTimeValue(TimeUnits.SECONDS));
            telemetry.addLine("Distance threshold: " + drive.robot.getFeeder().getConeInRobotDistanceThreshold());
            telemetry.addLine("hasConeInRobot: " + drive.robot.getFeeder().hasConeInRobot());
            //The following code ensure state machine updates i.e. parallel execution with drivetrain
            drive.getExpansionHubs().update(getDt());
            drive.robot.getFeeder().update(getDt());
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
