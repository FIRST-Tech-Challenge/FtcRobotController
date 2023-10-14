package org.firstinspires.ftc.teamcode.team10515;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.auto.PPBaseFeeder;
import org.firstinspires.ftc.teamcode.team10515.states.FeederConeGripperStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.VirtualFourBarStateMachine;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Feeder;

@TeleOp(name = "PP TeleOp Feeder", group = "Main")
public class PPTeleopFeeder extends PPTeleopRobotFeeder {
    private double speedMultiplier = 0.7;
    //these are based on LiftTest
    private static final double HIGH = 24d;
    private static final double MID = 23.5d;
    private static final double LOW = 14d;
    private boolean liftdown = true;
    private boolean armMid = true;

    //private boolean coneloaded = false;
    private Pose2d poseEstimate;

    //RevBlinkinLedDriver blinkinLedDriver;
    //RevBlinkinLedDriver.BlinkinPattern pattern;
    //private ElapsedTime timeSinceIntakeButton = new ElapsedTime();

    @Override
    public void init(){
        drive = new PPBaseFeeder(hardwareMap, true);
        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        super.init();
//        for (int i = 0; i < ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles().length; i++){
//            DbgLog.msg(String.valueOf(ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles()[i]));
//        }
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
        poseEstimate = drive.getPoseEstimate();

        //Gamepad 1
        //drive
        if (getEnhancedGamepad1().getLeft_trigger() > 0) {
            speedMultiplier = 0.7;
        }
        if (getEnhancedGamepad1().getRight_trigger() > 0) {
            speedMultiplier = 0.5;
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier
                )
        );

        if(getEnhancedGamepad2().isDpadRightJustPressed()) {
//            drive.robot.getStackTracker().takeConeFromStack();
            drive.robot.getStackTracker().incrementPoleTargetType();
        } else if(getEnhancedGamepad2().isDpadLeftJustPressed()) {
//            drive.robot.getStackTracker().addConeToStack();
            drive.robot.getStackTracker().decrementPoleTargetType();
        } else if(gamepad2.left_trigger > 0.05d) {
//            drive.robot.getStackTracker().resetStack();
            drive.robot.getStackTracker().resetPoleTargetType();
        }
        if(getEnhancedGamepad2().isDpadUpJustPressed()) {
//            drive.robot.getFeeder().extend();
            drive.robot.getFeeder().extendPoles();
        } else if(getEnhancedGamepad2().isDpadDownJustPressed()) {
            drive.robot.getFeeder().retract();
        }
        if(getEnhancedGamepad2().isaJustPressed()) {
            Feeder.getVirtualFourBarStateMachine().updateState(VirtualFourBarStateMachine.State.INIT);
        }
        if(getEnhancedGamepad2().isbJustPressed()) {
            Feeder.getVirtualFourBarStateMachine().updateState(VirtualFourBarStateMachine.State.RIGHT);
        }
        if(getEnhancedGamepad2().isxJustPressed()) {
            Feeder.getVirtualFourBarStateMachine().updateState(VirtualFourBarStateMachine.State.LEFT);
        }
//        if(getEnhancedGamepad2().isRightBumperJustPressed() && drive.robot.getStackTracker().getExtensionHeight() == Feeder.getSetpoint()) {
//            Feeder.getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.OPEN);
//        }
        //TODO: Replace logic below with the logic above in 91-93
        //Claw
        if (getEnhancedGamepad1().isLeftBumperJustPressed()) {
            Feeder.getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.OPEN);
        }
        if (getEnhancedGamepad1().isRightBumperJustPressed()) {
            Feeder.getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.CLOSE);
        }

//        telemetry.addLine("Stones stacked: " + drive.robot.getStackTracker());
        telemetry.addLine("Stacked Height: " + drive.robot.getStackTracker().getExtensionHeight());
        telemetry.addLine(drive.robot.getStackTracker().toString_PoleType());
        telemetry.addLine("Pole Height: " + drive.robot.getStackTracker().getPoleExtensionHeight());
        telemetry.addLine("Extension Setpoint: " + Feeder.getSetpoint());
        telemetry.addLine("Extension Desired Setpoint: " + Feeder.getDesiredSetpoint());
        telemetry.addLine("Extension Height: " + drive.robot.getFeeder().getLeftExtension().getPosition());
        telemetry.addLine("Extension State: " + Feeder.getFeederExtensionStateMachine().getState().getName());
        telemetry.addLine("Left Extension Power: " + drive.robot.getFeeder().getLeftExtension().getLastPower());
        telemetry.addLine("Right Extension Power: " + drive.robot.getFeeder().getRightExtension().getLastPower());
        telemetry.addLine("Arm State: " + Feeder.getVirtualFourBarStateMachine().getState().getName());
        telemetry.addLine("Time seen cone: " + drive.robot.getFeeder().getTimeProfilerConeDetection().getDeltaTime(TimeUnits.SECONDS, false));
        telemetry.addLine("Cone distance: " + drive.robot.getFeeder().getConeDetector().getDistance(DistanceUnit.INCH));
        telemetry.addLine("Time threshold: " + drive.robot.getFeeder().getConeInRobotTimeThreshold().getTimeValue(TimeUnits.SECONDS));
        telemetry.addLine("Distance threshold: " + drive.robot.getFeeder().getConeInRobotDistanceThreshold());
        telemetry.addLine("Claw state:" + Feeder.getFeederConeGripperStateMachine().getState().getName());
        telemetry.addLine("hasConeInRobot: " + drive.robot.getFeeder().hasConeInRobot());
        updateTelemetry(telemetry);
    }
}
