package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.states.FeederConeGripperStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.VirtualFourBarStateMachine;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Feeder;

@TeleOp(name="Feeder Test", group="Test")
public class FeederTest extends PPRobot {
    @Override
    public void init() {
        super.init();
        telemetry.addData("Init", "Hello Storm Trooper");
        updateTelemetry(telemetry);
    }

    @Override
    public void loop() {
        super.loop();
        if(getEnhancedGamepad2().isDpadRightJustPressed()) {
            getStackTracker().takeConeFromStack();
        } else if(getEnhancedGamepad2().isDpadLeftJustPressed()) {
            getStackTracker().addConeToStack();
        } else if(gamepad2.left_trigger > 0.05d) {
            getStackTracker().resetStack();
        }
        if(getEnhancedGamepad2().isDpadUpJustPressed()) {
            getFeeder().extend();
        } else if(getEnhancedGamepad2().isDpadDownJustPressed()) {
            getFeeder().retract();
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
        if(getEnhancedGamepad2().isRightBumperJustPressed() && getStackTracker().getExtensionHeight() == Feeder.getSetpoint()) {
            Feeder.getFeederConeGripperStateMachine().updateState(FeederConeGripperStateMachine.State.OPEN);
        }

        telemetry.addLine("Stones stacked: " + getStackTracker());
        telemetry.addLine("Stacked Height: " + getStackTracker().getExtensionHeight());
        telemetry.addLine("Extension Setpoint: " + Feeder.getSetpoint());
        telemetry.addLine("Extension Desired Setpoint: " + Feeder.getDesiredSetpoint());
        telemetry.addLine("Extension Height: " + getFeeder().getLeftExtension().getPosition());
        telemetry.addLine("Extension State: " + Feeder.getFeederExtensionStateMachine().getState().getName());
        telemetry.addLine("Left Extension Power: " + getFeeder().getLeftExtension().getLastPower());
        telemetry.addLine("Right Extension Power: " + getFeeder().getRightExtension().getLastPower());
        telemetry.addLine("Arm State: " + Feeder.getVirtualFourBarStateMachine().getState().getName());
        telemetry.addLine("Time seen cone: " + getFeeder().getTimeProfilerConeDetection().getDeltaTime(TimeUnits.SECONDS, false));
        telemetry.addLine("Cone distance: " + getFeeder().getConeDetector().getDistance(DistanceUnit.INCH));
        telemetry.addLine("Time threshold: " + getFeeder().getConeInRobotTimeThreshold().getTimeValue(TimeUnits.SECONDS));
        telemetry.addLine("Distance threshold: " + getFeeder().getConeInRobotDistanceThreshold());
        telemetry.addLine("Claw state:" + Feeder.getFeederConeGripperStateMachine().getState().getName());
        telemetry.addLine("hasConeInRobot: " + getFeeder().hasConeInRobot());
        //telemetry.addLine("Feeder Extension Constants: " + Feeder.getExtendControlConstants());
        //telemetry.addLine("Extension close to setpoint: " + getFeeder().closeToSetpoint(1 / 4d));
        //telemetry.addLine("Extension Profile: " + (Feeder.getExtensionProfile() != null));
        if(Feeder.getExtensionProfile() != null) {
            telemetry.addLine("" + Feeder.getExtensionProfile().getPosition());
        }

//        telemetry.addLine("Lift State: " + getFeeder().getFeederExtensionStateMachine().getState());
//        telemetry.addLine("Lift State: " + getStackTracker().getExtensionHeight());
//        telemetry.addLine("Encoder Ticks Left: " + getMotors()[4].getCurrentEncoderTicks());
//        telemetry.addLine("Encoder Ticks Right: " + getMotors()[5].getCurrentEncoderTicks());
//        telemetry.addLine("Last Power L: " + getFeeder().getLeftExtension().getLastPower());
//        telemetry.addLine("Last Power R: " + getFeeder().getRightExtension().getLastPower());
//        telemetry.addLine("Position L: " + getFeeder().getLeftExtension().getPosition());
//        telemetry.addLine("Position R: " + getFeeder().getRightExtension().getPosition());
        updateTelemetry(telemetry);
    }
}
