package org.nknsd.teamcode.programs.tests.thisYear.aprilTag;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.nknsd.teamcode.components.drivers.MixedInputWheelDriver;
import org.nknsd.teamcode.components.handlers.gamepad.GamePadHandler;
import org.nknsd.teamcode.components.handlers.odometry.CameraFlowFuser;
import org.nknsd.teamcode.components.motormixers.AbsolutePowerMixer;
import org.nknsd.teamcode.components.motormixers.MecanumMotorMixer;
import org.nknsd.teamcode.components.motormixers.PowerInputMixer;
import org.nknsd.teamcode.components.sensors.AprilTagSensor3d;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.controlSchemes.defaults.WheelControlScheme;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;

import java.util.List;

@TeleOp (name = "Cam and Flow TeleOp", group = "Tests")
public class TestCameraTeleOp extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        //this teleop uses technically an outdated, more basic version of what we should do, where it lists all the parts and make them individually.
        //I did not want to remake setup for our new robot especially as we are currently lacking in that stuff rn.

        //gamepads
        GamePadHandler gamePadHandler = new GamePadHandler();
        components.add(gamePadHandler);
        telemetryEnabled.add(gamePadHandler);

        //stateMachine
        components.add(StateMachine.INSTANCE);
        telemetryEnabled.add(StateMachine.INSTANCE);

        //control scheme
        WheelControlScheme wheelControlScheme = new WheelControlScheme();
        wheelControlScheme.link(gamePadHandler);



        //sensors

        AprilTagSensor3d aprilTagSensor3d = new AprilTagSensor3d();
        components.add(aprilTagSensor3d);
        FlowSensor flowSensor = new FlowSensor("flowSens");
        components.add(flowSensor);
        CameraFlowFuser cameraFlowFuser = new CameraFlowFuser(flowSensor, aprilTagSensor3d);

        telemetryEnabled.add(cameraFlowFuser);
        telemetryEnabled.add(flowSensor);

//        IntakeControlScheme intakeControlScheme = new IntakeControlScheme();
//        intakeControlScheme.link(gamePadHandler);
//
//        BallLauncherControlScheme ballLauncherControlScheme = new BallLauncherControlScheme();
//        ballLauncherControlScheme.link(gamePadHandler);

        //wheels
        AbsolutePowerMixer absolutePowerMixer = new AbsolutePowerMixer();
        components.add(absolutePowerMixer);
        MecanumMotorMixer mecanumMotorMixer = new MecanumMotorMixer();
        components.add(mecanumMotorMixer);
        PowerInputMixer powerInputMixer = new PowerInputMixer();
        components.add(powerInputMixer);
        powerInputMixer.link(absolutePowerMixer, mecanumMotorMixer);

        MixedInputWheelDriver mixedInputWheelDriver = new MixedInputWheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(mixedInputWheelDriver);
        mixedInputWheelDriver.link(gamePadHandler, powerInputMixer, wheelControlScheme);



        //intake
//        IntakeHandler intakeHandler = new IntakeHandler();
//        IntakeDriver intakeDriver = new IntakeDriver();
//        components.add(intakeDriver);
//        components.add(intakeHandler);
//        intakeDriver.link(gamePadHandler, intakeControlScheme, intakeHandler);
//        telemetryEnabled.add(intakeHandler);

        //launcher stuff
//        BallLiftDriver ballLiftDriver = new BallLiftDriver();
//        BallLiftHandler ballLiftHandler = new BallLiftHandler();
//        components.add(ballLiftHandler);
//        components.add(ballLiftDriver);
//        ballLiftDriver.link(gamePadHandler, ballLauncherControlScheme, ballLiftHandler);
//        telemetryEnabled.add(ballLiftHandler);

    }
}
