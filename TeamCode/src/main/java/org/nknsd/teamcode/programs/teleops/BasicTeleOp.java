package org.nknsd.teamcode.programs.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.nknsd.teamcode.components.drivers.BallLiftDriver;
import org.nknsd.teamcode.components.drivers.IntakeDriver;
import org.nknsd.teamcode.components.drivers.MixedInputWheelDriver;
import org.nknsd.teamcode.components.handlers.BallLiftHandler;
import org.nknsd.teamcode.components.handlers.intakeTurret.IntakeHandler;
import org.nknsd.teamcode.components.handlers.gamepad.GamePadHandler;
import org.nknsd.teamcode.components.motormixers.AbsolutePowerMixer;
import org.nknsd.teamcode.components.motormixers.MecanumMotorMixer;
import org.nknsd.teamcode.components.motormixers.PowerInputMixer;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.controlSchemes.defaults.BallLauncherControlScheme;
import org.nknsd.teamcode.controlSchemes.defaults.IntakeControlScheme;
import org.nknsd.teamcode.controlSchemes.defaults.WheelControlScheme;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;

import java.util.List;

@TeleOp (name = "basicTeleOp")
public class BasicTeleOp extends NKNProgram {
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

        IntakeControlScheme intakeControlScheme = new IntakeControlScheme();
        intakeControlScheme.link(gamePadHandler);

        BallLauncherControlScheme ballLauncherControlScheme = new BallLauncherControlScheme();
        ballLauncherControlScheme.link(gamePadHandler);

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
        IntakeHandler intakeHandler = new IntakeHandler();
        IntakeDriver intakeDriver = new IntakeDriver();
        components.add(intakeDriver);
        components.add(intakeHandler);
        intakeDriver.link(gamePadHandler, intakeControlScheme, intakeHandler);
        telemetryEnabled.add(intakeHandler);

        //launcher stuff
        BallLiftDriver ballLiftDriver = new BallLiftDriver();
        BallLiftHandler ballLiftHandler = new BallLiftHandler();
        components.add(ballLiftHandler);
        components.add(ballLiftDriver);
        ballLiftDriver.link(gamePadHandler, ballLauncherControlScheme, ballLiftHandler);
        telemetryEnabled.add(ballLiftHandler);

    }
}
