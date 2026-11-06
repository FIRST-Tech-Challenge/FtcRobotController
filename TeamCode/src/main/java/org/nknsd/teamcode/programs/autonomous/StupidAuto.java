package org.nknsd.teamcode.programs.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.nknsd.teamcode.autoStates.AutoStupidLaunchState;
import org.nknsd.teamcode.components.drivers.MixedInputWheelDriver;
import org.nknsd.teamcode.components.handlers.BallLiftHandler;
import org.nknsd.teamcode.components.handlers.odometry.AbsolutePosition;
import org.nknsd.teamcode.components.motormixers.AbsolutePowerMixer;
import org.nknsd.teamcode.components.motormixers.MecanumMotorMixer;
import org.nknsd.teamcode.components.motormixers.PowerInputMixer;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;

import java.util.List;

@Autonomous(name = "StupidAuto")
public class StupidAuto extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {

        components.add(StateMachine.INSTANCE);
        telemetryEnabled.add(StateMachine.INSTANCE);

        BallLiftHandler ballLiftHandler = new BallLiftHandler();
        components.add(ballLiftHandler);

        PowerInputMixer powerInputMixer = new PowerInputMixer();
        components.add(powerInputMixer);
        AbsolutePowerMixer absolutePowerMixer = new AbsolutePowerMixer();
        MecanumMotorMixer mecanumMotorMixer = new MecanumMotorMixer();
        components.add(absolutePowerMixer);
        components.add(mecanumMotorMixer);

        powerInputMixer.setDirectPower(false);
        powerInputMixer.link(absolutePowerMixer, mecanumMotorMixer);


        //this is so stupid and will likely break terribly
        StateMachine.INSTANCE.startAnonymous(new AutoStupidLaunchState(ballLiftHandler, powerInputMixer));
    }
}
