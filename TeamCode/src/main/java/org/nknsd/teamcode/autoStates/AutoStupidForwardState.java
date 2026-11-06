package org.nknsd.teamcode.autoStates;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.drivers.MixedInputWheelDriver;
import org.nknsd.teamcode.components.motormixers.PowerInputMixer;
import org.nknsd.teamcode.components.utility.StateMachine;

public class AutoStupidForwardState extends StateMachine.State{
    private final PowerInputMixer powerInputMixer;

    public AutoStupidForwardState(PowerInputMixer powerInputMixer) {
        this.powerInputMixer = powerInputMixer;
    }

    @Override
    protected void run(ElapsedTime runtime, Telemetry telemetry) {
        powerInputMixer.setManualPowers(new double[]{-0.5, 0, 0});
        if(runtime.milliseconds() > 500 + startTimeMS){
            StateMachine.INSTANCE.stopAnonymous(this);
        }
    }

    @Override
    protected void started() {

    }

    @Override
    protected void stopped() {
        powerInputMixer.setManualPowers(new double[]{0, 0, 0});
    }
}
