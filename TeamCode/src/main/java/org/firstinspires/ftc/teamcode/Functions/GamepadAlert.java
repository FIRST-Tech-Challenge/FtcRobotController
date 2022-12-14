package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadAlert {
    Gamepad gamepad1, gamepad2;
    Gamepad.RumbleEffect customEffect;

    public GamepadAlert(Gamepad _gamepad1, Gamepad _gamepad2){
        gamepad1 = _gamepad1;
        gamepad2 = _gamepad2;
        customEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0, 1, 500)
                .addStep(0, 0, 300)
                .addStep(1, 0, 250)
                .addStep(0, 0, 250)
                .addStep(1, 0, 250)
                .build();

    }

    public void AlertDrivers(double timestamp, int seconds, double runtime){
        if(timestamp<=runtime&&timestamp+seconds>=runtime) {
            if(!gamepad1.isRumbling()) {
                gamepad1.rumble(1000 * seconds);
            }
            if(!gamepad2.isRumbling()) {
                gamepad2.rumble(1000 * seconds);
            }
        }
    }

    public void Alert(int blips){
        if(!gamepad1.isRumbling()) {
            gamepad1.rumbleBlips(blips);
        }
        if(!gamepad2.isRumbling()) {
            gamepad2.rumbleBlips(blips);
        }

    }


    public void Stop(){
        gamepad1.stopRumble();
        gamepad2.stopRumble();
    }

    public void AlertCustom(){
        if(!gamepad1.isRumbling()) {
            gamepad1.runRumbleEffect(customEffect);
        }
        if(!gamepad2.isRumbling()) {
            gamepad2.runRumbleEffect(customEffect);
        }
    }


}
