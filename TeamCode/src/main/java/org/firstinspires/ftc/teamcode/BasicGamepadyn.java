package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import computer.living.gamepadyn.EActionType;
import computer.living.gamepadyn.Gamepadyn;
import computer.living.gamepadyn.IUserAction;

@TeleOp
public class BasicGamepadyn extends OpMode {

    enum UserAction implements IUserAction {
        ACTION_DEBUG_1,
        ACTION_DEBUG_2(2);

        final int axes;
        UserAction() { this.axes = 0; }
        UserAction(int axes) { this.axes = axes; }
        @Override
        public int getAxes() { return this.axes; }
    }

    Gamepadyn<UserAction> gamepadyn = new Gamepadyn<>(this, UserAction.values());

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
