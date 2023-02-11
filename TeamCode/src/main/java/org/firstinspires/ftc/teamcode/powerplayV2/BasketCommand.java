package org.firstinspires.ftc.teamcode.powerplayV2;

import com.arcrobotics.ftclib.command.CommandBase;

public class BasketCommand extends CommandBase {
    private BasketSubsystem basket;
    private BasketSubsystem.State basketState;

    public BasketCommand(BasketSubsystem basket) {
        this.basket = basket;
        addRequirements(basket);
    }

    @Override
    public void initialize() {
        basketState = basket.getState();

        if(basketState == BasketSubsystem.State.OUTTAKE) basket.setTravel();
        else basket.setOuttake();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
