package com.kalipsorobotics.actions;

public class DoneStateAction extends Action {

    public DoneStateAction() {
        isDone = true;
    }

    @Override
    public boolean checkDoneCondition() {
        return true;
    }

    @Override
    public void update() {
    }
}
