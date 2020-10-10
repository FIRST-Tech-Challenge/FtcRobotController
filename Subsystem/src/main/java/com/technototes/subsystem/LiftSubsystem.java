package com.technototes.subsystem;

public interface LiftSubsystem extends PositionalSubsystem {
    default void up(){
        goToPosition(Math.min(getPosition(), getMaxPosition()));
    }
    default void down(){
        goToPosition(Math.max(getPosition(), getMinPosition()));
    }
    default void goToTop(){
        goToPosition(getMaxPosition());
    }
    default void goToBottom(){
        goToPosition(getMinPosition());
    }
}
