package com.technototes.subsystem;

public interface OpenCloseSubsystem {

    enum State{
        OPENED, CLOSED
    }
    State getState();
    void open();
    void close();
}
