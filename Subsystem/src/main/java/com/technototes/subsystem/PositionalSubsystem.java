package com.technototes.subsystem;

public interface PositionalSubsystem {
    int getMaxPosition();
    default int getMinPosition(){
        return 0;
    }
    int getPosition();
    void goToPosition(int position);
}
