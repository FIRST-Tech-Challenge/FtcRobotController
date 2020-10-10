package com.technototes.subsystem;

public interface IndexerSubsystem extends IntakeSubsystem {
    void empty(int num);
    default void emptyOne(){
        empty(1);
    }
    default void unjam(){
        extake();
    }
    default void emptyAll(){
        intake();
    }
}
