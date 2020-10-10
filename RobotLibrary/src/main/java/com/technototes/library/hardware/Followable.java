package com.technototes.library.hardware;

public interface Followable<T extends HardwareDevice> {
    T follow(T d);
}
