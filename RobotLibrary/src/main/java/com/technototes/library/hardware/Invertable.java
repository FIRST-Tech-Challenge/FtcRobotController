package com.technototes.library.hardware;

public interface Invertable<T extends HardwareDevice> {
    T setInverted(boolean val);

    boolean getInverted();
}
