package com.technototes.library.hardware;

public interface HardwareDeviceGroup<T extends HardwareDevice> {
    T[] getFollowers();

    T[] getAllDevices();
}
