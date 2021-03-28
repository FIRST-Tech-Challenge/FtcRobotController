package com.technototes.library.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

/** Class for hardware devices
 * @author Alex Stedman
 * @param <T> The class for the default device (ones found in ftcsdk)
 */
public abstract class HardwareDevice<T extends com.qualcomm.robotcore.hardware.HardwareDevice> {
    /** Hardware map object for stuff
     *
     */
    public static HardwareMap hardwareMap = null;
    private T device;

    /** Make a hardware device
     *
     * @param device The default device
     */
    public HardwareDevice(T device) {
        this.device = device;
    }

    /** Make a hardware device with the string to get from robotmap
     *
     * @param deviceName The device name
     */
    public HardwareDevice(String deviceName) {
        this(hardwareMap.get((Class<T>) com.qualcomm.robotcore.hardware.HardwareDevice.class /*T.class*/, deviceName));
    }

    /** Get encapsulated device
     *
     * @return The device
     */
    public T getDevice() {
        return device;
    }
}
