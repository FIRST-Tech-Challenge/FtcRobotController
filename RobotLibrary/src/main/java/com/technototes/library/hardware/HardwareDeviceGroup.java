package com.technototes.library.hardware;

/** Interface for hardware device groups
 * @author Alex Stedman
 * @param <T>
 */
public interface HardwareDeviceGroup<T extends HardwareDevice> {
    /** Get the followers for the lead device
     *
     * @return The followers
     */
    T[] getFollowers();

    /** Get all devices in group
     *
     * @return All devices
     */
    T[] getAllDevices();

    /** Propogate actions across the followers
     *
     * @param value the value to propogate
     */
    void propogate(double value);
}
