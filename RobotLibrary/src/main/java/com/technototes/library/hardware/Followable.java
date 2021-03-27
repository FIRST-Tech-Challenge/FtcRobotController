package com.technototes.library.hardware;

/** Simple interface for devices that can follow eachother
 * @author Alex Stedman
 * @param <T> The device class
 */
@Deprecated
public interface Followable<T extends HardwareDevice> {
    /** Follow function
     *
     * @param device The device to follow
     * @return The interfaced Device
     */
    T follow(T device);
}
