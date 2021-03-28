package com.technototes.library.subsystem;

import android.util.Pair;

import com.technototes.library.command.Command;
import com.technototes.library.hardware.HardwareDevice;

import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;

/** Root class for subsystems
 * @author Alex Stedman
 * @param <T> The {@link HardwareDevice} for this subsystem
 */
public abstract class Subsystem<T extends HardwareDevice<?>> {
    private T[] devices;
    private Command defaultCommand;

    /** Create a subsystem
     *
     * @param devices The main devices for the subsystem
     */
    @SafeVarargs
    public Subsystem(T... devices) {
        this.devices = devices;
    }

    /** Set the default command of the subsystem
     *
     * @param command The default command
     * @return this
     */
    public Subsystem<?> setDefaultCommand(Command command){
        defaultCommand = command;
        return this;
    }

    /** Get the devices for this subsystem
     *
     * @return The devices
     */
    public T[] getDevices() {
        return devices;
    }

    /** Get the default command for the subsystem
     *
     * @return The default command
     */
    public Command getDefaultCommand(){
        return defaultCommand;
    }

    public void periodic(){

    }
}
