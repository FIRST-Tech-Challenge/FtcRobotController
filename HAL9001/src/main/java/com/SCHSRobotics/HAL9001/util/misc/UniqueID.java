package com.SCHSRobotics.HAL9001.util.misc;

import org.jetbrains.annotations.NotNull;

/**
 * A class used for creating Unique identifiers.
 * <p>
 * Creation Date: 9/14/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.1.0
 */
public class UniqueID {
    //The global identifier integer. Used to create unique ids so that they are REALLY unique to each implementation.
    private static int globalIdentifier = 0;
    //The local integer id associated with this unique id instance. Each instance of unique id will have its own unique local identifier.
    private int localIdentifier;
    //The text associated with the id object. This is only used for debugging purposes, the real comparison happens using the localIdentifier variable.
    private String id;

    /**
     * The constructor for Unique id.
     *
     * @param id The text associated with the id object. This is only used for debugging purposes, as each instance is unique regardless of the id string.
     */
    public UniqueID(String id) {
        this.id = id;
        localIdentifier = globalIdentifier;
        globalIdentifier++;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof UniqueID) {
            UniqueID otherId = (UniqueID) obj;
            return otherId.localIdentifier == this.localIdentifier;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return localIdentifier;
    }

    @NotNull
    @Override
    public String toString() {
        return id;
    }
}