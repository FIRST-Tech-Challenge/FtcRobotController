package com.SCHSRobotics.HAL9001.system.gui;

import com.SCHSRobotics.HAL9001.util.misc.UniqueID;

import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * A data transfer object used to transfer data between menus.
 * <p>
 * Creation Date: 4/29/20
 *
 * @author Cole Savage, Level Up
 * @version 1.1.0
 * @see HALMenu
 * @see UniqueID
 * @since 1.1.0
 */
public class Payload {
    //Maps unique ids to instances of objects.
    private final Map<UniqueID, Object> objectMap = new HashMap<>();
    //Maps unique ids to classes of objects.
    private final Map<UniqueID, Class<?>> classMap = new HashMap<>();

    /**
     * Adds an entry to the payload.
     *
     * @param id  The unique id that will be used as a key.
     * @param obj The object to add to the payload.
     * @return This payload.
     * @see UniqueID
     */
    public Payload add(UniqueID id, Object obj) {
        objectMap.put(id, obj);
        classMap.put(id, obj.getClass());
        return this;
    }

    /**
     * Copies all entries in the given payload with the specified ids to this payload.
     *
     * @param otherPayload The source of the entries for copying.
     * @param idsToAdd     The ids of the entries to copy.
     * @return This payload.
     * @see UniqueID
     */
    public Payload copyFrom(Payload otherPayload, @NotNull UniqueID... idsToAdd) {
        for (UniqueID id : idsToAdd) {
            if (otherPayload.idPresent(id)) {
                add(id, otherPayload.get(id));
            }
        }
        return this;
    }

    /**
     * Removes an entry from the payload.
     *
     * @param id  The id of the entry to remove.
     * @param <T> The type of the entry to remove.
     * @return The entry being removed from the payload.
     * @see UniqueID
     */
    @SuppressWarnings("unchecked")
    public <T> T remove(UniqueID id) {
        if (!idPresent(id)) return null;

        Object obj = objectMap.remove(id);
        Class<T> clazz = Objects.requireNonNull((Class<T>) classMap.remove(id));
        return clazz.cast(obj);
    }

    /**
     * Gets en entry from the payload without removing it.
     *
     * @param id  The id of the entry to get.
     * @param <T> The type of the entry to get.
     * @return The entry listed under the specified id in the payload.
     * @see UniqueID
     */
    @SuppressWarnings("unchecked")
    public <T> T get(UniqueID id) {
        if (!idPresent(id)) return null;

        Object obj = objectMap.get(id);
        Class<T> clazz = Objects.requireNonNull((Class<T>) classMap.get(id));
        return clazz.cast(obj);
    }

    /**
     * Returns whether there is an entry in the payload under the given id.
     *
     * @param id The id to search under.
     * @return Whether there is an entry in the payload under the given id.
     * @see UniqueID
     */
    public boolean idPresent(UniqueID id) {
        return objectMap.containsKey(id);
    }

    @NotNull
    @Override
    public String toString() {
        return objectMap.keySet().toString();
    }
}