package com.SCHSRobotics.HAL9001.system.gui.event;

import com.SCHSRobotics.HAL9001.util.math.datastructures.MultiElementMap;

import org.jetbrains.annotations.NotNull;

import java.util.Set;

/**
 * A packet of tasks. Used to easily run large classes of provided tasks.
 * <p>
 * Creation Date: 5/17/20
 *
 * @param <T> The type of object that will be used as a key to look up individual tasks.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Task
 * @see MultiElementMap
 * @since 1.1.0
 */
public final class TaskPacket<T> {
    //A multi-element map linking objects of type T with tasks.
    private final MultiElementMap<T, Task> tasks = new MultiElementMap<>();

    /**
     * Adds a task to the packet under the special key, null.
     *
     * @param task The task to add.
     * @see Task
     * @see MultiElementMap
     */
    public final void add(Task task) {
        add(null, task);
    }

    /**
     * Adds a task to the packet under the specified key.
     *
     * @param key  The key that the task will be added under.
     * @param task The task to add.
     * @see Task
     * @see MultiElementMap
     */
    public final void add(T key, Task task) {
        tasks.put(key, task);
    }

    /**
     * Gets the set of all valid keys.
     *
     * @return The set of all valid keys.
     * @see MultiElementMap
     */
    @NotNull
    public final Set<T> getValidKeys() {
        return tasks.keySet();
    }

    /**
     * Runs all tasks that have been added under the given key.
     *
     * @param key    The key for the list of tasks to run.
     * @param packet The data packet that will be given to the tasks.
     * @see MultiElementMap
     * @see Task
     * @see DataPacket
     */
    public void runTasks(T key, DataPacket packet) {
        if (tasks.containsKey(key)) {
            for (Task task : tasks.get(key)) {
                task.run(packet);
            }
        }
    }

    /**
     * Runs all tasks that have been added under the special key, null.
     *
     * @param packet The data packet that will be given to the tasks.
     * @see MultiElementMap
     * @see Task
     * @see DataPacket
     */
    public void runTasks(DataPacket packet) {
        runTasks(null, packet);
    }
}