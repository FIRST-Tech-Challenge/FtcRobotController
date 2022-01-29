package com.SCHSRobotics.HAL9001.util.math.datastructures;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * A map datastructure where multiple values under the same key are stored as a list.
 * <p>
 * Creation Date: 5/17/20
 *
 * @param <K> The key datatype.
 * @param <V> The value datatype.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Iterable
 * @see Map
 * @since 1.1.0
 */
public class MultiElementMap<K, V> implements Iterable<List<V>> {
    //The internal storage for this hashmap, relating the key to a list of values.
    private Map<K, List<V>> internalStorage = new HashMap<>();

    /**
     * Gets the size of the map.
     *
     * @return The size of the map.
     */
    public int size() {
        return internalStorage.size();
    }

    /**
     * Gets whether the map is empty.
     *
     * @return Whether the map is empty.
     */
    public boolean isEmpty() {
        return internalStorage.isEmpty();
    }

    /**
     * Gets whether the map has a given key.
     *
     * @param o The key object.
     * @return Whether the map contains the given key.
     */
    @SuppressWarnings("all")
    public boolean containsKey(Object o) {
        return internalStorage.containsKey(o);
    }

    /**
     * Gets whether the map has a given value.
     *
     * @param o The key object.
     * @return Whether the map contains the given value.
     */
    @SuppressWarnings("all")
    public boolean containsValue(Object o) {
        Collection<List<V>> values = internalStorage.values();

        for (List<V> valueList : values) {
            if (valueList.contains(o)) return true;
        }
        return false;
    }

    /**
     * Gets the list of values associated with the given key.
     *
     * @param o The key to get values for.
     * @return The list of values associated with the given key.
     */
    @SuppressWarnings("all")
    public List<V> get(Object o) {
        return internalStorage.get(o);
    }

    /**
     * Puts adds a value under a given key.
     *
     * @param k The key.
     * @param v The value.
     * @return The value.
     */
    public V put(K k, @NotNull V v) {
        List<V> currentlyStored = internalStorage.get(k);
        if (currentlyStored == null) {
            currentlyStored = new ArrayList<>();
            currentlyStored.add(v);
            internalStorage.put(k, currentlyStored);
        } else currentlyStored.add(v);

        return v;
    }

    /**
     * Puts a list of values under a given key.
     *
     * @param k The key.
     * @param v The list of values.
     */
    public void putList(K k, @NotNull List<V> v) {
        internalStorage.put(k, v);
    }

    /**
     * Removes the values associated with a key in the map and returns them.
     *
     * @param o The key.
     * @return The list of values that were associated with that key.
     */
    @SuppressWarnings("all")
    public List<V> remove(Object o) {
        return Objects.requireNonNull(internalStorage.remove(o));
    }

    /**
     * Adds all elements in a map to the internal map.
     *
     * @param map The map object to add to this map.
     */
    public void putAll(@NotNull Map<? extends K, ? extends List<V>> map) {
        for (Map.Entry<? extends K, ? extends List<V>> pair : map.entrySet()) {
            putList(pair.getKey(), pair.getValue());
        }
    }

    /**
     * Clears the ma
     */
    public void clear() {
        for (List<V> valueList : internalStorage.values()) {
            valueList.clear();
        }
        internalStorage.clear();
    }

    /**
     * Gets the map keyset.
     *
     * @return The map keyset.
     */
    @NotNull
    public Set<K> keySet() {
        return internalStorage.keySet();
    }

    /**
     * Gets all the values in the map.
     *
     * @return All the values in the map.
     */
    @NotNull
    public Collection<List<V>> values() {
        return internalStorage.values();
    }

    /**
     * Gets the entrysets for this map.
     *
     * @return The entrysets for this map.
     */
    @NotNull
    public Set<Map.Entry<K, List<V>>> entrySet() {
        return internalStorage.entrySet();
    }

    /**
     * Gets the iterator for this map.
     *
     * @return The iterator for this map.
     */
    @NotNull
    @Override
    public Iterator<List<V>> iterator() {
        return internalStorage.values().iterator();
    }
}