package com.SCHSRobotics.HAL9001.util.math.datastructures;

import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.exceptions.HALMathException;

import org.jetbrains.annotations.NotNull;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * A bidirectional version of a hashmap. Each key can be used to look up a value and each value can be used to look up a key.
 * <p>
 * Creation Date: 5/17/20
 *
 * @param <K> The datatype of the keys.
 * @param <V> The datatype of the values.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Map
 * @since 1.1.0
 */
public class BidirectionalMap<K, V> implements Map<K, V> {

    //A map relating the keys to the values.
    private Map<K, V> forwardMap = new HashMap<>();
    //A map relating the values to the keys.
    private Map<V, K> reverseMap = new HashMap<>();

    /**
     * A simple constructor for BidirectionalMap
     */
    public BidirectionalMap() {
    }

    /**
     * A constructor for BidirectionalMap that allows you to convert a normal map into a bidirectional map, given that no keys map to the same value.
     *
     * @param forwardMap A map relating unique keys to unique values.
     * @throws HALMathException Throws this exception when the values in a map are not unique.
     */
    public BidirectionalMap(@NotNull Map<K, V> forwardMap) {
        this.forwardMap = forwardMap;
        this.reverseMap = new HashMap<>();
        for (Entry<K, V> p : forwardMap.entrySet()) {
            ExceptionChecker.assertFalse(reverseMap.containsKey(p.getValue()), new HALMathException("Duplicate value detected: Key value pairs in a bidirectional map must be invertible."));
            reverseMap.put(p.getValue(), p.getKey());
        }
    }

    @Override
    public int size() {
        return forwardMap.size();
    }

    @Override
    public boolean isEmpty() {
        return forwardMap.isEmpty();
    }

    @Override
    public boolean containsKey(Object o) {
        return forwardMap.containsKey(o);
    }

    @Override
    public boolean containsValue(Object o) {
        return forwardMap.containsValue(o);
    }

    @Override
    public V get(Object o) {
        return forwardMap.get(o);
    }

    /**
     * Gets a value for a given key.
     *
     * @param o The key to look up in the map.
     * @return The value associated with that key, or null otherwise.
     */
    public V getForward(Object o) {
        return get(o);
    }

    /**
     * Gets a key for a given value.
     *
     * @param o The value to look up the key for in the map.
     * @return The key associated with that value.
     */
    public K getReverse(Object o) {
        return reverseMap.get(o);
    }

    @Override
    public V put(@NotNull K k, @NotNull V v) {
        forwardMap.put(k, v);
        reverseMap.put(v, k);
        return v;
    }

    /**
     * Puts a key, value pair into the map.
     *
     * @param k The key.
     * @param v The value.
     * @return The value.
     */
    public V putForward(@NotNull K k, @NotNull V v) {
        return put(k, v);
    }

    /**
     * Puts a value, key pair into the map.
     *
     * @param v The value.
     * @param k The key.
     * @return The key.
     */
    public K putReverse(@NotNull V v, @NotNull K k) {
        forwardMap.put(k, v);
        reverseMap.put(v, k);
        return k;
    }

    @Override
    public V remove(Object o) {
        V val = forwardMap.remove(o);
        reverseMap.remove(val);
        return val;
    }

    @Override
    public void putAll(@NotNull Map<? extends K, ? extends V> map) {
        forwardMap.putAll(map);
        for (Entry<? extends K, ? extends V> entry : map.entrySet()) {
            reverseMap.put(entry.getValue(), entry.getKey());
        }
    }

    /**
     * Inverts the bidirectional map, making keys into values and values into keys.
     *
     * @return An inverted version of this bidirectional map.
     */
    public BidirectionalMap<V, K> invert() {
        return new BidirectionalMap<>(reverseMap);
    }

    @Override
    public void clear() {
        forwardMap.clear();
        reverseMap.clear();
    }

    @Override
    public @NotNull Set<K> keySet() {
        return forwardMap.keySet();
    }

    @Override
    public @NotNull Collection<V> values() {
        return forwardMap.values();
    }

    @Override
    public @NotNull Set<Entry<K, V>> entrySet() {
        return forwardMap.entrySet();
    }

    @Override
    public boolean equals(Object o) {
        if (o.getClass() == BidirectionalMap.class) return false;

        return forwardMap.equals(o);
    }

    @Override
    public int hashCode() {
        return forwardMap.hashCode();
    }
}