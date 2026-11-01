/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.utils;

import java.util.Map;
import java.util.TreeMap;

/**
 * A numeric bijective map that allows for interpolation and finding closest keys/values.
 * This map supports double keys and values, allowing for numeric operations.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class BijectiveMap {
    private final InterpolatableMap forwardMap;
    private final InterpolatableMap reverseMap;

    public BijectiveMap(InterpolatableMap forwardMap, InterpolatableMap reverseMap) {
        this.forwardMap = forwardMap;
        this.reverseMap = reverseMap;
    }

    /**
     * A map that allows for interpolation and finding closest keys/values.
     * This map supports double keys and values, allowing for numeric operations.
     */
    public static class InterpolatableMap extends TreeMap<Double, Double> {
        public InterpolatableMap() {
            super(Double::compareTo);
        }

        /**
         * Finds the closest key to the given key within a specified threshold.
         * @param key the key to find the closest match for
         * @param threshold the maximum difference allowed to consider a key as closest
         * @return the closest key within the threshold, or NaN if no key is found within the threshold
         */
        private double findClosestInput(double key, double threshold) {
            Map.Entry<Double, Double> lowerVal = floorEntry(key);
            Map.Entry<Double, Double> higherVal = ceilingEntry(key);

            if (lowerVal == null && higherVal == null) {
                return Double.NaN; // No keys available
            }

            double lowerKey = (lowerVal != null) ? lowerVal.getKey() : Double.NEGATIVE_INFINITY;
            double higherKey = (higherVal != null) ? higherVal.getKey() : Double.POSITIVE_INFINITY;
            double lowerDiff = Math.abs(lowerKey - key);
            double higherDiff = Math.abs(higherKey - key);

            if (lowerDiff <= threshold && lowerDiff <= higherDiff) {
                return lowerKey;
            } else if (higherDiff <= threshold && higherDiff < lowerDiff) {
                return higherKey;
            } else {
                return Double.NaN; // No key within the threshold
            }
        }

        /**
         * Finds the closest key to the given key.
         * @param key the key to find the closest match for
         * @return the closest key, or NaN if no keys are available
         */
        private double findClosestInput(double key) {
            Map.Entry<Double, Double> lowerVal = floorEntry(key);
            Map.Entry<Double, Double> higherVal = ceilingEntry(key);

            if (lowerVal == null && higherVal == null) {
                return Double.NaN; // No keys available
            }

            double lowerKey = (lowerVal != null) ? lowerVal.getKey() : Double.NEGATIVE_INFINITY;
            double higherKey = (higherVal != null) ? higherVal.getKey() : Double.POSITIVE_INFINITY;
            double lowerDiff = Math.abs(lowerKey - key);
            double higherDiff = Math.abs(higherKey - key);

            if (lowerDiff <= higherDiff) {
                return lowerKey;
            } else {
                return higherKey;
            }
        }

        /**
         * Interpolates the value for a given key based on the surrounding keys and values.
         * If the key is outside the range of existing keys, it returns the closest value.
         * @param key the key to interpolate
         * @return the interpolated value, or NaN if no keys are available
         */
        private double interpolate(double key) {
            Map.Entry<Double, Double> lowerEntry = floorEntry(key);
            Map.Entry<Double, Double> higherEntry = ceilingEntry(key);

            if (lowerEntry == null && higherEntry == null) {
                return Double.NaN; // No keys available
            }

            if (lowerEntry == null) {
                return higherEntry.getValue(); // Only higher key exists
            }

            if (higherEntry == null) {
                return lowerEntry.getValue(); // Only lower key exists
            }

            if (higherEntry.equals(lowerEntry)) return lowerEntry.getValue();

            double lowerKey = lowerEntry.getKey();
            double higherKey = higherEntry.getKey();
            double lowerValue = lowerEntry.getValue();
            double higherValue = higherEntry.getValue();

            if (lowerKey == higherKey) return lowerValue; // Both keys are the same

            // Linear interpolation
            return lowerValue + (higherValue - lowerValue) * ((key - lowerKey) / (higherKey - lowerKey));
        }
    }

    public BijectiveMap() {
        forwardMap = new InterpolatableMap();
        reverseMap = new InterpolatableMap();
    }

    public void put(Double key, Double value) {
        forwardMap.put(key, value);
        reverseMap.put(value, key);
    }

    public Double get(Double key) {
        return forwardMap.get(key);
    }

    public Double invert(Double value) {
        return reverseMap.get(value);
    }

    public boolean containsKey(Double key) {
        return forwardMap.containsKey(key);
    }

    public boolean containsValue(Double value) {
        return reverseMap.containsKey(value);
    }

    public void remove(Double key) {
        Double value = forwardMap.remove(key);
        if (value != null) {
            reverseMap.remove(value);
        }
    }

    public void removeValue(Double value) {
        Double key = reverseMap.remove(value);
        if (key != null) {
            forwardMap.remove(key);
        }
    }

    public int size() {
        return forwardMap.size();
    }

    public TreeMap<Double, Double> getForwardMap() {
        return forwardMap;
    }

    public TreeMap<Double, Double> getReverseMap() {
        return reverseMap;
    }

    /**
     * Finds the closest key to the given key within a specified threshold.
     * @param key the key to find the closest match for
     * @param threshold the maximum difference allowed to consider a key as closest
     * @return the closest key within the threshold, or NaN if no key is found within the threshold
     */
    public double closestKey(double key, double threshold) {
        return forwardMap.findClosestInput(key, threshold);
    }

    /**
     * Finds the closest key to the given key.
     * @param key the key to find the closest match for
     * @return the closest key, or NaN if no keys are available
     */
    public double closestKey(double key) {
        return forwardMap.findClosestInput(key);
    }

    /**
     * Finds the closest value to the given value within a specified threshold.
     * @param value the value to find the closest match for
     * @param threshold the maximum difference allowed to consider a value as closest
     * @return the closest value within the threshold, or NaN if no value is found within the threshold
     */
    public double closestValue(double value, double threshold) {
        return reverseMap.findClosestInput(value, threshold);
    }

    /**
     * Finds the closest value to the given value.
     * @param value the value to find the closest match for
     * @return the closest value, or NaN if no values are available
     */
    public double closestValue(double value) {
        return reverseMap.findClosestInput(value);
    }

    /**
     * Interpolates the value for a given key based on the surrounding keys and values.
     * If the key is outside the range of existing keys, it returns the closest value.
     * @param key the key to interpolate
     * @return the interpolated value, or NaN if no keys are available
     */
    public double interpolateKey(double key) {
        return forwardMap.interpolate(key);
    }

    /**
     * Interpolates the value for a given value based on the surrounding values and keys.
     * If the value is outside the range of existing values, it returns the closest key.
     * @param value the value to interpolate
     * @return the interpolated value, or NaN if no values are available
     */
    public double interpolateValue(double value) {
        return reverseMap.interpolate(value);
    }
}
