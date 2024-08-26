package com.acmerobotics.dashboard.config.variable;

import java.util.HashSet;
import java.util.Set;

/**
 * Type-independent dashboard configuration variable.
 */
public abstract class ConfigVariable<T> {
    public static final String TYPE_KEY = "__type";
    public static final String VALUE_KEY = "__value";
    public static final String ENUM_CLASS_KEY = "__enumClass";
    public static final String ENUM_VALUES_KEY = "__enumValues";

    // Set.of() is unfortunately unavailable
    private static final Set<String> RESERVED_KEYS = new HashSet<>();

    static {
        RESERVED_KEYS.add(TYPE_KEY);
        RESERVED_KEYS.add(VALUE_KEY);
        RESERVED_KEYS.add(ENUM_CLASS_KEY);
        RESERVED_KEYS.add(ENUM_VALUES_KEY);
    }

    public static boolean isReserved(String name) {
        return RESERVED_KEYS.contains(name);
    }

    public abstract VariableType getType();

    public abstract T getValue();

    public abstract void update(ConfigVariable<T> newVariable);
}
