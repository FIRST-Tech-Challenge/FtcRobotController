/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.config;

import static org.firstinspires.ftc.teamcode.follower.utils.Utils.concat;
import static org.firstinspires.ftc.teamcode.follower.utils.Utils.listOf;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class ConfigVar<T> implements Supplier<T> {
    private final List<Validator<T>> validators;
    private T value;
    private boolean hasValue;

    /**
     * Creates a new ConfigVar with the given value and validators.
     */
    private ConfigVar(T value, List<Validator<T>> validators) {
        this.value = value;
        this.hasValue = true;
        this.validators = Collections.unmodifiableList(validators);
        validate(value);
    }

    /**
     * Creates a new ConfigVar with no value and the given validators.
     */
    private ConfigVar(List<Validator<T>> validators) {
        this.hasValue = false;
        this.validators = Collections.unmodifiableList(validators);
    }

    /**
     * Creates a new ConfigVar of given type with the given validators.
     * The value must be set before it can be used in configurations.
     */
    public static <T> ConfigVar<T> required(List<Validator<T>> validators) {
        return new ConfigVar<>(concat(validators, Collections.singletonList(Validator.nonnull())));
    }

    /**
     * Creates a new ConfigVar of given type with the given validators.
     * The value must be set before it can be used in configurations.
     */
    @SafeVarargs
    public static <T> ConfigVar<T> required(Validator<T>... validators) {
        return required(listOf(validators));
    }

    /**
     * Creates a new ConfigVar of given type with the given validators.
     * The value may be null, but if it is not null, it must pass the validators.
     */
    public static <T> ConfigVar<T> requiredNullable(List<Validator<T>> validators) {
        return new ConfigVar<>(validators);
    }

    /**
     * Creates a new ConfigVar of given type with the given validators.
     * The value may be null, but if it is not null, it must pass the validators.
     */
    @SafeVarargs
    public static <T> ConfigVar<T> requiredNullable(Validator<T>... validators) {
        return requiredNullable(listOf(validators));
    }

    /**
     * Creates a new ConfigVar of given type with the given value and validators.
     * This value serves as a "default" value, and will be used if the value is not set in the configuration.
     * The value must pass the validators and not be null.
     */
    public static <T> ConfigVar<T> of(T value, List<Validator<T>> validators) {
        return new ConfigVar<>(value, concat(validators, Collections.singletonList(Validator.nonnull())));
    }

    /**
     * Creates a new ConfigVar of given type with the given value and validators.
     * This value serves as a "default" value, and will be used if the value is not set in the configuration.
     * The value must pass the validators and not be null.
     */
    @SafeVarargs
    public static <T> ConfigVar<T> of(T value, Validator<T>... validators) {
        return of(value, listOf(validators));
    }

    /**
     * Creates a new ConfigVar of given type with the given value and validators.
     * This value serves as a "default" value, and will be used if the value is not set in the configuration.
     * The value may be null, but if it is not null, it must pass the validators.
     */
    public static <T> ConfigVar<T> ofNullable(T value, List<Validator<T>> validators) {
        return new ConfigVar<>(value, validators);
    }

    /**
     * Creates a new ConfigVar of given type with the given value and validators.
     * This value serves as a "default" value, and will be used if the value is not set in the configuration.
     * The value may be null, but if it is not null, it must pass the validators.
     */
    @SafeVarargs
    public static <T> ConfigVar<T> ofNullable(T value, Validator<T>... validators) {
        return ofNullable(value, listOf(validators));
    }

    /**
     * Returns the value of this ConfigVar.
     * If the value has not been set, it will throw an IllegalStateException.
     */
    @Override
    public T get() {
        require();
        validate(value);
        return value;
    }

    /**
     * Sets the value of this ConfigVar.
     * The value must pass the validators of this ConfigVar.
     */
    public void set(T value) {
        this.value = value;
        this.hasValue = true;
    }

    /**
     * Returns a Modifier that temporarily sets the value of this ConfigVar to the given value.
     * The value must pass the validators of this ConfigVar.
     */
    public Modifier at(T tempValue) {
        validate(tempValue);
        return new Modifier() {
            private T originalValue;

            @Override
            public void apply() {
                require();
                originalValue = value;
                value = tempValue;
            }

            @Override
            public void revert() {
                value = originalValue;
            }
        };
    }

    /**
     * Validates the given value against the validators of this ConfigVar, throwing an IllegalArgumentException if any validator fails.
     */
    private void validate(T value) {
        for (Validator<T> validator : validators) {
            if (!validator.validate(value)) {
                throw new IllegalArgumentException("Invalid value for config variable of " + value);
            }
        }
    }

    /**
     * Throws an IllegalStateException if the value has not been set.
     */
    private void require() {
        if (!hasValue) throw new IllegalStateException("Config variable has not been set");
    }
}
