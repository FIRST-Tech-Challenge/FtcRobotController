package com.acmerobotics.dashboard.config.variable;

import com.acmerobotics.dashboard.config.ValueProvider;

public class BasicVariable<T> extends ConfigVariable<T> {
    private VariableType type;
    private ValueProvider<T> provider;

    private static <T> VariableType inferType(ValueProvider<T> provider) {
        Class<?> providerClass = provider.get().getClass();
        return VariableType.fromClass(providerClass);
    }

    public BasicVariable(ValueProvider<T> provider) {
        this(inferType(provider), provider);
    }

    public BasicVariable(VariableType type, ValueProvider<T> provider) {
        this.type = type;
        this.provider = provider;
    }

    @Override
    public VariableType getType() {
        return type;
    }

    @Override
    public T getValue() {
        return provider.get();
    }

    @Override
    public void update(ConfigVariable<T> newVariable) {
        provider.set(newVariable.getValue());
    }
}
