package com.acmerobotics.dashboard.config.variable;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * Custom (nested) configuration variable used to represent nested objects.
 */
public class CustomVariable extends ConfigVariable<Object> {
    private Map<String, ConfigVariable> variables;

    public CustomVariable(Map<String, ConfigVariable> variables) {
        this.variables = variables;
    }

    public CustomVariable() {
        this(new HashMap<>());
    }

    public void putVariable(String name, ConfigVariable variable) {
        if (isReserved(name)) {
            throw new RuntimeException();
        }
        variables.put(name, variable);
    }

    public void removeVariable(String name) {
        variables.remove(name);
    }

    public ConfigVariable<?> getVariable(String name) {
        return variables.get(name);
    }

    public int size() {
        return variables.size();
    }

    public Set<Map.Entry<String, ConfigVariable>> entrySet() {
        return variables.entrySet();
    }

    @Override
    public VariableType getType() {
        return VariableType.CUSTOM;
    }

    @Override
    public Object getValue() {
        return variables;
    }

    @SuppressWarnings("unchecked")
    @Override
    public void update(ConfigVariable<Object> newVariable) {
        if (newVariable instanceof CustomVariable) {
            CustomVariable newCustomVariable = (CustomVariable) newVariable;
            for (Map.Entry<String, ConfigVariable> entry : newCustomVariable.variables.entrySet()) {
                ConfigVariable newChildVariable = newCustomVariable.getVariable(entry.getKey());
                getVariable(entry.getKey()).update(newChildVariable);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
