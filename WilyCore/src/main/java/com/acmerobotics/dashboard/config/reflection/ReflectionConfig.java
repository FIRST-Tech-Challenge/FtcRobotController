package com.acmerobotics.dashboard.config.reflection;

import com.acmerobotics.dashboard.config.variable.ConfigVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import java.lang.reflect.Field;

public class ReflectionConfig {
    private ReflectionConfig() {
    }

    public static CustomVariable createVariableFromClass(Class<?> configClass) {
        return null; // ###
    }

    private static ConfigVariable<?> createVariableFromArrayField(Field field, Class<?> fieldClass,
                                                                  Object parent, int[] indices) {
        return null; // ###
    }
}
