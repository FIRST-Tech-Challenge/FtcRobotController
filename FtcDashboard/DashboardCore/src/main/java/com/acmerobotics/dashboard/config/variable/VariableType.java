package com.acmerobotics.dashboard.config.variable;

import com.google.gson.annotations.SerializedName;

/**
 * Types of variables supported by the dashboard.
 */
public enum VariableType {
    @SerializedName("boolean")
    BOOLEAN,

    @SerializedName("int")
    INT,

    @SerializedName("long")
    LONG,

    @SerializedName("float")
    FLOAT,

    @SerializedName("double")
    DOUBLE,

    @SerializedName("string")
    STRING,

    @SerializedName("enum")
    ENUM,

    @SerializedName("custom")
    CUSTOM;

    /**
     * Returns the variable type corresponding to the class provided.
     *
     * @param klass
     */
    public static VariableType fromClass(Class<?> klass) {
        if (klass == Boolean.class || klass == boolean.class) {
            return BOOLEAN;
        } else if (klass == Integer.class || klass == int.class) {
            return INT;
        } else if (klass == Long.class || klass == long.class) {
            return LONG;
        } else if (klass == Float.class || klass == float.class) {
            return FLOAT;
        } else if (klass == Double.class || klass == double.class) {
            return DOUBLE;
        } else if (klass == String.class) {
            return STRING;
        } else if (klass.isEnum()) {
            return ENUM;
        } else {
            return CUSTOM;
        }
    }
}
