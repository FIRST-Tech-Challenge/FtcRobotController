package com.acmerobotics.dashboard.config.variable;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;
import java.lang.reflect.Type;

public class ConfigVariableSerializer implements JsonSerializer<ConfigVariable<?>> {
    @Override
    public JsonElement serialize(ConfigVariable<?> configVariable, Type type,
                                 JsonSerializationContext jsonSerializationContext) {
        Object value = configVariable.getValue();

        JsonObject obj = new JsonObject();
        obj.add(ConfigVariable.TYPE_KEY,
            jsonSerializationContext.serialize(configVariable.getType()));

        if (value == null) {
            obj.add(ConfigVariable.VALUE_KEY, null);
            return obj;
        }

        if (configVariable.getType() == VariableType.DOUBLE && !Double.isFinite((double) value)) {
            obj.add(ConfigVariable.VALUE_KEY, new JsonPrimitive(String.valueOf(value)));
        } else {
            obj.add(ConfigVariable.VALUE_KEY,
                jsonSerializationContext.serialize(value));
        }

        if (configVariable.getType() == VariableType.ENUM) {
            obj.add(ConfigVariable.ENUM_CLASS_KEY, new JsonPrimitive(
                value.getClass().getName()));
            JsonArray values = new JsonArray();
            for (Object o : value.getClass().getEnumConstants()) {
                values.add(o.toString());
            }
            obj.add(ConfigVariable.ENUM_VALUES_KEY, values);
        }

        return obj;
    }
}
