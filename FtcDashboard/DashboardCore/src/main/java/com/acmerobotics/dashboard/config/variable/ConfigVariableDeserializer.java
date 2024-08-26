package com.acmerobotics.dashboard.config.variable;

import com.acmerobotics.dashboard.config.ConstantProvider;
import com.google.gson.JsonDeserializationContext;
import com.google.gson.JsonDeserializer;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import java.lang.reflect.Type;
import java.util.Map;

public class ConfigVariableDeserializer implements JsonDeserializer<ConfigVariable<?>> {
    @Override
    public ConfigVariable<?> deserialize(JsonElement jsonElement, Type type,
                                         JsonDeserializationContext jsonDeserializationContext)
        throws JsonParseException {
        JsonObject obj = jsonElement.getAsJsonObject();
        VariableType varType = jsonDeserializationContext.deserialize(
            obj.get(ConfigVariable.TYPE_KEY), VariableType.class);
        JsonElement valueEl = obj.get(ConfigVariable.VALUE_KEY);
        switch (varType) {
            case BOOLEAN:
                return new BasicVariable<>(varType,
                    new ConstantProvider<>(valueEl.isJsonNull() ? null : valueEl.getAsBoolean()));
            case INT:
                return new BasicVariable<>(varType,
                    new ConstantProvider<>(valueEl.isJsonNull() ? null : valueEl.getAsInt()));
            case LONG:
                return new BasicVariable<>(varType,
                    new ConstantProvider<>(valueEl.isJsonNull() ? null : valueEl.getAsLong()));
            case FLOAT:
                return new BasicVariable<>(varType,
                    new ConstantProvider<>(valueEl.isJsonNull() ? null : valueEl.getAsFloat()));
            case DOUBLE:
                return new BasicVariable<>(varType,
                    new ConstantProvider<>(valueEl.isJsonNull() ? null : valueEl.getAsDouble()));
            case STRING:
                return new BasicVariable<>(varType,
                    new ConstantProvider<>(valueEl.isJsonNull() ? null : valueEl.getAsString()));
            case ENUM:
                if (valueEl.isJsonNull()) {
                    return new BasicVariable<>(varType, new ConstantProvider<>(null));
                }

                try {
                    Class<?> enumClass = Class.forName(
                        obj.get(ConfigVariable.ENUM_CLASS_KEY).getAsString());
                    return new BasicVariable<>(varType, new ConstantProvider<>(
                        jsonDeserializationContext.deserialize(
                            obj.get(ConfigVariable.VALUE_KEY), enumClass)));
                } catch (ClassNotFoundException e) {
                    throw new RuntimeException();
                }
            case CUSTOM:
                if (valueEl.isJsonNull()) {
                    return new CustomVariable(null);
                }

                CustomVariable customVariable = new CustomVariable();
                JsonObject valueObj = valueEl.getAsJsonObject();
                for (Map.Entry<String, JsonElement> entry : valueObj.entrySet()) {
                    JsonObject childObj = entry.getValue().getAsJsonObject();
                    VariableType childType = jsonDeserializationContext.deserialize(
                        childObj.get(ConfigVariable.TYPE_KEY), VariableType.class);
                    ConfigVariable child;
                    if (childType == VariableType.CUSTOM) {
                        child = jsonDeserializationContext.deserialize(
                            entry.getValue(), CustomVariable.class);
                    } else {
                        child = jsonDeserializationContext.deserialize(
                            entry.getValue(), BasicVariable.class);
                    }

                    customVariable.putVariable(entry.getKey(), child);
                }
                return customVariable;
            default:
                throw new RuntimeException();
        }
    }
}
