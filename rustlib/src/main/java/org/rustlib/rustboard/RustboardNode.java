package org.rustlib.rustboard;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Objects;

import javax.json.Json;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonValue;

public class RustboardNode {
    public final Type type;
    public final String id;
    private String state;
    private Time lastUpdate;

    public RustboardNode(String id, Type type, String state, Time lastUpdate) {
        this.id = id;
        this.type = type;
        this.state = state;
        this.lastUpdate = lastUpdate;
    }

    public RustboardNode(String id, Type type, String state) {
        this(id, type, state, new Time());
    }

    void localUpdateState(Object state) {
        this.state = state.toString();
        lastUpdate = new Time();
        updateClient();
    }

    void remoteUpdateState(Object state, long time) {
        this.state = state.toString();
        lastUpdate = new Time(time, true);
    }

    JsonObject getSendableData() {
        return Json.createObjectBuilder()
                .add("messageType", "node update")
                .add("nodeID", id)
                .add("state", state)
                .add("last_update", lastUpdate.getTimeMS())
                .build();
    }

    String getState() {
        return state;
    }

    public enum Type {
        BUTTON("button", OverrideAbility.NEVER),
        TOGGLE("toggle", OverrideAbility.CHECK_TIME),
        SELECTOR("selector", OverrideAbility.CHECK_TIME),
        BOOLEAN_TELEMETRY("boolean telemetry", OverrideAbility.ALWAYS),
        TEXT_TELEMETRY("text telemetry", OverrideAbility.ALWAYS),
        TEXT_INPUT("text input", OverrideAbility.CHECK_TIME),
        POSITION_GRAPH("position graph", OverrideAbility.ALWAYS),
        PATH("path", OverrideAbility.CHECK_TIME),
        CAMERA_STREAM("camera stream", OverrideAbility.NEVER);

        public final String typeName;
        private final OverrideAbility overrideAbility;
        private static final HashMap<String, Type> these = new HashMap<>();

        static {
            EnumSet.allOf(Type.class).forEach((Type type) -> these.put(type.typeName, type));
        }

        Type(String typeName, OverrideAbility overrideAbility) {
            this.typeName = typeName;
            this.overrideAbility = overrideAbility;
        }

        public static Type getType(String typeName) {
            Type type = these.get(typeName);
            if (type == null) {
                throw new IllegalArgumentException(String.format("No such node type '%s'", typeName));
            }
            return type;
        }
    }

    enum OverrideAbility {
        NEVER,
        CHECK_TIME,
        ALWAYS
    }

    public static class NoSuchNodeException extends RuntimeException {
        public NoSuchNodeException(String nodeId) {
            super("These are not the droids you're looking for!\nCould not find a node with id '" + nodeId + "'");
        }
    }

    JsonObjectBuilder getJsonBuilder() {
        JsonObjectBuilder builder = Json.createObjectBuilder();
        builder.add("id", id);
        builder.add("type", type.typeName);
        builder.add("state", state);
        return builder;
    }

    static RustboardNode buildFromJson(JsonValue json) {
        JsonObject data = (JsonObject) json;
        return new RustboardNode(data.getString("id"), Type.getType(data.getString("type")), data.getString("state"), new Time(data.getJsonNumber("last_update").longValue(), true));
    }

    RustboardNode merge(RustboardNode toCompare) {
        switch (type.overrideAbility) {
            case ALWAYS:
                updateClient();
                return this;
            case CHECK_TIME:
                if (toCompare.lastUpdate.getTimeMS() > lastUpdate.getTimeMS()) {
                    return toCompare;
                } else {
                    updateClient();
                    return this;
                }
            case NEVER:
                return toCompare;
        }
        return toCompare;
    }

    public boolean strictEquals(Object o) {
        if (o instanceof RustboardNode) {
            RustboardNode node = (RustboardNode) o;
            return id.equals(node.id) && type == node.type && state.equals(node.state);
        }
        return false;
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof RustboardNode) {
            RustboardNode node = (RustboardNode) o;
            return id.equals(node.id) && type == node.type;
        }
        return false;
    }

    public void updateClient() {
        RustboardServer.getInstance().getClientUpdater().updateNode(this);
    }

    @Override
    public int hashCode() {
        return Objects.hash(id, type.typeName);
    }

    @Override
    public String toString() {
        return String.format("id: '%s'\ntype: '%s'\nstate: '%s'", id, type.typeName, state);
    }
}
