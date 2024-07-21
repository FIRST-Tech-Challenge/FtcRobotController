package org.rustlib.rustboard;

import org.java_websocket.WebSocket;
import org.rustlib.rustboard.RustboardServer.Timestamp;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Objects;

import javax.json.Json;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonValue;

public class RustboardNode {
    private WebSocket connection;
    private String parentLayoutId;
    public final Type type;
    public final String id;
    private String state;
    private boolean clientAwaitingUpdate = false;
    private Timestamp lastLocalUpdate;
    private Timestamp lastClientUpdate;

    public RustboardNode(String id, Type type, String state) {
        this.id = id;
        this.type = type;
        this.state = state;
    }

    public boolean fromLayout(String layoutId) {
        return Objects.equals(parentLayoutId, layoutId);
    }

    private boolean canUpdate() {
        switch (type.overrideAbility) {
            case CHECK_TIME:
                if (Timestamp.isTimeCalibrated() && lastLocalUpdate.getTimeMillis() > lastClientUpdate.getTimeMillis()) {
                    return true;
                }
                break;
            case ALWAYS:
                return true;
        }
        return false;
    }

    void update(Object state) {
        this.state = state.toString();
    }

    public boolean isClientAwaitingUpdate() {
        return lastLocalUpdate.getTimeMillis() > lastClientUpdate.getTimeMillis();
    }

    JsonObject getSendableData() {
        return Json.createObjectBuilder()
                .add("messageType", "node update")
                .add("nodeID", id)
                .add("state", state)
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
        POSITION_GRAPH("position_graph", OverrideAbility.ALWAYS),
        PATH("path", OverrideAbility.CHECK_TIME),
        CAMERA_STREAM("camera steam", OverrideAbility.NEVER);

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

    public static class NodeNotFoundException extends RuntimeException {
        public NodeNotFoundException(String nodeId) {
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
        return new RustboardNode(data.getString("id"), Type.getType(data.getString("type")), data.getString("state"));
    }
}
