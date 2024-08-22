package org.rustlib.rustboard;

import static org.rustlib.rustboard.JsonKeys.LAST_NODE_UPDATE_KEY;
import static org.rustlib.rustboard.JsonKeys.NODE_ID_KEY;
import static org.rustlib.rustboard.JsonKeys.NODE_STATE_KEY;
import static org.rustlib.rustboard.JsonKeys.NODE_TYPE_KEY;

import org.rustlib.rustboard.Rustboard.RustboardException;

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
        lastUpdate = Time.now();
        updateClient();
    }

    void remoteUpdateState(Object state, long time) {
        this.state = state.toString();
        lastUpdate = new Time(time, true);
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
            super("These are not the droids you're looking for!  Could not find a node with id '" + nodeId + "'");
        }
    }

    JsonObjectBuilder getJsonBuilder() {
        JsonObjectBuilder builder = Json.createObjectBuilder();
        builder.add(NODE_ID_KEY, id);
        builder.add(NODE_TYPE_KEY, type.typeName);
        builder.add(NODE_STATE_KEY, state);
        builder.add(LAST_NODE_UPDATE_KEY, lastUpdate.getTimeMS());
        return builder;
    }

    static RustboardNode buildFromJson(JsonValue json) {
        try {
            JsonObject data = json.asJsonObject();
            return new RustboardNode(
                    data.getString(NODE_ID_KEY), Type.getType(data.getString(NODE_TYPE_KEY)),
                    data.getString(NODE_STATE_KEY),
                    data.containsKey(LAST_NODE_UPDATE_KEY) ? new Time(data.getJsonNumber(LAST_NODE_UPDATE_KEY).longValue(), true) : new Time()
            );
        } catch (RuntimeException e) {
            throw new InvalidNodeJsonException("The following JSON does not conform to the JSON protocol for Rustboard Nodes: \n" + json.toString());
        }
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
        return String.format("id: '%s'\ntype: '%s'\nstate: '%s'\nlast update: %s", id, type.typeName, state, lastUpdate.getTimeMS());
    }

    static class InvalidNodeJsonException extends RustboardException {

        public InvalidNodeJsonException(String message) {
            super(message);
        }
    }
}
