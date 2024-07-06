package org.rustlib.rustboard;

import org.java_websocket.WebSocket;
import org.rustlib.rustboard.RustboardServer.Timestamp;

import java.util.Objects;

import javax.json.Json;
import javax.json.JsonObject;

public class RustboardNode {
    private WebSocket connection;
    private String parentLayoutId;
    public final Type type;
    public final String id;
    private String state;
    private boolean clientAwaitingUpdate = false;
    private Timestamp lastLocalUpdate;
    private Timestamp lastClientUpdate;

    public RustboardNode(WebSocket connection, String parentLayoutId, Type type, String id, String state) {
        this.connection = connection;
        this.parentLayoutId = parentLayoutId;
        this.type = type;
        this.id = id;
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

    void updateLocal(Object state) {
        if (canUpdate()) {
            this.state = state.toString();
            lastLocalUpdate = new Timestamp();
        }
    }

    void updateAndSend(Object state) {
        updateLocal(state);
        requestClientModification();
    }

    void requestClientModification() {
        if (canUpdate()) {
            if (connection != null && connection.isOpen()) {
                connection.send(getSendableData().toString());
                lastClientUpdate = new Timestamp();
            }
        }
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

        public final String name;
        private final OverrideAbility overrideAbility;

        Type(String name, OverrideAbility overrideAbility) {
            this.name = name;
            this.overrideAbility = overrideAbility;
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
}
