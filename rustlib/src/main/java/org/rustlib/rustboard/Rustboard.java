package org.rustlib.rustboard;

import static org.rustlib.rustboard.RustboardServer.rustboardStorageDir;

import org.java_websocket.WebSocket;
import org.rustlib.commandsystem.Command;
import org.rustlib.config.Loader;
import org.rustlib.geometry.Pose2d;
import org.rustlib.rustboard.RustboardNode.NoSuchNodeException;
import org.rustlib.rustboard.RustboardNode.Type;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonValue;

public class Rustboard {

    interface SetUUID {
        Builder setUUID(String uuid);
    }

    static class Builder implements SetUUID {
        private String uuid;
        private final List<RustboardNode> nodes = new ArrayList<>();

        private Builder() {
        }


        @Override
        public Builder setUUID(String uuid) {
            this.uuid = uuid;
            return this;
        }

        public Builder addNode(RustboardNode node) {
            nodes.add(node);
            return this;
        }

        public Rustboard build() {
            return new Rustboard(this);
        }
    }

    static SetUUID getBuilder() {
        return new Builder();
    }

    private WebSocket connection = null;
    private final String uuid;
    private final Set<RustboardNode> nodes = new HashSet<>();
    private boolean connected = false;
    private final Map<String, Runnable> callbacks = new HashMap<>();
    Map<String, RustboardNode> toUpdate = new ConcurrentHashMap<>();


    Map<String, RustboardNode> getNodeList() {
        return toUpdate;
    }

    Rustboard(String uuid, JsonObject json) {
        this.uuid = uuid;
    }

    public Rustboard(String uuid, Set<RustboardNode> nodes) {
        this.uuid = uuid;
        this.nodes.addAll(nodes);
    }

    Rustboard(Builder builder) {
        this.uuid = builder.uuid;
    }

    public static Rustboard getActiveRustboard() {
        return RustboardServer.getInstance().getActiveRustboard();
    }

    WebSocket getConnection() {
        return connection;
    }

    void onConnect(WebSocket connection) {
        connected = true;
        this.connection = connection;
    }

    static Rustboard load(String uuid) throws NoSuchRustboardException {
        try {
            Builder rustboardBuilder = getBuilder().setUUID(uuid);
            JsonObject json = Loader.loadJsonObject(uuid + ".json");
            JsonArray nodes = json.getJsonArray("nodes");
            nodes.forEach((JsonValue nodeJson) -> rustboardBuilder.addNode(RustboardNode.buildFromJson(nodeJson)));
            return rustboardBuilder.build();
        } catch (RuntimeException e) {
            throw new NoSuchRustboardException("");
        }
    }

    Rustboard mergeWithClientRustboard(JsonArray clientNodes) {
        Set<RustboardNode> updatedNodeList = new HashSet<>(nodes);
        for (JsonValue nodeJson : clientNodes) {
            RustboardNode clientNode = RustboardNode.buildFromJson(nodeJson);
            for (RustboardNode node : nodes) {
                if (node.equals(clientNode)) {
                    updatedNodeList.add(node.merge(clientNode));
                    // TODO: update client
                    updatedNodeList.remove(node);
                }
            }
        }
        return new Rustboard(uuid, updatedNodeList);
    }

    void onMessage(JsonObject messageJson) {
        switch (messageJson.getString("action")) {
            case "update_node":
                String id = messageJson.getString("node_id");
                Type type = Type.getType(messageJson.getString("node_type"));
                String state = messageJson.getString("node_state");
                try {
                    getNode(id, type).update(state);
                } catch (NoSuchNodeException e) {
                    this.nodes.add(new RustboardNode(id, type, state));
                }
                break;
            case "save_path":
                try {
                    Loader.writeString(rustboardStorageDir, Objects.requireNonNull(messageJson.get("path")).toString());
                    createNotice("Saved path to robot", RustboardNotice.NoticeType.POSITIVE, 8000);
                } catch (IOException | NullPointerException e) {
                    createNotice("Could not save the path to the robot", RustboardNotice.NoticeType.NEGATIVE, 8000);
                    RustboardServer.log(e.toString());
                }
                break;
            case "save_value":
                try {
                    Loader.writeString(rustboardStorageDir, Objects.requireNonNull(messageJson.get("value")).toString()); // TODO: fix
                    createNotice("Saved value to robot", RustboardNotice.NoticeType.POSITIVE, 8000);
                } catch (IOException | NullPointerException e) {
                    createNotice("Could not save the value to the robot", RustboardNotice.NoticeType.NEGATIVE, 8000);
                    RustboardServer.log(e.toString());
                }
                break;
            case "click_button":
                String nodeId = messageJson.getString("nodeID");
                if (callbacks.containsKey(nodeId)) {
                    clickButton(nodeId);
                }
                break;
        }
    }

    void onDisconnect() {
        connected = false;
    }

    public boolean isConnected() {
        return connected;
    }

    public void createNotice(String notice, RustboardNotice.NoticeType type, int durationMilliseconds) {
        JsonObject message = Json.createObjectBuilder()
                .add("action", "create_notice")
                .add("notice_message", notice)
                .add("notice_type", type.value)
                .add("notice_duration", durationMilliseconds)
                .build();
        connection.send(message.toString());
    }

    private RustboardNode getNode(String id, Type type) {
        for (RustboardNode node : nodes) {
            if (node.id.equals(id) && node.type == type) {
                return node;
            }
        }
        throw new NoSuchNodeException(id);
    }

    private boolean nodeExists(String id, Type type) {
        try {
            getNode(id, type);
            return true;
        } catch (NoSuchNodeException e) {
            return false;
        }
    }

    public void addCallback(String buttonId, Runnable callback) {
        if (nodeExists(buttonId, Type.BUTTON)) {
            callbacks.put(buttonId, callback);
        } else {
            throw new NoSuchNodeException(buttonId);
        }
    }

    public void addCallback(String buttonId, Command command) {
        callbacks.put(buttonId, command::schedule);
    }

    public void clickButton(String buttonId) {
        try {
            callbacks.get(buttonId).run();
        } catch (NullPointerException e) {
            if (nodeExists(buttonId, Type.BUTTON)) {
                throw new RuntimeException("No runnable was mapped to the button with id '" + buttonId + "'");
            } else {
                throw new NoSuchNodeException(buttonId);
            }
        }
    }

    public static void updatePositionGraphNode(String id, Pose2d position) {
        RustboardServer.getInstance().getActiveRustboard().getNode(id, Type.POSITION_GRAPH).update((position));
    }

    public static void updateSelectorValueNode(String id, String value) {
        RustboardServer.getInstance().getActiveRustboard().getNode(id, Type.SELECTOR).update((value));
    }

    public static void updateTelemetryNode(String id, Object value) {
        RustboardServer.getInstance().getActiveRustboard().getNode(id, Type.TEXT_TELEMETRY).update((value));
    }

    public static void updateInputNode(String id, Object value) {
        RustboardServer.getInstance().getActiveRustboard().getNode(id, Type.TEXT_INPUT).update((value));
    }

    public static void updateBooleanTelemetryNode(String id, boolean value) {
        RustboardServer.getInstance().getActiveRustboard().getNode(id, Type.BOOLEAN_TELEMETRY).update((value));
    }

    public static void updateToggleNode(String id, boolean value) {
        RustboardServer.getInstance().getActiveRustboard().getNode(id, Type.TOGGLE).update(value);
    }

    public void updatePositionGraph(String id, Pose2d position) {
        getNode(id, Type.POSITION_GRAPH).update((position));
    }

    public void updateSelectorValue(String id, String value) {
        getNode(id, Type.SELECTOR).update((value));
    }

    public void updateTelemetry(String id, Object value) {
        getNode(id, Type.TEXT_TELEMETRY).update((value));
    }

    public void updateInput(String id, Object value) {
        getNode(id, Type.TEXT_INPUT).update((value));
    }

    public void updateBooleanTelemetry(String id, boolean value) {
        getNode(id, Type.BOOLEAN_TELEMETRY).update((value));
    }

    public void updateToggle(String id, boolean value) {
        getNode(id, Type.TOGGLE).update(value);
    }

    public String getInputValue(String id, String defaultValue) {
        try {
            return getNode(id, Type.TEXT_INPUT).getState();
        } catch (NoSuchNodeException e) {
            return defaultValue;
        }
    }

    public String getInputValue(String id) throws NoSuchNodeException {
        return getNode(id, Type.TEXT_INPUT).getState();
    }

    public String safeGetInputValue(String id) {
        return getInputValue(id, "");
    }

    public boolean getToggleValue(String id, boolean defaultValue) {
        try {
            return Boolean.parseBoolean(getNode(id, Type.TOGGLE).getState());
        } catch (NoSuchNodeException e) {
            return defaultValue;
        }
    }

    public boolean getToggleValue(String id) throws NoSuchNodeException {
        return Boolean.parseBoolean(getNode(id, Type.TOGGLE).getState());
    }

    public boolean safeGetToggleValue(String id) {
        return getToggleValue(id, false);
    }

    public String getSelectorValue(String id, String defaultValue) {
        try {
            return getNode(id, Type.SELECTOR).getState();
        } catch (NoSuchNodeException e) {
            return defaultValue;
        }
    }

    public String getSelectorValue(String id) throws NoSuchNodeException {
        return getNode(id, Type.SELECTOR).getState();
    }

    public String safeGetSelectorValue(String id) {
        return getSelectorValue(id, null);
    }

    JsonObject getJson() {
        JsonObjectBuilder jsonBuilder = Json.createObjectBuilder();
        jsonBuilder.add("uuid", uuid);
        JsonArrayBuilder nodeArray = Json.createArrayBuilder();
        nodes.forEach((RustboardNode node) -> nodeArray.add(node.getJsonBuilder()));
        jsonBuilder.add("nodes", nodeArray);
        return jsonBuilder.build();
    }

    void save(File file) {
        try {
            Loader.writeJson(file, getJson());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    static class NoSuchRustboardException extends Exception {
        NoSuchRustboardException(String uuid) {
            super(String.format("The rustboard with the id '%s' has no corresponding file", uuid));
        }
    }
}
