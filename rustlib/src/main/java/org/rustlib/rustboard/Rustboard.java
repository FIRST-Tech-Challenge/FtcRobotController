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

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonValue;

/**
 * All public static methods in this class will be executed on the currently <b>active</b> rustboard.  These method calls will refer back to the RustboardServer singleton to access the currently active rustboard instance and then execute the non-static methods of that instance.  This means that you can only update nodes on the rustboard.
 */
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

    Rustboard(String uuid, JsonObject json) {
        this.uuid = uuid;
        JsonArray nodes = json.getJsonArray("nodes");
        nodes.forEach((JsonValue nodeJson) -> this.nodes.add(RustboardNode.buildFromJson(nodeJson)));
    }

    Rustboard(String uuid, Set<RustboardNode> nodes) {
        this.uuid = uuid;
        this.nodes.addAll(nodes);
    }

    static Rustboard emptyRustboard() {
        return new Rustboard(null, new HashSet<>());
    }

    Rustboard(Builder builder) {
        this.uuid = builder.uuid;
    }

    static Rustboard getActiveRustboard() {
        return RustboardServer.getInstance().getActiveRustboard();
    }

    static Rustboard requireActiveRustboard() {
        Rustboard activeRustboard = RustboardServer.getInstance().getActiveRustboard();
        if (activeRustboard == null) {
            throw new NullPointerException("No active rustboard available");
        }
        return activeRustboard;
    }

    WebSocket getConnection() {
        return connection;
    }

    void setConnection(WebSocket connection) {
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
                if (node.equals(clientNode) && !node.strictEquals(clientNode)) {
                    RustboardNode updatedNode = node.merge(clientNode);
                    updatedNodeList.add(updatedNode);
                    RustboardServer.getInstance().getClientUpdater().updateNode(updatedNode);
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
                    notifyClient("Saved path to robot", NoticeType.POSITIVE, 8000);
                } catch (IOException | NullPointerException e) {
                    notifyClient("Could not save the path to the robot", NoticeType.NEGATIVE, 8000);
                    RustboardServer.log(e.toString());
                }
                break;
            case "save_value":
                try {
                    Loader.writeString(rustboardStorageDir, Objects.requireNonNull(messageJson.get("value")).toString());
                    notifyClient("Saved value to robot", NoticeType.POSITIVE, 8000);
                } catch (IOException | NullPointerException e) {
                    notifyClient("Could not save the value to the robot", NoticeType.NEGATIVE, 8000);
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

    private static JsonObject createNoticeJson(String notice, NoticeType type, int durationMilliseconds) {
        return Json.createObjectBuilder()
                .add("action", "create_notice")
                .add("notice_message", notice)
                .add("notice_type", type.value)
                .add("notice_duration", durationMilliseconds)
                .build();
    }

    public void notifyClient(String notice, NoticeType type, int durationMilliseconds) {
        connection.send(createNoticeJson(notice, type, durationMilliseconds).toString());
    }

    public static void notifyActiveClient(String notice, NoticeType type, int durationMilliseconds) {
        if (RustboardServer.isActiveRustboard()) {
            getActiveRustboard().notifyClient(notice, type, durationMilliseconds);
        }
    }

    public static void notifyAllClients(String notice, NoticeType type, int durationMilliseconds) {
        for (WebSocket connection : RustboardServer.getInstance().getConnections()) {
            connection.send(createNoticeJson(notice, type, durationMilliseconds).toString());
        }
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

    private static void updateNode(String id, Type type, Object value) {
        Rustboard activeRustboard = RustboardServer.getInstance().getActiveRustboard();
        try {
            activeRustboard.getNode(id, type).update(value);
        } catch (NoSuchNodeException e) {
            activeRustboard.nodes.add(new RustboardNode(id, type, value.toString()));
        }
    }

    public static void updatePositionGraphNode(String id, Pose2d position) {
        updateNode(id, Type.POSITION_GRAPH, position);
    }

    public static void updateSelectorNode(String id, String option) {
        updateNode(id, Type.SELECTOR, option);
    }

    public static void updateTelemetryNode(String id, Object value) {
        updateNode(id, Type.TEXT_TELEMETRY, value);
    }

    public static void updateInputNode(String id, Object value) {
        updateNode(id, Type.TEXT_INPUT, value);
    }

    public static void updateBooleanTelemetryNode(String id, boolean value) {
        updateNode(id, Type.BOOLEAN_TELEMETRY, value);
    }

    public static void updateToggleNode(String id, boolean value) {
        updateNode(id, Type.TOGGLE, value);
    }

    public static void updatePositionGraph(String id, Pose2d position) {
        updateNode(id, Type.POSITION_GRAPH, position);
    }

    public static String getString(String id, String defaultValue) {
        try {
            return requireActiveRustboard().getNode(id, Type.TEXT_INPUT).getState();
        } catch (NoSuchNodeException | NullPointerException e) {
            return defaultValue;
        }
    }

    public static String getString(String id) throws NoSuchNodeException, NullPointerException {
        return getActiveRustboard().getNode(id, Type.TEXT_INPUT).getState();
    }

    public static String safeGetString(String id) {
        return getString(id, "");
    }

    public static double getDouble(String id, double defaultValue) {
        try {
            return Double.parseDouble(getString(id));
        } catch (NumberFormatException | NoSuchNodeException | NullPointerException e) {
            return defaultValue;
        }
    }

    public static double getDouble(String id) throws NumberFormatException, NoSuchNodeException, NullPointerException {
        return Double.parseDouble(getString(id));
    }

    public static double safeGetDouble(String id) {
        return getDouble(id, 0);
    }

    public static boolean getToggleValue(String id, boolean defaultValue) {
        try {
            return Boolean.parseBoolean(getActiveRustboard().getNode(id, Type.TOGGLE).getState());
        } catch (NoSuchNodeException e) {
            return defaultValue;
        }
    }

    public static boolean getToggleValue(String id) throws NoSuchNodeException {
        return Boolean.parseBoolean(getActiveRustboard().getNode(id, Type.TOGGLE).getState());
    }

    public static boolean safeGetToggleValue(String id) {
        return getToggleValue(id, false);
    }

    public static String getSelectedOption(String id, String defaultValue) {
        try {
            return getActiveRustboard().getNode(id, Type.SELECTOR).getState();
        } catch (NoSuchNodeException e) {
            return defaultValue;
        }
    }

    public static String getSelectedOption(String id) throws NoSuchNodeException {
        return getActiveRustboard().getNode(id, Type.SELECTOR).getState();
    }

    public String safeGetSelectedOption(String id) {
        return getSelectedOption(id, null);
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
