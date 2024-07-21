package org.rustlib.rustboard;

import org.java_websocket.WebSocket;
import org.rustlib.commandsystem.Command;
import org.rustlib.config.Loader;
import org.rustlib.geometry.Pose2d;
import org.rustlib.rustboard.RustboardNode.NodeNotFoundException;
import org.rustlib.rustboard.RustboardNode.Type;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
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
    private Set<RustboardNode> nodes = new HashSet<>();
    private boolean connected = false;
    private Map<String, Runnable> callbacks = new HashMap();
    Map<String, RustboardNode> toUpdate = new ConcurrentHashMap();


    Map<String, RustboardNode> getNodeList() {
        return toUpdate;
    }

    Rustboard(String uuid, JsonObject json) {
        this.uuid = uuid;
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

    Rustboard mergeWithClientRustboard(JsonObject clientRustboardJson) {
        Builder builder = getBuilder().setUUID(uuid);
        for () {

        }
        return null;
    }

    void onMessage(JsonObject messageJson) {
        String messageType = messageJson.getString("messageType");
        switch (messageType) {
            case "connection info":
                // TODO
                break;
            case "switch layout":
                break;
            case "layout state":
                // TODO
                break;
            case "node update":
                // TODO
                break;
            case "path update":
                try {
                    Loader.savePath(messageJson);
                    createNotice("Saved path to robot", RustboardNotice.NoticeType.POSITIVE, 8000);
                } catch (IOException e) {
                    createNotice("Could not save the path to the robot", RustboardNotice.NoticeType.NEGATIVE, 8000);
                    RustboardServer.log(e.toString());
                }
                break;
            case "value update":
                try {
                    Loader.saveText(messageJson.toString());
                    createNotice("Saved value to robot", RustboardNotice.NoticeType.POSITIVE, 8000);
                } catch (IOException e) {
                    createNotice("Could not save the value to the robot", RustboardNotice.NoticeType.NEGATIVE, 8000);
                    RustboardServer.log(e.toString());
                }
                break;
            case "click":
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
        JsonObject data = Json.createObjectBuilder()
                .add("messageType", "notify")
                .add("message", notice)
                .add("type", type.value)
                .add("duration", durationMilliseconds)
                .build();
        connection.send(data.toString());
    }

    private RustboardNode getNode(String id, Type type) {
        for (RustboardNode node : nodes) {
            if (node.type == type) {
                return node;
            }
        }
        throw new RustboardNode.NodeNotFoundException(id);
    }

    private boolean nodeExists(String id, Type type) {
        try {
            getNode(id, type);
            return true;
        } catch (NodeNotFoundException e) {
            return false;
        }
    }

    public void addCallback(String buttonId, Runnable callback) {
        if (nodeExists(buttonId, Type.BUTTON)) {
            callbacks.put(buttonId, callback);
        } else {
            throw new NodeNotFoundException(buttonId);
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
                throw new NodeNotFoundException(buttonId);
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
