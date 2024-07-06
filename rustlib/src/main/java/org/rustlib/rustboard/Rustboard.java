package org.rustlib.rustboard;

import org.java_websocket.WebSocket;
import org.rustlib.commandsystem.Command;
import org.rustlib.config.Loader;
import org.rustlib.geometry.Pose2d;
import org.rustlib.rustboard.RustboardNode.NodeNotFoundException;
import org.rustlib.rustboard.RustboardNode.Type;

import java.io.IOException;
import java.io.StringReader;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import javax.json.Json;
import javax.json.JsonObject;
import javax.json.JsonReader;

public class Rustboard {
    private static int nextAvailableClientId = 0;
    private WebSocket connection = null;
    private int clientId = -1;
    private Set<RustboardNode> nodes = new HashSet<>();
    private boolean connected = false;
    private Map<String, Runnable> callbacks = new HashMap();

    Rustboard(JsonObject descriptor) {

    }

    WebSocket getConnection() {
        return connection;
    }

    void onConnect(WebSocket connection) {
        connected = true;
        this.connection = connection;
        negotiateIds();
    }

    void onMessage(String data) {
        JsonReader reader = Json.createReader(new StringReader(data));
        JsonObject object = reader.readObject();
        JsonObject message = object.getJsonObject("message");
        String messageType = message.getString("messageType");
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
                    Loader.savePath(message);
                    createNotice("Saved path to robot", NoticeType.POSITIVE, 8000);
                } catch (IOException e) {
                    createNotice("Could not save the path to the robot", NoticeType.NEGATIVE, 8000);
                    RustboardServer.log(e.toString());
                }
                break;
            case "value update":
                try {
                    Loader.saveValue(message);
                    createNotice("Saved value to robot", NoticeType.POSITIVE, 8000);
                } catch (IOException e) {
                    createNotice("Could not save the value to the robot", NoticeType.NEGATIVE, 8000);
                    RustboardServer.log(e.toString());
                }
                break;
            case "click":
                String nodeId = message.getString("nodeID");
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

    public void createNotice(String notice, NoticeType type, int durationMilliseconds) {
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

    public void updatePositionGraph(String id, Pose2d position) {
        getNode(id, Type.POSITION_GRAPH).updateAndSend((position));
    }

    public void updateSelectorValue(String id, String value) {
        getNode(id, Type.SELECTOR).updateAndSend((value));
    }

    public void updateTelemetryNode(String id, Object value) {
        getNode(id, Type.TEXT_TELEMETRY).updateAndSend((value));
    }

    public void updateInputNode(String id, Object value) {
        getNode(id, Type.TEXT_INPUT).updateAndSend((value));
    }

    public void updateBooleanTelemetryNode(String id, boolean value) {
        getNode(id, Type.BOOLEAN_TELEMETRY).updateAndSend((value));
    }

    public void updateToggleNode(String id, boolean value) {
        getNode(id, Type.TOGGLE).updateAndSend(value);
    }

    private void negotiateIds() { // TODO: add code for negotiating client and server ids

    }

    public enum NoticeType {
        POSITIVE("positive"),
        NEGATIVE("negative"),
        NEUTRAL("neutral");

        String value;

        NoticeType(String value) {
            this.value = value;
        }
    }
}
