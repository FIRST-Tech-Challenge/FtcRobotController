package org.rustlib.rustboard;

import com.google.gson.JsonParseException;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.java_websocket.WebSocket;
import org.rustlib.config.Loader;

import java.io.File;
import java.io.IOException;
import java.io.StringReader;
import java.util.HashMap;
import java.util.Objects;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObject;
import javax.json.JsonValue;

public class RustboardLayout {
    protected static final String layoutFilePrefix = "dashboard_layout_";
    protected final WebSocket connection;
    private final HashMap<String, Runnable> callbacks = new HashMap<>();
    protected HashMap<String, RustboardNode> nodes = new HashMap<>();
    protected String id = "";
    private long connectionOpenedTimestamp = -1;
    private JsonObject jsonDescriptor;

    public RustboardLayout(WebSocket connection) {
        this.connection = connection;
    }

    private static RustboardNode.Type getNodeType(String inputType) {
        RustboardNode.Type type = null;
        RustboardNode.Type[] types = RustboardNode.Type.values();
        for (RustboardNode.Type value : types) {
            if (Objects.equals(value.name, inputType)) {
                type = value;
            }
        }
        return type;
    }

    protected static JsonObject getSendableNodeData(String id, String value) {
        return Json.createObjectBuilder()
                .add("messageType", "node update")
                .add("nodeID", id)
                .add("state", value)
                .build();
    }

    public static boolean isDashboardLayoutFile(String fileName) {
        return fileName.contains(layoutFilePrefix);
    }

    public static RustboardLayout loadLayout(String fileName) {
        String layoutData = Loader.loadString(Loader.defaultStorageDirectory.getPath(), fileName, "json");
        RustboardLayout layout = new RustboardLayout(null);
        try {
            layout.update(Json.createReader(new StringReader(layoutData)).readObject());
        } catch (JsonParseException e) {
            Rustboard.log(e.toString());
            e.printStackTrace();
        }
        return layout;
    }

    public static double loadDouble(String fileName, double defaultValue) {
        try {
            return Double.parseDouble(Loader.loadString(Loader.defaultStorageDirectory.getPath(), fileName));
        } catch (NumberFormatException e) {
            Rustboard.log(e.toString());
            return defaultValue;
        }
    }

    public static boolean loadBoolean(String fileName) {
        return Boolean.parseBoolean(Loader.loadString(Loader.defaultStorageDirectory.getPath(), fileName));
    }

    public long getConnectionOpenedTimestamp() {
        return connectionOpenedTimestamp;
    }

    public void setConnectionOpenedTimestamp(long timestamp) {
        if (connectionOpenedTimestamp != -1) {
            connectionOpenedTimestamp = timestamp;
        }
    }

    /**
     * Sets the value of a node linked to this dashboard.
     *
     * @param id    The id of the target dashboard node.
     * @param value The value to send to the target dashboard node.
     */
    public void setNodeValue(String id, String value) {
        Objects.requireNonNull(nodes.get(id)).state = value;
        if (SubsystemConstants.debugMode) {
            JsonObject jsonObject = getSendableNodeData(id, value);
            Rustboard.getInstance().sendToConnection(this, jsonObject.toString());
        }
    }

    /**
     * Sets the value of a node linked to this dashboard.
     *
     * @param id    The id of the target dashboard node.
     * @param value The value to send to the target dashboard node.
     */
    public void setNodeValue(String id, Object value) {
        setNodeValue(id, String.valueOf(value));
    }

    public void update(JsonObject object) {
        HashMap<String, RustboardNode> nodes = new HashMap<>();
        JsonArray jsonValues = object.getJsonArray("layout");
        id = object.getString("id");
        for (JsonValue jsonValue : jsonValues) {
            JsonObject nodeJson = jsonValue.asJsonObject();
            RustboardNode node = new RustboardNode(nodeJson.getString("id"), getNodeType(nodeJson.getString("type")), nodeJson.getString("state"), Rustboard.getInstance().getUTCTime());
            nodes.put(node.id, node);
        }
        this.nodes = nodes;
    }

    public void updateNode(JsonObject object) {
        JsonObject configuration = object.getJsonObject("configuration");
        String nodeID = configuration.getString("id");
        nodes.get(nodeID).state = Objects.requireNonNull(configuration.getString("state"));
    }

    private String getNodeState(String id, RustboardNode.Type... requiredTypes) {
        RustboardNode node = nodes.get(id);
        if (node != null) {
            boolean correctType = false;
            for (RustboardNode.Type requiredType : requiredTypes) {
                if (node.type == requiredType) {
                    correctType = true;
                    break;
                }
            }
            if (!correctType) {
                StringBuilder errMsg = new StringBuilder("The type of the requested node (\"" + node.type.name + "\") does not match any of the required types:\n Required types: ");
                for (int i = 0; i < requiredTypes.length; i++) {
                    errMsg.append("\"" + requiredTypes[i].name + "\"");
                    if (i < requiredTypes.length - 1) {
                        errMsg.append(", ");
                    }
                }
                throw new IllegalArgumentException(errMsg.toString());
            }
            return node.state;
        }
        return "";
    }

    public double getDoubleValue(String id, double defaultValue) {
        try {
            return Double.parseDouble(getNodeState(id, RustboardNode.Type.TEXT_INPUT, RustboardNode.Type.TEXT_TELEMETRY));
        } catch (NumberFormatException e) {
            Rustboard.log(e + "\n + Couldn't get double value for id " + id);
            return defaultValue;
        }
    }

    public boolean getBooleanValue(String id) {
        return Boolean.parseBoolean(getNodeState(id, RustboardNode.Type.BOOLEAN_TELEMETRY, RustboardNode.Type.TOGGLE));
    }

    public String getInputValue(String id) {
        return getNodeState(id, RustboardNode.Type.TEXT_INPUT);
    }

    public String getSelectedValue(String id) {
        return getNodeState(id, RustboardNode.Type.SELECTOR);
    }

    public void buttonClicked(String id) {
        try {
            Objects.requireNonNull(callbacks.get(id)).run();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
    }

    public void addCallback(String buttonName, Runnable callback) {
        callbacks.put(buttonName, callback);
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

    private JsonArray getLayoutJSON() {
        JsonArrayBuilder draggableDataBuilder = Json.createArrayBuilder();
        nodes.forEach((id, node) -> draggableDataBuilder.add(Json.createObjectBuilder()
                .add("id", node.id)
                .add("type", node.type.name)
                .add("state", node.state)
                .build()));
        return draggableDataBuilder.build();
    }

    public void save() throws IOException {
        JsonObject data = Json.createObjectBuilder()
                .add("messageType", "layout state")
                .add("layout", getLayoutJSON())
                .add("id", id)
                .build();
        String fileName = layoutFilePrefix + id.replace(" ", "_") + "json";
        File output = new File(Loader.defaultStorageDirectory, fileName);
        Loader.writeString(output, data.toString());
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

    public static class RustboardNode {
        private final String id;
        private final Type type;
        private String state;
        private long lastUpdated;

        public RustboardNode(String id, Type type, String state, long lastUpdated) {
            this.id = id;
            this.type = type;
            this.state = state;
            this.lastUpdated = lastUpdated;
        }

        public enum Type {
            BUTTON("button"),
            TOGGLE("toggle"),
            SELECTOR("selector"),
            BOOLEAN_TELEMETRY("boolean telemetry"),
            TEXT_TELEMETRY("text telemetry"),
            TEXT_INPUT("text input"),
            POSITION_GRAPH("position_graph"),
            PATH("path"),
            CAMERA_STREAM("camera steam");

            private final String name;

            Type(String name) {
                this.name = name;
            }
        }
    }
}
