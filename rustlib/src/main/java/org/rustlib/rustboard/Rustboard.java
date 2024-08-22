package org.rustlib.rustboard;

import static org.rustlib.rustboard.JsonKeys.LAST_NODE_UPDATE_KEY;
import static org.rustlib.rustboard.JsonKeys.NODE_ARRAY_KEY;
import static org.rustlib.rustboard.JsonKeys.NODE_ID_KEY;
import static org.rustlib.rustboard.JsonKeys.NODE_STATE_KEY;
import static org.rustlib.rustboard.JsonKeys.NODE_TYPE_KEY;
import static org.rustlib.rustboard.JsonKeys.UUID_KEY;
import static org.rustlib.rustboard.MessageActions.MESSAGE_ACTION_KEY;
import static org.rustlib.rustboard.MessageActions.NOTIFY;
import static org.rustlib.rustboard.NoticeType.NEUTRAL;
import static org.rustlib.rustboard.NoticeType.NOTICE_DURATION_KEY;
import static org.rustlib.rustboard.NoticeType.NOTICE_MESSAGE_KEY;
import static org.rustlib.rustboard.NoticeType.NOTICE_TYPE_KEY;
import static org.rustlib.rustboard.RustboardServer.NEW_STORED_RUSTBOARD_DIR;
import static org.rustlib.rustboard.RustboardServer.OLD_STORED_RUSTBOARD_DIR;
import static org.rustlib.rustboard.RustboardServer.RUSTBOARD_STORAGE_DIR;
import static org.rustlib.rustboard.RustboardServer.log;

import org.java_websocket.WebSocket;
import org.rustlib.commandsystem.Command;
import org.rustlib.geometry.Pose2d;
import org.rustlib.rustboard.RustboardNode.InvalidNodeJsonException;
import org.rustlib.rustboard.RustboardNode.NoSuchNodeException;
import org.rustlib.rustboard.RustboardNode.Type;
import org.rustlib.utils.FileUtils;

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
 * All public static methods in this class will be executed on the currently <b>active</b> rustboard.  These method calls will refer back to the RustboardServer singleton to access the currently active rustboard instance and then execute the non-static methods of that instance.  This means that you can only update nodes on the active rustboard.
 */
public class Rustboard {
    private static final int defaultNoticeDuration = 5000;
    private static final NoticeType defaultNoticeType = NEUTRAL;

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

    Rustboard(String uuid, JsonArray nodes) {
        this.uuid = uuid;
        nodes.forEach((JsonValue nodeJson) -> {
            try {
                this.nodes.add(RustboardNode.buildFromJson(nodeJson));
            } catch (InvalidNodeJsonException e) {
                log(e);
            }
        });
    }

    Rustboard(String uuid, Set<RustboardNode> nodes) {
        this.uuid = uuid;
        this.nodes.addAll(nodes);
    }

    static Rustboard emptyRustboard(String uuid) {
        return new Rustboard(uuid, new HashSet<>());
    }

    Rustboard(Builder builder) {
        this.uuid = builder.uuid;
        this.nodes.addAll(builder.nodes);
    }

    public static Rustboard getActiveRustboard() {
        return RustboardServer.getInstance().getActiveRustboard();
    }

    static Rustboard requireActiveRustboard() {
        Rustboard activeRustboard = RustboardServer.getInstance().getActiveRustboard();
        if (activeRustboard == null) {
            throw new NullPointerException("No active rustboard available");
        }
        return activeRustboard;
    }

    public String getUuid() {
        return uuid;
    }

    WebSocket getConnection() {
        return connection;
    }

    void setConnection(WebSocket connection) {
        connected = true;
        this.connection = connection;
    }

    static Rustboard load(String uuid) {
        try {
            return loadRustboard(getLatestRustboardVersion(uuid), uuid);
        } catch (Exception e) {
            try {
                return loadRustboard(getPreviousRustboardVersion(uuid), uuid);
            } catch (Exception e1) {
                throw new RustboardLoadException(String.format("Failed to load rustboard '%s'", uuid));
            }
        }
    }

    private static Rustboard loadRustboard(File file, String uuid) throws IOException, InvalidNodeJsonException {
        if (file.exists()) {
            Builder rustboardBuilder = getBuilder().setUUID(uuid);
            JsonObject json = FileUtils.loadJsonObject(file);
            JsonArray nodes = Objects.requireNonNull(json.getJsonArray(NODE_ARRAY_KEY));
            nodes.forEach((JsonValue nodeJson) -> rustboardBuilder.addNode(RustboardNode.buildFromJson(nodeJson)));
            return rustboardBuilder.build();
        } else {
            throw new NoSuchRustboardException(uuid);
        }
    }

    Rustboard mergeWithClientRustboard(JsonArray clientNodes) {
        Set<RustboardNode> updatedNodeList = new HashSet<>(nodes);
        for (JsonValue nodeJson : clientNodes) {
            RustboardNode clientNode = RustboardNode.buildFromJson(nodeJson);
            if (nodeExists(clientNode.id, clientNode.type)) {
                RustboardNode serverNode = getNode(clientNode.id, clientNode.type);
                updatedNodeList.add(serverNode.merge(clientNode));
                updatedNodeList.remove(serverNode);
            } else {
                updatedNodeList.add(clientNode);
            }
        }
        return new Rustboard(uuid, updatedNodeList);
    }

    private void applyNodeUpdateMessage(JsonObject nodeJson) {
        String id = nodeJson.getString(NODE_ID_KEY);
        Type type = Type.getType(nodeJson.getString(NODE_TYPE_KEY));
        String state = nodeJson.getString(NODE_STATE_KEY);
        long lastUpdate;
        try {
            lastUpdate = nodeJson.getJsonNumber(LAST_NODE_UPDATE_KEY).longValue();
        } catch (NullPointerException e) {
            lastUpdate = 0;
        }
        try {
            getNode(id, type).remoteUpdateState(state, lastUpdate);
        } catch (NoSuchNodeException e) {
            this.nodes.add(new RustboardNode(id, type, state));
        }
    }

    void onMessage(JsonObject messageJson) {
        switch (messageJson.getString(MESSAGE_ACTION_KEY)) {
            case MessageActions.UPDATE_NODE:
                applyNodeUpdateMessage(messageJson);
                break;
            case MessageActions.UPDATE_NODES:
                JsonArray nodes = messageJson.getJsonArray(NODE_ARRAY_KEY);
                for (JsonValue value : nodes) {
                    JsonObject nodeJson = value.asJsonObject();
                    applyNodeUpdateMessage(nodeJson);
                }
                break;
            case MessageActions.SAVE_PATH:
                try {
                    FileUtils.writeString(new File(RUSTBOARD_STORAGE_DIR, messageJson.getString(NODE_ID_KEY)), Objects.requireNonNull(messageJson.get("path")).toString());
                    notifyClient("Saved path to robot", NoticeType.POSITIVE, 8000);
                } catch (IOException | NullPointerException e) {
                    notifyClient("Could not save the path to the robot", NoticeType.NEGATIVE, 8000);
                    log(e.toString());
                }
                break;
            case MessageActions.SAVE_VALUE:
                try {
                    FileUtils.writeString(RUSTBOARD_STORAGE_DIR, Objects.requireNonNull(messageJson.get("value")).toString());
                    notifyClient("Saved value to robot", NoticeType.POSITIVE, 8000);
                } catch (IOException | NullPointerException e) {
                    notifyClient("Could not save the value to the robot", NoticeType.NEGATIVE, 8000);
                    log(e.toString());
                }
                break;
            case MessageActions.CLICK_BUTTON:
                String nodeId = messageJson.getString(NODE_ID_KEY);
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
                .add(MESSAGE_ACTION_KEY, NOTIFY)
                .add(NOTICE_MESSAGE_KEY, notice)
                .add(NOTICE_TYPE_KEY, type.value)
                .add(NOTICE_DURATION_KEY, durationMilliseconds)
                .build();
    }

    public void notifyClient(Object notice, NoticeType type, int durationMilliseconds) {
        if (connection != null && connection.isOpen()) {
            connection.send(createNoticeJson(notice.toString(), type, durationMilliseconds).toString());
        }
    }

    public void notifyClient(Object notice, NoticeType type) {
        notifyClient(notice, type, defaultNoticeDuration);
    }

    public void notifyClient(Object notice, int durationMilliseconds) {
        notifyClient(notice, defaultNoticeType, durationMilliseconds);
    }

    public void notifyClient(Object notice) {
        notifyClient(notice, NoticeType.NEUTRAL);
    }

    public static void notifyActiveClient(Object notice, NoticeType type, int durationMilliseconds) {
        if (RustboardServer.isActiveRustboard()) {
            getActiveRustboard().notifyClient(notice.toString(), type, durationMilliseconds);
        }
    }

    public static void notifyActiveClient(Object notice, NoticeType type) {
        notifyActiveClient(notice, type, defaultNoticeDuration);
    }

    public static void notifyActiveClient(Object notice, int durationMilliseconds) {
        notifyActiveClient(notice, defaultNoticeType, durationMilliseconds);
    }

    public static void notifyActiveClient(Object notice) {
        notifyActiveClient(notice, NoticeType.NEUTRAL);
    }

    public static void notifyAllClients(Object notice, NoticeType type, int durationMilliseconds) {
        RustboardServer.getInstance().broadcastToClients(createNoticeJson(notice.toString(), type, durationMilliseconds).toString());
    }

    public static void notifyAllClients(Object notice, NoticeType type) {
        notifyAllClients(notice, type, defaultNoticeDuration);
    }

    public static void notifyAllClients(Object notice, int durationMilliseconds) {
        notifyAllClients(notice, defaultNoticeType, durationMilliseconds);
    }

    public static void notifyAllClients(Object notice) {
        notifyAllClients(notice, NoticeType.NEUTRAL);
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
        RustboardNode toUpdate;
        try {
            toUpdate = activeRustboard.getNode(id, type);
            toUpdate.localUpdateState(value);
        } catch (NoSuchNodeException e) {
            toUpdate = new RustboardNode(id, type, value.toString());
            activeRustboard.nodes.add(toUpdate);
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
        jsonBuilder.add(UUID_KEY, uuid);
        JsonArrayBuilder nodeArray = Json.createArrayBuilder();
        nodes.forEach((RustboardNode node) -> nodeArray.add(node.getJsonBuilder()));
        jsonBuilder.add(NODE_ARRAY_KEY, nodeArray);
        return jsonBuilder.build();
    }

    private static File getLatestRustboardVersion(String uuid) {
        return new File(NEW_STORED_RUSTBOARD_DIR, uuid + ".json");
    }

    private static File getPreviousRustboardVersion(String uuid) {
        return new File(OLD_STORED_RUSTBOARD_DIR, uuid + ".json");
    }

    private void save(File file) {
        try {
            FileUtils.writeJson(file, getJson());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    void save() {
        File newRustboardFile = getLatestRustboardVersion(uuid);
        try {
            FileUtils.copyFile(getLatestRustboardVersion(uuid), getPreviousRustboardVersion(uuid));
        } catch (IOException e) {
            RustboardServer.log(e);
        } finally {
            save(newRustboardFile);
        }
    }

    private static String toFilePath(String fileName) {
        return FileUtils.externalStorage + "\\" + fileName + ".txt";
    }

    public static String loadSavedString(String fileName, String defaultValue) {
        return FileUtils.safeReadString(toFilePath(fileName), defaultValue);
    }

    public static double loadSavedDouble(String fileName, double defaultValue) {
        return FileUtils.loadDouble(toFilePath(fileName), defaultValue);
    }

    public static long loadSavedLong(String fileName, long defaultValue) {
        return FileUtils.loadLong(toFilePath(fileName), defaultValue);
    }

    public static class RustboardException extends RuntimeException {
        public RustboardException(String message) {
            super(message);
        }
    }

    public static class RustboardLoadException extends RustboardException {
        public RustboardLoadException(String message) {
            super(message);
        }
    }

    static class NoSuchRustboardException extends RustboardException {
        NoSuchRustboardException(String uuid) {
            super(String.format("The rustboard with the id '%s' has no corresponding file", uuid));
        }
    }
}
