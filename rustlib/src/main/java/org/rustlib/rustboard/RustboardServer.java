package org.rustlib.rustboard;

import static org.rustlib.rustboard.JsonKeys.ACTIVE_KEY;
import static org.rustlib.rustboard.JsonKeys.CONSOLE_INFO_KEY;
import static org.rustlib.rustboard.JsonKeys.EXCEPTION_MESSAGE_KEY;
import static org.rustlib.rustboard.JsonKeys.NODE_ARRAY_KEY;
import static org.rustlib.rustboard.JsonKeys.RUSTBOARD_ARRAY_KEY;
import static org.rustlib.rustboard.JsonKeys.UTC_KEY;
import static org.rustlib.rustboard.JsonKeys.UUID_KEY;
import static org.rustlib.rustboard.MessageActions.CONSOLE_LOG;
import static org.rustlib.rustboard.MessageActions.CONSOLE_WARN;
import static org.rustlib.rustboard.MessageActions.MESSAGE_ACTION_KEY;
import static org.rustlib.rustboard.MessageActions.SET_ACTIVE;
import static org.rustlib.rustboard.Rustboard.RustboardLoadException;
import static org.rustlib.utils.FileUtils.clearDir;
import static org.rustlib.utils.FileUtils.externalStorage;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.rustlib.core.RobotBase;
import org.rustlib.rustboard.Rustboard.RustboardException;
import org.rustlib.utils.FileUtils;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonValue;

public class RustboardServer extends WebSocketServer { // TODO: public, but only for now
    public static final int port = 5801;
    static final File RUSTBOARD_STORAGE_DIR = new File(externalStorage, "Rustboard");
    private static final File RUSTBOARD_METADATA_FILE = new File(RUSTBOARD_STORAGE_DIR, "rustboard_metadata.json");
    static final File OLD_STORED_RUSTBOARD_DIR = new File(RUSTBOARD_STORAGE_DIR, "rustboards_previous");
    static final File NEW_STORED_RUSTBOARD_DIR = new File(RUSTBOARD_STORAGE_DIR, "rustboards_latest");
    private static RustboardServer instance = null;
    private static boolean debugMode = false;
    private Rustboard activeRustboard;
    private final JsonObject rustboardMetaData;
    private final HashSet<String> storedRustboardIds = new HashSet<>();
    private final HashMap<String, Rustboard> loadedRustboards = new HashMap<>();
    private static final int rustboardAutoSavePeriod = 10000;
    private static final int pingClientPeriod = 4000;
    private static final String pingClientMessage = "{\"action\": \"ping\"}";
    private static final ClientUpdater clientUpdater = new ClientUpdater();
    private final ScheduledExecutorService executorService = new ScheduledThreadPoolExecutor(3);
    private final Set<WebSocket> connections = Collections.synchronizedSet(new HashSet<>());

    ClientUpdater getClientUpdater() {
        return clientUpdater;
    }

    private void autoSave() {
        if (!RobotBase.opModeRunning()) {
            saveLayouts();
        }
        executorService.schedule(this::autoSave, rustboardAutoSavePeriod, TimeUnit.MILLISECONDS);
    }

    public Rustboard getActiveRustboard() {
        if (activeRustboard == null) {
            activeRustboard = Rustboard.emptyRustboard(null);
        }
        return activeRustboard;
    }

    private RustboardServer(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
        setReuseAddr(true);
        FileUtils.makeDirIfMissing(RUSTBOARD_STORAGE_DIR);
        JsonObject defaultMetaData = Json.createObjectBuilder().add(RUSTBOARD_ARRAY_KEY, Json.createArrayBuilder().build()).build();
        rustboardMetaData = FileUtils.safeLoadJsonObject(RUSTBOARD_METADATA_FILE, defaultMetaData);
        if (rustboardMetaData.containsKey(RUSTBOARD_ARRAY_KEY)) {
            JsonArray dataArray = rustboardMetaData.getJsonArray(RUSTBOARD_ARRAY_KEY);
            for (JsonValue value : dataArray) {
                JsonObject rustboardDescriptor = value.asJsonObject();
                String uuid = rustboardDescriptor.getString(UUID_KEY);
                if (uuid != null) {
                    storedRustboardIds.add(uuid);
                }
                if (rustboardDescriptor.getBoolean(ACTIVE_KEY)) {
                    Rustboard rustboard;
                    try {
                        rustboard = Rustboard.load(uuid);
                    } catch (RustboardLoadException e) {
                        rustboard = Rustboard.emptyRustboard(uuid);
                    }
                    activeRustboard = rustboard;
                    loadedRustboards.put(uuid, rustboard);
                }
            }

        }
        executorService.scheduleAtFixedRate(clientUpdater, 0, 50, TimeUnit.MILLISECONDS);
        executorService.scheduleAtFixedRate(() -> {
            connections.forEach((connection) -> {
                if (connection != null && connection.isOpen()) {
                    connection.send(pingClientMessage);
                }
            });
        }, pingClientPeriod, pingClientPeriod, TimeUnit.MILLISECONDS);
        executorService.schedule(this::autoSave, 10000, TimeUnit.MILLISECONDS);
    }

    static void messageActiveRustboard(JsonObject json) {
        WebSocket connection = getInstance().activeRustboard.getConnection();
        if (connection != null && connection.isOpen())
            connection.send(json.toString());
    }

    public static void enableDebugMode() { // TODO: make a good way to enable and disable debug mode
        debugMode = false;
    }

    public static void disableDebugMode() {
        debugMode = true;
    }

    public static boolean inDebugMode() {
        return debugMode;
    }

    public static void log(Object value) { // TODO: create simpler logger

    }

    private static void clearStorage() throws IOException {
        clearDir(RUSTBOARD_STORAGE_DIR, true);
    }

    public static RustboardServer getInstance() {
        if (instance == null) {
            try {
                instance = new RustboardServer(port);
            } catch (UnknownHostException e) {
                e.printStackTrace();
            }
        }
        return instance;
    }

    @Override
    public void start() {
        if (!debugMode) {
            try {
                super.start();
            } catch (IllegalStateException e) {
                log(e);
            }
        }
    }

    @Override
    public void stop() {
        saveLayouts();
        try {
            super.stop();
        } catch (IOException | InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void onStart() {
        setConnectionLostTimeout(3);
    }

    public static boolean isActiveRustboard() {
        return getInstance().activeRustboard != null && getInstance().activeRustboard.isConnected();
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        connections.add(conn);
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        connections.remove(conn);
        if (activeRustboard.getConnection() == conn) {
            activeRustboard.onDisconnect();
        }
        for (Rustboard rustboard : loadedRustboards.values()) {
            if (rustboard.isConnected()) {
                activateRustboard(rustboard);
                break;
            }
        }
        log("client " + conn.getRemoteSocketAddress().toString() + " disconnected from the robot.");
    }

    @Override
    @SuppressWarnings("ConstantConditions")
    public void onMessage(WebSocket conn, String message) {
        JsonObject messageJson = FileUtils.readJsonString(message);
        try {
            switch (messageJson.getString(MESSAGE_ACTION_KEY)) {
                case MessageActions.CLIENT_DETAILS:
                    Time.calibrateUTCTime(messageJson.getJsonNumber(UTC_KEY).longValue());
                    String uuid = messageJson.getString(UUID_KEY);
                    JsonArray clientNodes = messageJson.getJsonArray(NODE_ARRAY_KEY);
                    Rustboard rustboard;
                    if (loadedRustboards.containsKey(uuid)) {
                        rustboard = loadedRustboards.get(uuid).mergeWithClientRustboard(clientNodes);
                    } else if (storedRustboardIds.contains(uuid)) {
                        try {
                            rustboard = Rustboard.load(uuid).mergeWithClientRustboard(clientNodes);
                        } catch (RustboardLoadException e) {
                            rustboard = new Rustboard(uuid, clientNodes);
                        }
                    } else {
                        rustboard = new Rustboard(uuid, clientNodes);
                    }
                    rustboard.setConnection(conn);
                    loadedRustboards.put(uuid, rustboard);
                    if (isActiveRustboard() && !rustboard.getUuid().equals(activeRustboard.getUuid())) {
                        rustboard.notifyClient("Rustboard queued because another Rustboard is connected", NoticeType.NEUTRAL, 5000);
                    } else {
                        activateRustboard(rustboard);
                    }
                    break;
                case MessageActions.EXCEPTION:
                    throw new RustboardException(messageJson.getString(EXCEPTION_MESSAGE_KEY));
                default:
                    if (activeRustboard != null) {
                        activeRustboard.onMessage(messageJson);
                    }
            }
        } catch (Exception e) {
            Rustboard.notifyAllClients("Robot received an invalid websocket message");
            warnClientConsoles(e);
            log(e);
            throw new RuntimeException(e.getMessage() + "\n" + message); // TODO: remove after debugging
        }
    }

    private void activateRustboard(Rustboard rustboard) {
        activeRustboard = rustboard;
        JsonObjectBuilder builder = Json.createObjectBuilder();
        builder.add(MESSAGE_ACTION_KEY, SET_ACTIVE);
        WebSocket connection = rustboard.getConnection();
        if (connection != null) {
            connection.send(builder.build().toString());
        }
    }

    public void saveLayouts() {
        FileUtils.makeDirIfMissing(OLD_STORED_RUSTBOARD_DIR);
        FileUtils.makeDirIfMissing(NEW_STORED_RUSTBOARD_DIR);
        JsonObjectBuilder metadataBuilder = Json.createObjectBuilder();
        ArrayList<JsonValue> currentRustboardArray = new ArrayList<>(rustboardMetaData.getJsonArray(RUSTBOARD_ARRAY_KEY)); // JsonArrays are immutable, and calling remove() on one won't work.  The solution is to create a mutable arraylist using the JsonArray instance
        JsonArrayBuilder rustboardArrayBuilder = Json.createArrayBuilder();
        loadedRustboards.forEach((uuid, rustboard) -> {
                    currentRustboardArray.removeIf((rustboardJson) -> rustboardJson.asJsonObject().getString(UUID_KEY).equals(uuid));
                    JsonObjectBuilder rustboardDescriptor = Json.createObjectBuilder();
                    rustboardDescriptor.add(UUID_KEY, uuid);
                    rustboardDescriptor.add(ACTIVE_KEY, rustboard == activeRustboard);
                    rustboardArrayBuilder.add(rustboardDescriptor);
                    rustboard.save();
                }
        );
        currentRustboardArray.forEach(rustboardArrayBuilder::add);
        metadataBuilder.add(RUSTBOARD_ARRAY_KEY, rustboardArrayBuilder);
        try {
            FileUtils.writeJson(RUSTBOARD_METADATA_FILE, metadataBuilder.build());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void onError(WebSocket conn, Exception e) {
        log(e);
    }

    public Set<WebSocket> getConnectedSockets() {
        return connections;
    }

    public void broadcastToClients(String message) {
        for (WebSocket connection : connections) {
            if (connection.isOpen()) {
                connection.send(message);
            }
        }
    }

    public static void logToClientConsoles(String info) {
        JsonObjectBuilder builder = Json.createObjectBuilder();
        builder.add(MESSAGE_ACTION_KEY, CONSOLE_LOG);
        builder.add(CONSOLE_INFO_KEY, info);
        getInstance().broadcastToClients(builder.build().toString());
    }

    public static void logToClientConsoles(Object info) {
        logToClientConsoles(info.toString());
    }

    public static void warnClientConsoles(String info) {
        JsonObjectBuilder builder = Json.createObjectBuilder();
        builder.add(MESSAGE_ACTION_KEY, CONSOLE_WARN);
        builder.add(CONSOLE_INFO_KEY, info);
        getInstance().broadcastToClients(builder.build().toString());
    }

    public static void warnClientConsoles(Exception e) {
        warnClientConsoles(e.getMessage());
    }
}