package org.rustlib.rustboard;

import static org.rustlib.config.Loader.defaultStorageDirectory;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.rustlib.config.Loader;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.Timer;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonValue;

public class RustboardServer extends WebSocketServer {
    public static final int port = 21865;
    static final File rustboardStorageDir = new File(defaultStorageDirectory + "\\Rustboard");
    private static final File rustboardMetadataFile = new File(rustboardStorageDir + "\\rustboard_metadata.json");
    private static final File storedRustboardDir = new File(rustboardStorageDir + "\\rustboards");
    private static RustboardServer instance = null;
    private static boolean debugMode = true;
    private Rustboard activeRustboard;
    private final JsonObject rustboardMetaData;
    private final HashSet<String> storedRustboardIds = new HashSet<>();
    private final HashMap<String, Rustboard> loadedRustboards = new HashMap<>();

    private final Set<WebSocket> connections = new HashSet<>();

    public Rustboard getActiveRustboard() {
        if (activeRustboard == null) {
            activeRustboard = Rustboard.emptyRustboard();
        }
        return activeRustboard;
    }

    private RustboardServer(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
        setReuseAddr(true);
        RobotLog.v("Rustboard server initialized");
        if (!rustboardMetadataFile.exists()) {
            try {
                rustboardMetadataFile.createNewFile();
            } catch (IOException e) {
                throw new RuntimeException("Could not start the Rustboard server: unable to load metadata file");
            }
        }
        try {
            rustboardMetaData = Loader.loadJsonObject(rustboardMetadataFile);
            JsonArray dataArray = rustboardMetaData.getJsonArray("rustboards");
            for (JsonValue value : dataArray) {
                JsonObject data = (JsonObject) value;
                String uuid = data.getString("uuid");
                storedRustboardIds.add(uuid);
                if (data.getBoolean("active")) {
                    try {
                        activeRustboard = Rustboard.load(uuid);
                    } catch (Rustboard.NoSuchRustboardException e) {
                        e.printStackTrace();
                    }
                }
            }
        } finally {
            if (!rustboardStorageDir.exists()) {
                rustboardStorageDir.mkdir();
            }
        }
    }

    static void messageActiveRustboard(JsonObject json) {

    }

    public static void enableDebugMode() {
        debugMode = false;
    }

    public static void disableDebugMode() {
        debugMode = true;
    }

    public static boolean inDebugMode() {
        return debugMode;
    }

    public static void log(Object value) {
        log(value.toString());
    }

    private static void clearStorage() {
        File[] files = storedRustboardDir.listFiles();
        if (files != null) {
            for (File file : files) {
                file.delete();
            }
        }
    }

    public static RustboardServer getInstance() {
        if (instance == null) {
            try {
                instance = new RustboardServer(port);
                RobotLog.v("dashboard server started");
            } catch (UnknownHostException e) {
                e.printStackTrace();
            }
        }
        return instance;
    }

    @Override
    public void start() {
        if (!debugMode) {
            super.start();
        }
        new Timer().scheduleAtFixedRate(ClientUpdater.getInstance(), 0, 50);
    }

    @Override
    public void stop() throws IOException, InterruptedException {
        // TODO: save rustboards
        super.stop();
    }

    @Override
    public void onStart() {
        setConnectionLostTimeout(3);
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        connections.remove(conn);
        if (activeRustboard.getConnection() == conn) {
            activeRustboard.onDisconnect();
        }
        log("client " + conn.getRemoteSocketAddress().toString() + " disconnected from the robot.");
    }

    void setActiveRustboard(Rustboard rustboard) {
        if (activeRustboard == null) {
            activeRustboard = Objects.requireNonNull(rustboard);
        }
    }

    @Override
    @SuppressWarnings("ConstantConditions")
    public void onMessage(WebSocket conn, String message) {
        JsonObject messageJson = Loader.readJsonString(message);
        switch (messageJson.getString("action")) {
            case "client_details":
                Time.calibrateUTCTime(Long.parseLong(messageJson.getString("utc_time")));
                String uuid = messageJson.getString("uuid");
                JsonObject rustboardJson = messageJson.getJsonObject("rustboard");
                JsonArray clientNodes = messageJson.getJsonArray("nodes");
                Rustboard rustboard;
                if (loadedRustboards.containsKey(uuid)) {
                    rustboard = loadedRustboards.get(uuid).mergeWithClientRustboard(clientNodes);
                } else if (storedRustboardIds.contains(uuid)) {
                    try {
                        rustboard = Rustboard.load(uuid).mergeWithClientRustboard(clientNodes);
                    } catch (Rustboard.NoSuchRustboardException e) {
                        rustboard = new Rustboard(uuid, rustboardJson);
                    }
                } else {
                    rustboard = new Rustboard(uuid, rustboardJson);
                }
                if (activeRustboard == null || !activeRustboard.isConnected()) {
                    activeRustboard = rustboard;
                }
                loadedRustboards.put(uuid, rustboard);

                break;
            case "exception":
                throw new RustboardException(messageJson.getString("exception_message"));
            default:
                if (activeRustboard != null) {
                    activeRustboard.onMessage(messageJson);
                }
        }
    }

    public void saveAll() {
        JsonObjectBuilder metadataBuilder = Json.createObjectBuilder(rustboardMetaData);
        JsonArrayBuilder rustboardArrayBuilder = Json.createArrayBuilder(rustboardMetaData.getJsonArray("rustboards"));
        metadataBuilder.add("rustboards", rustboardArrayBuilder);
        loadedRustboards.forEach((uuid, rustboard) -> {
                    JsonObjectBuilder rustboardDescriptor = Json.createObjectBuilder();
                    rustboardDescriptor.add("uuid", uuid);
                    rustboardDescriptor.add("active", rustboard == activeRustboard);
                    rustboardArrayBuilder.add(rustboardDescriptor);
                    rustboard.save(new File(rustboardStorageDir, uuid + ".json"));
                }
        ); // TODO: add method to get file
    }

    @Override
    public void onError(WebSocket conn, Exception e) {
        log(e);
    }

    public void startOpMode(String opModeName) {
        RegisteredOpModes.getInstance().getOpMode(opModeName).init();
    }

    public List<OpModeMeta> getRegisteredOpModes() {
        return RegisteredOpModes.getInstance().getOpModes();
    }

    public class RustboardException extends RuntimeException {
        public RustboardException(String message) {
            super(message);
        }
    }
}