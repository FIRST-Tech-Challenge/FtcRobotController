package org.rustlib.rustboard;

import android.os.Environment;
import android.util.Pair;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class RustboardServer extends WebSocketServer {
    public static final int port = 21865;
    private static final File storageDir = new File(Environment.getExternalStorageDirectory() + "\\Download");
    private static RustboardServer instance = null;
    private static boolean debugMode = true;
    ElapsedTime timer;
    private ArrayList<Pair<String, String>> messageQueue = new ArrayList<>();
    private long timeOffset = 0;
    private Rustboard activeRustboard;

    private RustboardServer(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
        setReuseAddr(true);
        timer = new ElapsedTime();
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

    public static void setNodeValue(String id, String value) {
        /*if (!debugMode) {
            JsonObject jsonObject = RustboardLayout.getSendableNodeData(id, value);
            getInstance().broadcastJson(jsonObject);
        }*/ // TODO: get rid of this method
    }

    public static void log(Object value) {
        getInstance().log(value.toString());
    }

    public static void log(String value) {
        /*JsonObject message = Json.createObjectBuilder()
                .add("messageType", "log")
                .add("value", getInstance().timer.milliseconds() + ": " + value)
                .build();
        getInstance().broadcastJson(message);*/ // TODO
    }

    private static void clearStorage() {
        File[] files = storageDir.listFiles();
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

    private void negotiateClientId() {

    }

    private void activateRustboard(Rustboard rustboard, WebSocket connection) {
        activeRustboard = rustboard;
        rustboard.onConnect(connection);
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        if (activeRustboard.getConnection() == conn) {
            activeRustboard.onDisconnect();
        }
        log("client " + conn.getRemoteSocketAddress().toString() + " disconnected from the robot.");
    }

    @Override
    public void onMessage(WebSocket conn, String data) {
        activeRustboard.onMessage(data);
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

    static class Timestamp {
        private static Set<Timestamp> registry = new HashSet<>();
        private static long offset = 0;
        private static boolean calibrated;

        static long getTime() {
            return System.currentTimeMillis() + offset;
        }

        static boolean isTimeCalibrated() {
            return calibrated;
        }

        static void calibrateTime(long actualTime) {
            if (!calibrated) {
                double offset = actualTime - System.currentTimeMillis();
                registry.forEach((timestamp) -> timestamp.timeMillis += offset);
                calibrated = true;
            }
        }

        private long timeMillis;

        Timestamp(long timeMillis) {
            this.timeMillis = timeMillis;
            registry.add(this);
        }

        Timestamp() {
            this(getTime());
        }

        long getTimeMillis() {
            return timeMillis;
        }

        @Override
        public boolean equals(Object o) {
            if (o instanceof Timestamp) {
                return Math.abs(((Timestamp) o).timeMillis - timeMillis) < 1;
            }
            return false;
        }
    }
}