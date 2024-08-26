package com.acmerobotics.dashboard;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.Typeface;
import android.util.Base64;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.LinearLayout;
import android.widget.TextView;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.redux.InitOpMode;
import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.acmerobotics.dashboard.message.redux.ReceiveImage;
import com.acmerobotics.dashboard.message.redux.ReceiveOpModeList;
import com.acmerobotics.dashboard.message.redux.ReceiveRobotStatus;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.qualcomm.robotcore.util.WebHandlerManager;
import com.qualcomm.robotcore.util.WebServer;
import dalvik.system.DexFile;
import fi.iki.elonen.NanoHTTPD;
import fi.iki.elonen.NanoWSD;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.ftccommon.external.OnCreateMenu;
import org.firstinspires.ftc.ftccommon.external.OnDestroy;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.ftccommon.internal.FtcRobotControllerWatchdogService;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.robotserver.internal.webserver.MimeTypesUtil;

/**
 * Main class for interacting with the instance.
 */
public class FtcDashboard implements OpModeManagerImpl.Notifications {
    private static final String TAG = "FtcDashboard";

    private static final int DEFAULT_IMAGE_QUALITY = 50; // 0-100
    private static final int GAMEPAD_WATCHDOG_INTERVAL = 500; // ms

    private static boolean suppressOpMode = false;

    private static final String PREFS_NAME = "FtcDashboard";
    private static final String PREFS_AUTO_ENABLE_KEY = "autoEnable";

    private static FtcDashboard instance;

    @OpModeRegistrar
    public static void registerOpMode(OpModeManager manager) {
        if (instance != null && !suppressOpMode) {
            instance.internalRegisterOpMode(manager);
        }
    }

    /**
     * Call before start to suppress the enable/disable op mode.
     */
    public static void suppressOpMode() {
        suppressOpMode = true;
    }

    /**
     * Starts the dashboard.
     */
    @OnCreate
    public static void start(Context context) {
        if (instance == null) {
            instance = new FtcDashboard();
        }
    }

    /**
     * Attaches a web server for accessing the dashboard through the phone (like OBJ/Blocks).
     */
    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        if (instance != null) {
            instance.internalAttachWebServer(manager.getWebServer());
        }
    }

    /**
     * Attaches the event loop to the instance for op mode management.
     */
    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        if (instance != null) {
            instance.internalAttachEventLoop(eventLoop);
        }
    }

    /**
     * Populates the menu with dashboard enable/disable options.
     *
     * @param menu menu
     */
    @OnCreateMenu
    public static void populateMenu(Context context, Menu menu) {
        if (instance != null) {
            instance.internalPopulateMenu(menu);
        }
    }

    /**
     * Stops the instance and the underlying WebSocket server.
     */
    @OnDestroy
    public static void stop(Context context) {
        if (!FtcRobotControllerWatchdogService.isLaunchActivity(
            AppUtil.getInstance().getRootActivity())) {
            // prevent premature stop when the app is launched via hardware attachment
            return;
        }

        if (instance != null) {
            instance.close();
            instance = null;
        }
    }

    /**
     * Returns the active instance instance. This should be called after {@link #start(Context)}.
     *
     * @return active instance instance or null outside of its lifecycle
     */
    public static FtcDashboard getInstance() {
        return instance;
    }

    /**
     * @return a boolean indicating if the dashboard is currently active
     */
    public boolean isEnabled() { return core.enabled; }

    private DashboardCore core = new DashboardCore();

    private NanoWSD server = new NanoWSD(8000) {
        @Override
        protected NanoWSD.WebSocket openWebSocket(NanoHTTPD.IHTTPSession handshake) {
            return new DashWebSocket(handshake);
        }
    };

    private SharedPreferences prefs;
    private final List<MenuItem> enableMenuItems;
    private final List<MenuItem> disableMenuItems;

    private Telemetry telemetry = new TelemetryAdapter();

    private ExecutorService cameraStreamExecutor;
    private int imageQuality = DEFAULT_IMAGE_QUALITY;

    // only modified inside withConfigRoot
    private final List<String[]> varsToRemove = new ArrayList<>();

    private FtcEventLoop eventLoop;
    private OpModeManagerImpl opModeManager;

    private final Mutex<OpModeAndStatus> activeOpMode = new Mutex<>(new OpModeAndStatus());

    private final Mutex<List<String>> opModeList = new Mutex<>(new ArrayList<>());

    private ExecutorService gamepadWatchdogExecutor;
    private long lastGamepadTimestamp;

    private boolean webServerAttached;

    private TextView connectionStatusTextView;
    private LinearLayout parentLayout;

    private static class OpModeAndStatus {
        public OpMode opMode;
        public RobotStatus.OpModeStatus status = RobotStatus.OpModeStatus.STOPPED;
    }

    private class GamepadWatchdogRunnable implements Runnable {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                long timestamp = System.currentTimeMillis();
                try {
                    if (lastGamepadTimestamp == 0) {
                        Thread.sleep(GAMEPAD_WATCHDOG_INTERVAL);
                    } else if ((timestamp - lastGamepadTimestamp) > GAMEPAD_WATCHDOG_INTERVAL) {
                        activeOpMode.with(o -> {
                            o.opMode.gamepad1.copy(new Gamepad());
                            o.opMode.gamepad2.copy(new Gamepad());
                        });
                        lastGamepadTimestamp = 0;
                    } else {
                        Thread.sleep(GAMEPAD_WATCHDOG_INTERVAL
                            - (timestamp - lastGamepadTimestamp));
                    }
                } catch (InterruptedException e) {
                    break;
                }
            }
        }
    }

    private class ListOpModesRunnable implements Runnable {
        @Override
        public void run() {
            RegisteredOpModes.getInstance().waitOpModesRegistered();

            opModeList.with(l -> {
                l.clear();
                for (OpModeMeta opModeMeta : RegisteredOpModes.getInstance().getOpModes()) {
                    if (opModeMeta.flavor != OpModeMeta.Flavor.SYSTEM) {
                        l.add(opModeMeta.name);
                    }
                }
                Collections.sort(l);
                sendAll(new ReceiveOpModeList(l));
            });
        }
    }

    /**
     * Adapter to use dashboard telemetry like normal SDK telemetry. Note that this doesn't support
     * all of the operations yet.
     */
    private class TelemetryAdapter implements Telemetry {
        private TelemetryPacket currentPacket;
        private LogAdapter log;

        public TelemetryAdapter() {
            currentPacket = new TelemetryPacket();
            log = new LogAdapter(currentPacket);
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return addData(caption, String.format(format, args));
        }

        @Override
        public Item addData(String caption, Object value) {
            currentPacket.put(caption, value);
            return null;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            throw new UnsupportedOperationException();
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean removeItem(Item item) {
            throw new UnsupportedOperationException();
        }

        @Override
        public void clear() {
            clearTelemetry();

            currentPacket = new TelemetryPacket();
            log = new LogAdapter(currentPacket);
        }

        @Override
        public void clearAll() {
            clear();
        }

        @Override
        public Object addAction(Runnable action) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean removeAction(Object token) {
            throw new UnsupportedOperationException();
        }

        @Override
        public void speak(String text) {
            throw new UnsupportedOperationException();
        }

        @Override
        public void speak(String text, String languageCode, String countryCode) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean update() {
            sendTelemetryPacket(currentPacket);

            currentPacket = new TelemetryPacket();
            log = new LogAdapter(currentPacket);

            return true;
        }

        @Override
        public Line addLine() {
            return null;
        }

        @Override
        public Line addLine(String lineCaption) {
            currentPacket.addLine(lineCaption);
            return null;
        }

        @Override
        public boolean removeLine(Line line) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean isAutoClear() {
            return false;
        }

        @Override
        public void setAutoClear(boolean autoClear) {
            throw new UnsupportedOperationException();
        }

        @Override
        public int getMsTransmissionInterval() {
            return getTelemetryTransmissionInterval();
        }

        @Override
        public void setMsTransmissionInterval(int msTransmissionInterval) {
            setTelemetryTransmissionInterval(msTransmissionInterval);
        }

        @Override
        public String getItemSeparator() {
            return null;
        }

        @Override
        public void setItemSeparator(String itemSeparator) {

        }

        @Override
        public String getCaptionValueSeparator() {
            return null;
        }

        @Override
        public void setCaptionValueSeparator(String captionValueSeparator) {

        }

        @Override
        public void setDisplayFormat(DisplayFormat displayFormat) {

        }

        @Override
        public Log log() {
            return log;
        }
    }

    private static class LogAdapter implements Telemetry.Log {
        private TelemetryPacket telemetryPacket;

        private LogAdapter(TelemetryPacket packet) {
            telemetryPacket = packet;
        }

        @Override
        public int getCapacity() {
            return 0;
        }

        @Override
        public void setCapacity(int capacity) {

        }

        @Override
        public DisplayOrder getDisplayOrder() {
            return DisplayOrder.OLDEST_FIRST;
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {

        }

        @Override
        public void add(String entry) {
            telemetryPacket.addLine(entry);
        }

        @Override
        public void add(String format, Object... args) {
            telemetryPacket.addLine(String.format(format, args));
        }

        @Override
        public void clear() {
            telemetryPacket.clearLines();
        }
    }

    private static String bitmapToJpegString(Bitmap bitmap, int quality) {
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        bitmap.compress(Bitmap.CompressFormat.JPEG, quality, outputStream);
        return Base64.encodeToString(outputStream.toByteArray(), Base64.DEFAULT);
    }

    private class CameraStreamRunnable implements Runnable {
        private CameraStreamSource source;
        private double maxFps;

        private CameraStreamRunnable(CameraStreamSource source, double maxFps) {
            this.source = source;
            this.maxFps = maxFps;
        }

        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    final long timestamp = System.currentTimeMillis();

                    if (core.clientCount() == 0) {
                        Thread.sleep(250);
                        continue;
                    }

                    final CountDownLatch latch = new CountDownLatch(1);
                    source.getFrameBitmap(Continuation.createTrivial(new Consumer<Bitmap>() {
                        @Override
                        public void accept(Bitmap value) {
                            sendAll(new ReceiveImage(bitmapToJpegString(value, imageQuality)));
                            latch.countDown();
                        }
                    }));

                    latch.await();

                    if (maxFps == 0) {
                        continue;
                    }

                    long sleepTime = (long) (1000 / maxFps
                        - (System.currentTimeMillis() - timestamp));
                    Thread.sleep(Math.max(sleepTime, 0));
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    private static final Set<String> IGNORED_PACKAGES = new HashSet<>(Arrays.asList(
        "java",
        "android",
        "com.sun",
        "com.vuforia",
        "com.google",
        "kotlin"
    ));

    private static void addConfigClasses(CustomVariable customVariable) {
        ClassLoader classLoader = FtcDashboard.class.getClassLoader();

        Context context = AppUtil.getInstance().getApplication();
        try {
            DexFile dexFile = new DexFile(context.getPackageCodePath());

            List<String> classNames = Collections.list(dexFile.entries());

            for (String className : classNames) {
                boolean skip = false;
                for (String prefix : IGNORED_PACKAGES) {
                    if (className.startsWith(prefix)) {
                        skip = true;
                        break;
                    }
                }

                if (skip) {
                    continue;
                }

                try {
                    Class<?> configClass = Class.forName(className, false, classLoader);

                    if (!configClass.isAnnotationPresent(Config.class)
                        || configClass.isAnnotationPresent(Disabled.class)) {
                        continue;
                    }

                    String name = configClass.getSimpleName();
                    String altName = configClass.getAnnotation(Config.class).value();
                    if (!altName.isEmpty()) {
                        name = altName;
                    }

                    customVariable.putVariable(name,
                        ReflectionConfig.createVariableFromClass(configClass));
                } catch (ClassNotFoundException | NoClassDefFoundError ignored) {
                    // dash is unable to access many classes and reporting every instance
                    // only clutters the logs
                }
            }
        } catch (IOException e) {
            RobotLog.logStackTrace(e);
        }
    }

    private class DashWebSocket extends NanoWSD.WebSocket implements SendFun {
        final SocketHandler sh = core.newSocket(this);

        public DashWebSocket(NanoHTTPD.IHTTPSession handshakeRequest) {
            super(handshakeRequest);
        }

        @Override
        public void send(Message message) {
            try {
                String messageStr = DashboardCore.GSON.toJson(message);
                send(messageStr);
            } catch (IOException e) {
                // NOTE: It's possible that the socket has closed and we have a backlog of messages
                // to send. Settle for logging here instead of trying to get all the checks right.
                RobotLog.logStackTrace(e);
            }
        }

        @Override
        protected void onOpen() {
            sh.onOpen();

            opModeList.with(l -> {
                if (l.size() > 0) {
                    send(new ReceiveOpModeList(l));
                }
            });

            updateStatusView();
        }

        @Override
        protected void onClose(NanoWSD.WebSocketFrame.CloseCode code, String reason,
                               boolean initiatedByRemote) {
            sh.onClose();

            updateStatusView();
        }

        @Override
        protected void onMessage(NanoWSD.WebSocketFrame message) {
            String payload = message.getTextPayload();
            Message msg = DashboardCore.GSON.fromJson(payload, Message.class);

            if (sh.onMessage(msg)) {
                return;
            }

            switch (msg.getType()) {
                case GET_ROBOT_STATUS: {
                    send(new ReceiveRobotStatus(getRobotStatus()));
                    break;
                }
                case INIT_OP_MODE: {
                    String opModeName = ((InitOpMode) msg).getOpModeName();
                    opModeManager.initOpMode(opModeName);
                    break;
                }
                case START_OP_MODE: {
                    opModeManager.startActiveOpMode();
                    break;
                }
                case STOP_OP_MODE: {
                    eventLoop.requestOpModeStop(opModeManager.getActiveOpMode());
                    break;
                }
                case RECEIVE_GAMEPAD_STATE: {
                    ReceiveGamepadState castMsg = (ReceiveGamepadState) msg;
                    updateGamepads(castMsg.getGamepad1(), castMsg.getGamepad2());
                    break;
                }
                default: {
                    Log.w(TAG, "Received unknown message of type " + msg.getType());
                    Log.w(TAG, msg.toString());
                    break;
                }
            }
        }

        @Override
        protected void onPong(NanoWSD.WebSocketFrame pong) {

        }

        @Override
        protected void onException(IOException exception) {

        }
    }

    private FtcDashboard() {
        core.withConfigRoot(new CustomVariableConsumer() {
            @Override
            public void accept(CustomVariable configRoot) {
                addConfigClasses(configRoot);
            }
        });

        try {
            server.start();
        } catch (IOException e) {
            RobotLog.logStackTrace(e);
        }

        enableMenuItems = new ArrayList<>();
        disableMenuItems = new ArrayList<>();

        Activity activity = AppUtil.getInstance().getActivity();
        prefs = activity.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        if (getAutoEnable()) {
            enable();
        }

        injectStatusView();
    }

    private boolean getAutoEnable() {
        return prefs.getBoolean(PREFS_AUTO_ENABLE_KEY, true);
    }

    private void setAutoEnable(boolean autoEnable) {
        prefs.edit()
            .putBoolean(PREFS_AUTO_ENABLE_KEY, autoEnable)
            .apply();
    }

    private void enable() {
        if (core.enabled) {
            return;
        }

        setAutoEnable(true);

        gamepadWatchdogExecutor = ThreadPool.newSingleThreadExecutor("gamepad watchdog");
        gamepadWatchdogExecutor.submit(new GamepadWatchdogRunnable());

        core.enabled = true;

        updateStatusView();
    }

    private void disable() {
        if (!core.enabled) {
            return;
        }

        setAutoEnable(false);

        gamepadWatchdogExecutor.shutdownNow();

        stopCameraStream();

        core.enabled = false;

        updateStatusView();
    }

    private void injectStatusView() {
        Activity activity = AppUtil.getInstance().getActivity();

        if (activity == null) {
            return;
        }

        connectionStatusTextView = new TextView(activity);
        connectionStatusTextView.setTypeface(Typeface.DEFAULT_BOLD);
        int color = activity.getResources().getColor(R.color.dashboardColor);
        connectionStatusTextView.setTextColor(color);
        int horizontalMarginId = activity.getResources().getIdentifier(
            "activity_horizontal_margin", "dimen", activity.getPackageName());
        int horizontalMargin = (int) activity.getResources().getDimension(horizontalMarginId);
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            LinearLayout.LayoutParams.WRAP_CONTENT
        );
        params.setMargins(horizontalMargin, 0, horizontalMargin, 0);
        connectionStatusTextView.setLayoutParams(params);

        int parentLayoutId = activity.getResources().getIdentifier(
            "entire_screen", "id", activity.getPackageName());
        parentLayout = activity.findViewById(parentLayoutId);
        int childCount = parentLayout.getChildCount();
        int relativeLayoutId = activity.getResources().getIdentifier(
            "RelativeLayout", "id", activity.getPackageName());
        int i;
        for (i = 0; i < childCount; i++) {
            if (parentLayout.getChildAt(i).getId() == relativeLayoutId) {
                break;
            }
        }
        final int relativeLayoutIndex = i;
        AppUtil.getInstance().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                parentLayout.addView(connectionStatusTextView, relativeLayoutIndex);
            }
        });

        updateStatusView();
    }

    private void removeStatusView() {
        if (parentLayout != null && connectionStatusTextView != null) {
            AppUtil.getInstance().runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    parentLayout.removeView(connectionStatusTextView);
                }
            });
        }
    }

    private void updateStatusView() {
        if (connectionStatusTextView != null) {
            AppUtil.getInstance().runOnUiThread(new Runnable() {
                @SuppressLint("SetTextI18n")
                @Override
                public void run() {
                    if (!core.enabled) {
                        connectionStatusTextView.setText("Dashboard: disabled");
                        return;
                    }

                    String serverStatus = webServerAttached ? "server attached" : "server detached";

                    String connStatus;
                    int connections = core.clientCount();
                    if (connections == 0) {
                        connStatus = "no connections";
                    } else if (connections == 1) {
                        connStatus = "1 connection";
                    } else {
                        connStatus = connections + " connections";
                    }

                    connectionStatusTextView.setText(
                        "Dashboard: " + serverStatus + ", " + connStatus);
                }
            });
        }
    }

    private WebHandler newStaticAssetHandler(final AssetManager assetManager, final String file) {
        return new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session)
                throws IOException {
                if (session.getMethod() == NanoHTTPD.Method.GET) {
                    String mimeType = MimeTypesUtil.determineMimeType(file);
                    return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                        mimeType, assetManager.open(file));
                } else {
                    return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                        NanoHTTPD.MIME_PLAINTEXT, "");
                }
            }
        };
    }

    private void addAssetWebHandlers(WebHandlerManager webHandlerManager,
                                     AssetManager assetManager, String path) {
        try {
            String[] list = assetManager.list(path);

            if (list == null) {
                return;
            }

            if (list.length > 0) {
                for (String file : list) {
                    addAssetWebHandlers(webHandlerManager, assetManager, path + "/" + file);
                }
            } else {
                webHandlerManager.register("/" + path,
                    newStaticAssetHandler(assetManager, path));
            }
        } catch (IOException e) {
            Log.w(TAG, e);
        }
    }

    private void internalAttachWebServer(WebServer webServer) {
        if (webServer == null) {
            return;
        }

        Activity activity = AppUtil.getInstance().getActivity();

        if (activity == null) {
            return;
        }

        WebHandlerManager webHandlerManager = webServer.getWebHandlerManager();
        AssetManager assetManager = activity.getAssets();
        webHandlerManager.register("/dash",
            newStaticAssetHandler(assetManager, "dash/index.html"));
        webHandlerManager.register("/dash/",
            newStaticAssetHandler(assetManager, "dash/index.html"));
        addAssetWebHandlers(webHandlerManager, assetManager, "dash");

        addAssetWebHandlers(webHandlerManager, assetManager, "images");

        webServerAttached = true;

        updateStatusView();
    }

    private void internalAttachEventLoop(FtcEventLoop eventLoop) {
        this.eventLoop = eventLoop;

        // this could be called multiple times within the lifecycle of the dashboard
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }

        opModeManager = eventLoop.getOpModeManager();
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        Thread t = new Thread(new ListOpModesRunnable());
        t.start();
    }

    private void internalPopulateMenu(Menu menu) {
        MenuItem enable = menu.add(Menu.NONE, Menu.NONE, 700, "Enable Dashboard");
        MenuItem disable = menu.add(Menu.NONE, Menu.NONE, 700, "Disable Dashboard");

        enable.setVisible(!core.enabled);
        disable.setVisible(core.enabled);

        synchronized (enableMenuItems) {
            enableMenuItems.add(enable);
        }

        synchronized (disableMenuItems) {
            disableMenuItems.add(disable);
        }

        enable.setOnMenuItemClickListener(new MenuItem.OnMenuItemClickListener() {
            @Override
            public boolean onMenuItemClick(MenuItem item) {
                enable();

                synchronized (enableMenuItems) {
                    for (MenuItem menuItem : enableMenuItems) {
                        menuItem.setVisible(false);
                    }
                }

                synchronized (disableMenuItems) {
                    for (MenuItem menuItem : disableMenuItems) {
                        menuItem.setVisible(true);
                    }
                }

                return true;
            }
        });

        disable.setOnMenuItemClickListener(new MenuItem.OnMenuItemClickListener() {
            @Override
            public boolean onMenuItemClick(MenuItem item) {
                disable();

                synchronized (enableMenuItems) {
                    for (MenuItem menuItem : enableMenuItems) {
                        menuItem.setVisible(true);
                    }
                }

                synchronized (disableMenuItems) {
                    for (MenuItem menuItem : disableMenuItems) {
                        menuItem.setVisible(false);
                    }
                }

                return true;
            }
        });
    }

    private void internalRegisterOpMode(OpModeManager manager) {
        manager.register(
            new OpModeMeta.Builder()
                .setName("Enable/Disable Dashboard")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup("dash")
                .build(),
            new LinearOpMode() {
                @Override
                public void runOpMode() throws InterruptedException {
                    telemetry.log().add(
                        Misc.formatInvariant("Dashboard is currently %s. Press Start to %s it.",
                            core.enabled ? "enabled" : "disabled",
                            core.enabled ? "disable" : "enable"));
                    telemetry.update();

                    waitForStart();

                    if (isStopRequested()) {
                        return;
                    }

                    if (core.enabled) {
                        disable();
                    } else {
                        enable();
                    }
                }
            });
    }

    /**
     * Queues a telemetry packet to be sent to all clients. Packets are sent in batches of
     * approximate period {@link #getTelemetryTransmissionInterval()}. Clients display the most
     * recent value received for each key, and the data is cleared upon op mode init or a call to
     * {@link #clearTelemetry()}.
     *
     * @param telemetryPacket packet to send
     */
    public void sendTelemetryPacket(TelemetryPacket telemetryPacket) {
        core.sendTelemetryPacket(telemetryPacket);
    }

    /**
     * Clears telemetry data from all clients.
     */
    public void clearTelemetry() {
        core.clearTelemetry();
    }

    /**
     * Returns a {@link Telemetry} object that delegates to the telemetry methods of this class.
     * Beware that the implementation of the interface is incomplete, and users should test each
     * method they intend to use.
     */
    public Telemetry getTelemetry() {
        return telemetry;
    }

    /**
     * Returns the telemetry transmission interval in milliseconds.
     */
    public int getTelemetryTransmissionInterval() {
        return core.getTelemetryTransmissionInterval();
    }

    /**
     * Sets the telemetry transmission interval.
     *
     * @param newTransmissionInterval transmission interval in milliseconds
     */
    public void setTelemetryTransmissionInterval(int newTransmissionInterval) {
        core.setTelemetryTransmissionInterval(newTransmissionInterval);
    }

    /**
     * Sends updated configuration data to all instance clients.
     */
    public void updateConfig() {
        core.updateConfig();
    }

    /**
     * Executes {@param function} in an exclusive context for thread-safe config tree modification
     * and calls {@link #updateConfig()} to keep clients up to date.
     *
     * <p>Do not leak the config tree outside the function.
     *
     * @param function custom variable consumer
     */
    public void withConfigRoot(CustomVariableConsumer function) {
        core.withConfigRoot(function);
    }

    /**
     * Add config variable with custom provider that is automatically removed when op mode ends.
     *
     * @param category top-level category
     * @param name     variable name
     * @param provider getter/setter for the variable
     * @param <T>      variable type
     */
    public <T> void addConfigVariable(String category, String name, ValueProvider<T> provider) {
        core.addConfigVariable(category, name, provider);
    }

    /**
     * Add config variable with custom provider.
     *
     * @param category   top-level category
     * @param name       variable name
     * @param provider   getter/setter for the variable
     * @param autoRemove if true, the variable is removed on op mode termination
     * @param <T>        variable type
     */
    public <T> void addConfigVariable(final String category, final String name,
                                      final ValueProvider<T> provider,
                                      final boolean autoRemove) {
        withConfigRoot(new CustomVariableConsumer() {
            @Override
            public void accept(CustomVariable configRoot) {
                core.addConfigVariable(category, name, provider);

                if (autoRemove) {
                    varsToRemove.add(new String[] {category, name});
                }
            }
        });
    }

    /**
     * Remove a config variable.
     *
     * @param category top-level category
     * @param name     variable name
     */
    public void removeConfigVariable(String category, String name) {
        core.removeConfigVariable(category, name);
    }

    /**
     * Sends an image to the dashboard for display (MJPEG style). Note that the encoding process is
     * synchronous. Stops the active stream if running.
     *
     * @param bitmap bitmap to send
     */
    public void sendImage(Bitmap bitmap) {
        if (!core.enabled) {
            return;
        }

        stopCameraStream();

        sendAll(new ReceiveImage(bitmapToJpegString(bitmap, imageQuality)));
    }

    /**
     * Sends a stream of camera frames at a regular interval.
     *
     * @param source camera stream source
     * @param maxFps maximum frames per second; 0 indicates unlimited
     */
    public void startCameraStream(CameraStreamSource source, double maxFps) {
        if (!core.enabled) {
            return;
        }

        stopCameraStream();

        cameraStreamExecutor = ThreadPool.newSingleThreadExecutor("camera stream");
        cameraStreamExecutor.submit(new CameraStreamRunnable(source, maxFps));
    }

    /**
     * Stops the camera frame stream.
     */
    public void stopCameraStream() {
        if (cameraStreamExecutor != null) {
            cameraStreamExecutor.shutdownNow();
            cameraStreamExecutor = null;
        }
    }

    /**
     * Returns the image quality used by {@link #sendImage(Bitmap)} and
     * {@link #startCameraStream(CameraStreamSource, double)}.
     */
    public int getImageQuality() {
        return imageQuality;
    }

    /**
     * Sets the image quality used by {@link #sendImage(Bitmap)} and
     * {@link #startCameraStream(CameraStreamSource, double)}.
     */
    public void setImageQuality(int quality) {
        imageQuality = quality;
    }

    public static void copyIntoSdkGamepad(ReceiveGamepadState.Gamepad src, Gamepad dst) {
        dst.left_stick_x = src.left_stick_x;
        dst.left_stick_y = src.left_stick_y;
        dst.right_stick_x = src.right_stick_x;
        dst.right_stick_y = src.right_stick_y;

        dst.dpad_up = src.dpad_up;
        dst.dpad_down = src.dpad_down;
        dst.dpad_left = src.dpad_left;
        dst.dpad_right = src.dpad_right;

        dst.a = src.a;
        dst.b = src.b;
        dst.x = src.x;
        dst.y = src.y;

        dst.guide = src.guide;
        dst.start = src.start;
        dst.back = src.back;

        dst.left_bumper = src.left_bumper;
        dst.right_bumper = src.right_bumper;

        dst.left_stick_button = src.left_stick_button;
        dst.right_stick_button = src.right_stick_button;

        dst.left_trigger = src.left_trigger;
        dst.right_trigger = src.right_trigger;

        dst.touchpad = src.touchpad;
    }

    private void updateGamepads(ReceiveGamepadState.Gamepad gamepad1,
                                ReceiveGamepadState.Gamepad gamepad2) {
        activeOpMode.with(o -> {
            // for now, the dashboard only overrides synthetic gamepads
            if (o.status == RobotStatus.OpModeStatus.STOPPED) {
                return;
            }

            if (o.opMode.gamepad1.getGamepadId() != Gamepad.ID_UNASSOCIATED
                || o.opMode.gamepad2.getGamepadId() != Gamepad.ID_UNASSOCIATED) {
                return;
            }

            copyIntoSdkGamepad(gamepad1, o.opMode.gamepad1);
            copyIntoSdkGamepad(gamepad2, o.opMode.gamepad2);
            lastGamepadTimestamp = System.currentTimeMillis();
        });
    }

    private RobotStatus getRobotStatus() {
        if (opModeManager == null) {
            return new RobotStatus(core.enabled, false, "", RobotStatus.OpModeStatus.STOPPED, "",
                "", -1.0);
        } else {
            return activeOpMode.with(o -> {
                double batteryVoltage = -1.0;
                if (o.opMode.hardwareMap != null) {
                    for (LynxModule m : o.opMode.hardwareMap.getAll(LynxModule.class)) {
                        batteryVoltage =
                            Math.max(batteryVoltage, m.getInputVoltage(VoltageUnit.VOLTS));
                    }
                }

                return new RobotStatus(
                    core.enabled, true, opModeManager.getActiveOpModeName(),
                    // status is an enum so it's okay to return a copy here.
                    o.status,
                    RobotLog.getGlobalWarningMessage().message, RobotLog.getGlobalErrorMsg(),
                    batteryVoltage
                );
            });
        }
    }

    private void sendAll(Message message) {
        core.sendAll(message);
    }

    private void close() {
        server.stop();

        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
        disable();

        removeStatusView();
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        activeOpMode.with(o -> {
            o.opMode = opMode;
            o.status = RobotStatus.OpModeStatus.INIT;
        });

        if (!(opMode instanceof OpModeManagerImpl.DefaultOpMode)) {
            clearTelemetry();
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        activeOpMode.with(o -> {
            o.opMode = opMode;
            o.status = RobotStatus.OpModeStatus.RUNNING;
        });
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        activeOpMode.with(o -> {
            o.opMode = opMode;
            o.status = RobotStatus.OpModeStatus.STOPPED;
        });

        // this callback is sometimes called from the UI thread
        (new Thread() {
            @Override
            public void run() {
                withConfigRoot(new CustomVariableConsumer() {
                    @Override
                    public void accept(CustomVariable configRoot) {
                        for (String[] var : varsToRemove) {
                            String category = var[0];
                            String name = var[1];
                            CustomVariable catVar =
                                (CustomVariable) configRoot.getVariable(category);
                            catVar.removeVariable(name);
                            if (catVar.size() == 0) {
                                configRoot.removeVariable(category);
                            }
                        }
                        varsToRemove.clear();
                    }
                });
            }
        }).start();

        stopCameraStream();
    }
}
