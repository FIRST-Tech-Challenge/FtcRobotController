package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.ConfigVariableDeserializer;
import com.acmerobotics.dashboard.config.variable.ConfigVariableSerializer;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageDeserializer;
import com.acmerobotics.dashboard.message.MessageType;
import com.acmerobotics.dashboard.message.redux.ReceiveConfig;
import com.acmerobotics.dashboard.message.redux.ReceiveTelemetry;
import com.acmerobotics.dashboard.message.redux.SaveConfig;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Main class for interacting with the instance.
 */
public class DashboardCore {
    /*
     * Telemetry packets are batched for transmission and sent at this interval.
     */
    private static final int DEFAULT_TELEMETRY_TRANSMISSION_INTERVAL = 100; // ms

    public boolean enabled;

    private final Mutex<List<SendFun>> sockets = new Mutex<>(new ArrayList<>());

    private ExecutorService telemetryExecutorService;
    // NOTE: We're doing fancy stuff that precludes the use of Mutex.
    private final List<TelemetryPacket> pendingTelemetry = new ArrayList<>(); // guarded by itself
    private volatile int telemetryTransmissionInterval = DEFAULT_TELEMETRY_TRANSMISSION_INTERVAL;

    private final Mutex<CustomVariable> configRoot = new Mutex<>(new CustomVariable());

    // NOTE: Helps to have this here for testing
    public static final Gson GSON = new GsonBuilder()
        .registerTypeAdapter(Message.class, new MessageDeserializer())
        .registerTypeAdapter(BasicVariable.class, new ConfigVariableSerializer())
        .registerTypeAdapter(BasicVariable.class, new ConfigVariableDeserializer())
        .registerTypeAdapter(CustomVariable.class, new ConfigVariableSerializer())
        .registerTypeAdapter(CustomVariable.class, new ConfigVariableDeserializer())
        .serializeNulls()
        .create();

    private class TelemetryUpdateRunnable implements Runnable {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    List<TelemetryPacket> telemetryToSend;

                    synchronized (pendingTelemetry) {
                        while (pendingTelemetry.isEmpty()) {
                            pendingTelemetry.wait();
                        }

                        telemetryToSend = new ArrayList<>(pendingTelemetry);
                        pendingTelemetry.clear();
                    }

                    // only the latest packet non-empty field overlay is used
                    // this helps save bandwidth, especially for more complex overlays
                    for (int i = telemetryToSend.size() - 1; i >= 0; i--) {
                        TelemetryPacket packet = telemetryToSend.get(i);
                        if (!packet.fieldOverlay().getOperations().isEmpty()) {
                            for (int j = 0; j < i; j++) {
                                TelemetryPacket packet2 = telemetryToSend.get(j);
                                packet2.field().clear();
                                packet2.fieldOverlay().clear();
                            }
                            break;
                        } else {
                            packet.field().clear();
                        }
                    }

                    sendAll(new ReceiveTelemetry(telemetryToSend));

                    Thread.sleep(telemetryTransmissionInterval);
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    }

    public DashboardCore() {
        telemetryExecutorService =
            Executors.newSingleThreadExecutor(r -> new Thread(r, "dash telemetry"));
        telemetryExecutorService.submit(new TelemetryUpdateRunnable());
    }

    public SocketHandler newSocket(final SendFun sendFun) {
        return new SocketHandler() {
            @Override
            public void onOpen() {
                configRoot.with(v -> {
                    sendFun.send(new ReceiveConfig(v));
                });

                sockets.with(l -> {
                    l.add(sendFun);
                });
            }

            @Override
            public void onClose() {
                sockets.with(l -> {
                    l.remove(sendFun);
                });
            }

            @Override
            public boolean onMessage(Message message) {
                // Swallow any messages when the server is disabled.
                if (!enabled && message.getType() != MessageType.GET_ROBOT_STATUS) {
                    return true;
                }

                switch (message.getType()) {
                    case GET_CONFIG: {
                        configRoot.with(v -> {
                            sendFun.send(new ReceiveConfig(v));
                        });
                        return true;
                    }
                    case SAVE_CONFIG: {
                        withConfigRoot(new CustomVariableConsumer() {
                            @Override
                            public void accept(CustomVariable configRoot) {
                                configRoot.update(((SaveConfig) message).getConfigDiff());
                            }
                        });

                        return true;
                    }
                    default:
                        return false;
                }
            }
        };
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
        if (!enabled) {
            return;
        }

        telemetryPacket.addTimestamp();

        synchronized (pendingTelemetry) {
            // TODO: a circular buffer is probably a better idea, but this will work for now
            if (pendingTelemetry.size() > 100) {
                return;
            }

            pendingTelemetry.add(telemetryPacket);

            // There should only be one thread, so we should avoid a thundering herd. (But then
            // why not just go with notify()?)
            pendingTelemetry.notifyAll();
        }
    }

    /**
     * Clears telemetry data from all clients.
     */
    public void clearTelemetry() {
        synchronized (pendingTelemetry) {
            pendingTelemetry.clear();

            sendAll(new ReceiveTelemetry(Collections.<TelemetryPacket>emptyList()));
        }
    }

    /**
     * Returns the telemetry transmission interval in milliseconds.
     */
    public int getTelemetryTransmissionInterval() {
        return telemetryTransmissionInterval;
    }

    /**
     * Sets the telemetry transmission interval.
     *
     * @param newTransmissionInterval transmission interval in milliseconds
     */
    public void setTelemetryTransmissionInterval(int newTransmissionInterval) {
        telemetryTransmissionInterval = newTransmissionInterval;
    }

    /**
     * Sends updated configuration data to all instance clients.
     */
    public void updateConfig() {
        configRoot.with(v -> {
            sendAll(new ReceiveConfig(v));
        });
    }

    /**
     * Executes {@param function} in an exclusive context for thread-safe config tree modification
     * and calls {@link #updateConfig()} to keep clients up to date.
     * <p>
     * Do not leak the config tree outside the function.
     *
     * @param function
     */
    public void withConfigRoot(CustomVariableConsumer function) {
        configRoot.with(function::accept);

        updateConfig();
    }

    /**
     * Add config variable with custom provider.
     *
     * @param category top-level category
     * @param name     variable name
     * @param provider getter/setter for the variable
     * @param <T>      variable type
     */
    public <T> void addConfigVariable(String category, String name, ValueProvider<T> provider) {
        configRoot.with(v -> {
            CustomVariable catVar = (CustomVariable) v.getVariable(category);
            if (catVar != null) {
                catVar.putVariable(name, new BasicVariable<>(provider));
            } else {
                catVar = new CustomVariable();
                catVar.putVariable(name, new BasicVariable<>(provider));
                v.putVariable(category, catVar);
            }
            updateConfig();
        });
    }

    /**
     * Remove a config variable.
     *
     * @param category top-level category
     * @param name     variable name
     */
    public void removeConfigVariable(String category, String name) {
        configRoot.with(v -> {
            CustomVariable catVar = (CustomVariable) v.getVariable(category);
            catVar.removeVariable(name);
            if (catVar.size() == 0) {
                v.removeVariable(category);
            }
            updateConfig();
        });
    }

    public void sendAll(Message message) {
        sockets.with(l -> {
            for (SendFun sf : l) {
                sf.send(message);
            }
        });
    }

    public int clientCount() {
        return sockets.with(l -> {
            return l.size();
        });
    }
}
