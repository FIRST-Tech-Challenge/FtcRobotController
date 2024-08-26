package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.redux.InitOpMode;
import com.acmerobotics.dashboard.message.redux.ReceiveOpModeList;
import com.acmerobotics.dashboard.message.redux.ReceiveRobotStatus;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.testopmode.TestOpMode;
import com.acmerobotics.dashboard.testopmode.TestOpModeManager;
import fi.iki.elonen.NanoHTTPD;
import fi.iki.elonen.NanoWSD;
import java.io.IOException;
import java.util.stream.Collectors;

enum TestEnum {
    Value1,
    Value2,
    Value3
};

public class TestDashboardInstance {
    private static TestDashboardInstance instance = new TestDashboardInstance();

    final String DEFAULT_OP_MODE_NAME = "$Stop$Robot$";
    TestOpModeManager opModeManager = new TestOpModeManager();

    private TelemetryPacket currentPacket;

    DashboardCore core = new DashboardCore();

    private NanoWSD server = new NanoWSD(8000) {
        @Override
        protected NanoWSD.WebSocket openWebSocket(NanoHTTPD.IHTTPSession handshake) {
            return new DashWebSocket(handshake);
        }
    };

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
                throw new RuntimeException(e);
            }
        }

        @Override
        protected void onOpen() {
            sh.onOpen();

            opModeManager.setSendFun(this);
            send(new ReceiveOpModeList(
                opModeManager
                    .getTestOpModes()
                    .stream().map(TestOpMode::getName)
                    .collect(Collectors.toList())
            ));
        }

        @Override
        protected void onClose(NanoWSD.WebSocketFrame.CloseCode code, String reason,
                               boolean initiatedByRemote) {
            sh.onClose();

            opModeManager.clearSendFun();
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
                    String opModeName;
                    RobotStatus.OpModeStatus opModeStatus;
                    if (opModeManager.getActiveOpMode() == null) {
                        opModeName = DEFAULT_OP_MODE_NAME;
                        opModeStatus = RobotStatus.OpModeStatus.STOPPED;
                    } else {
                        opModeName = opModeManager.getActiveOpMode().getName();
                        opModeStatus = opModeManager.getActiveOpMode().getOpModeStatus();
                    }

                    send(new ReceiveRobotStatus(
                        new RobotStatus(core.enabled, true, opModeName, opModeStatus, "", "", 12.0)
                    ));
                    break;
                }
                case INIT_OP_MODE: {
                    InitOpMode initOpMode = (InitOpMode) msg;
                    opModeManager.initOpMode(initOpMode.getOpModeName());
                    break;
                }
                case START_OP_MODE:
                    opModeManager.startOpMode();
                    break;
                case STOP_OP_MODE:
                    opModeManager.stopOpMode();
                    break;
                default:
                    System.out.println(msg.getType());
            }
        }

        @Override
        protected void onPong(NanoWSD.WebSocketFrame pong) {

        }

        @Override
        protected void onException(IOException exception) {

        }
    }

    public static TestDashboardInstance getInstance() {
        return instance;
    }

    public void start() throws InterruptedException {
        System.out.println("Starting Dashboard instance");

        core.enabled = true;

        core.addConfigVariable("Test", "LATERAL_MULTIPLIER", new ValueProvider<Double>() {
            private double x;

            @Override
            public Double get() {
                return x;
            }

            @Override
            public void set(Double value) {
                x = value;
            }
        });
        core.addConfigVariable("Test", "RUN_USING_ENCODER", new ValueProvider<Boolean>() {
            private boolean b;

            @Override
            public Boolean get() {
                return b;
            }

            @Override
            public void set(Boolean value) {
                b = value;
            }
        });
        core.addConfigVariable("Test", "SomeEnum", new ValueProvider<TestEnum>() {
            private TestEnum te = TestEnum.Value1;

            @Override
            public TestEnum get() {
                return te;
            }

            @Override
            public void set(TestEnum value) {
                te = value;
            }
        });

        core.addConfigVariable("More Primitives", "Long", new ValueProvider<Long>() {
            private long value = 10L;

            @Override
            public Long get() {
                return this.value;
            }

            @Override
            public void set(Long value) {
                this.value = value;
            }
        });
        core.addConfigVariable("More Primitives", "Float", new ValueProvider<Float>() {
            private float value = 10L;

            @Override
            public Float get() {
                return this.value;
            }

            @Override
            public void set(Float value) {
                this.value = value;
            }
        });

        try {
            server.start();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        while (true) {
            opModeManager.loop();
            Thread.yield();
        }
    }

    public void addData(String x, Object o) {
        if (currentPacket == null) {
            currentPacket = new TelemetryPacket();
        }

        currentPacket.put(x, o);
    }

    public void update() {
        if (currentPacket != null) {
            core.sendTelemetryPacket(currentPacket);
            currentPacket = null;
        }
    }

    public void sendTelemetryPacket(TelemetryPacket t) {
        core.sendTelemetryPacket(t);
    }
}
