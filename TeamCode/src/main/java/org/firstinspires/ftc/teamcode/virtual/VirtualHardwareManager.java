package org.firstinspires.ftc.teamcode.virtual;

import com.neovisionaries.ws.client.ThreadType;
import com.neovisionaries.ws.client.WebSocket;
import com.neovisionaries.ws.client.WebSocketException;
import com.neovisionaries.ws.client.WebSocketFactory;
import com.neovisionaries.ws.client.WebSocketFrame;
import com.neovisionaries.ws.client.WebSocketListener;
import com.neovisionaries.ws.client.WebSocketState;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class VirtualHardwareManager implements WebSocketListener {

    RobotHardware robotHardware;

    public enum VirtualHardwareType {
        MOTOR("motor");

        private String type;
        VirtualHardwareType(String type) {
            this.type = type;
        }


        @Override
        public String toString() {
            return this.type;
        }
    }


    static WebSocketFactory webSocketFactory = new WebSocketFactory();
    WebSocket ws;

    HashMap<String, VirtualHardwareType> availableHardware = new HashMap<>();
    ArrayList<VirtualDevice> virtualDevices = new ArrayList<>();
    HashMap<String, JSONObject> deviceUpdates = new HashMap<>();

    public void setRobotHardware(RobotHardware hardware) {
        robotHardware = hardware;
    }

    public void connect(String host, int port) throws IOException, WebSocketException {
        ws = webSocketFactory.createSocket(String.format("ws://%s:%d", host, port));
        ws.addListener(this);
        ws.connect();
        try {
            this.requestAvailableHardware();
        } catch (JSONException e) {
            if (robotHardware != null) {
                robotHardware.telemetry.addData("can't request hardware", e.getLocalizedMessage());
            }
        }
    }


    public void sendMessage(String text) {
        if (ws != null) {
            if (ws.isOpen()) {
                ws.sendText(text);
            }
        }
    }

    public void requestAvailableHardware() throws JSONException {
        if (ws != null) {
            JSONObject jo = new JSONObject();
            jo.put("cmd", "requesthw");
            ws.sendText(jo.toString());
        }
    }
    public void updateDevices() throws JSONException {
        // Transmit new information
        JSONObject payload = new JSONObject();
        JSONObject devicePayload = new JSONObject();
        for (VirtualDevice device : this.virtualDevices) {
            JSONObject deviceData = device.getDataToTransmit();
            if (deviceData != null) {
                devicePayload.put(device.deviceName, deviceData);
            }
        }

        deviceUpdates.clear();
    }


    public void disconnect() {
        if (ws != null) {
            ws.disconnect();
        }
    }

    public <T extends VirtualDevice> T initializeVirtualDevice(Class<? extends T> deviceClass, String name) {
        T device;
        try {
            device = deviceClass.newInstance();
            device.setParentConnectionManager(this);
            virtualDevices.add(device);
        } catch (Exception e) {
            return null;
        }

        return device;
    }


    //region WebSocketListener Methods
    @Override
    public void onStateChanged(WebSocket websocket, WebSocketState newState) throws Exception {

    }

    @Override
    public void onConnected(WebSocket websocket, Map<String, List<String>> headers) throws Exception {

    }

    @Override
    public void onConnectError(WebSocket websocket, WebSocketException cause) throws Exception {

    }

    @Override
    public void onDisconnected(WebSocket websocket, WebSocketFrame serverCloseFrame, WebSocketFrame clientCloseFrame, boolean closedByServer) throws Exception {
        if (closedByServer && robotHardware != null) {
            robotHardware.requestOpModeStop();
        }
    }

    @Override
    public void onFrame(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onContinuationFrame(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onTextFrame(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onBinaryFrame(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onCloseFrame(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onPingFrame(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onPongFrame(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onTextMessage(WebSocket websocket, String text) throws Exception {
        JSONObject jo = new JSONObject(text);
    }

    @Override
    public void onTextMessage(WebSocket websocket, byte[] data) throws Exception {

    }

    @Override
    public void onBinaryMessage(WebSocket websocket, byte[] binary) throws Exception {

    }

    @Override
    public void onSendingFrame(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onFrameSent(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onFrameUnsent(WebSocket websocket, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onThreadCreated(WebSocket websocket, ThreadType threadType, Thread thread) throws Exception {

    }

    @Override
    public void onThreadStarted(WebSocket websocket, ThreadType threadType, Thread thread) throws Exception {

    }

    @Override
    public void onThreadStopping(WebSocket websocket, ThreadType threadType, Thread thread) throws Exception {

    }

    @Override
    public void onError(WebSocket websocket, WebSocketException cause) throws Exception {

    }

    @Override
    public void onFrameError(WebSocket websocket, WebSocketException cause, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onMessageError(WebSocket websocket, WebSocketException cause, List<WebSocketFrame> frames) throws Exception {

    }

    @Override
    public void onMessageDecompressionError(WebSocket websocket, WebSocketException cause, byte[] compressed) throws Exception {

    }

    @Override
    public void onTextMessageError(WebSocket websocket, WebSocketException cause, byte[] data) throws Exception {

    }

    @Override
    public void onSendError(WebSocket websocket, WebSocketException cause, WebSocketFrame frame) throws Exception {

    }

    @Override
    public void onUnexpectedError(WebSocket websocket, WebSocketException cause) throws Exception {

    }

    @Override
    public void handleCallbackError(WebSocket websocket, Throwable cause) throws Exception {

    }

    @Override
    public void onSendingHandshake(WebSocket websocket, String requestLine, List<String[]> headers) throws Exception {

    }
    //endregion

}
