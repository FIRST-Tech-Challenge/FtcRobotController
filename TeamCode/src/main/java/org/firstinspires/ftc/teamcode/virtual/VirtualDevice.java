package org.firstinspires.ftc.teamcode.virtual;

import org.json.JSONException;
import org.json.JSONObject;

public abstract class VirtualDevice {

    VirtualHardwareManager parentConnectionManager;
    String deviceName;

    public void setParentConnectionManager(VirtualHardwareManager parentConnectionManager) {
        this.parentConnectionManager = parentConnectionManager;
    }

    /**
     *
     * @return A JSONObject to send, but if no new data needs to be sent, send null.
     */
    abstract JSONObject getDataToTransmit() throws JSONException;
    abstract void updateDevice(JSONObject newData);
}
