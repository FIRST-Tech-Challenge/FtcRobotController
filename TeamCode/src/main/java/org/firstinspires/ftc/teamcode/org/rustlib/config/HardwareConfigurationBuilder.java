package org.firstinspires.ftc.teamcode.org.rustlib.config;

import android.util.Pair;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.util.ArrayList;

public class HardwareConfigurationBuilder {

    private final ArrayList<Pair<String, DeviceType>> devices = new ArrayList<>();

    enum DeviceType {
        GOBILDA_5201_SERIES("", 0, 3),
        SERVO("Servo", 0, 5),
        SMART_SERVO("", 0, 5),
        BLINKIN_LED_DRIVER("", 0, 5);
        private final String deviceName;
        private final int portMin;
        private final int portMax;

        DeviceType(String deviceName, int portMin, int portMax) {
            this.deviceName = deviceName;
            this.portMin = portMin;
            this.portMax = portMax;
        }
    }

    public void build() {

    }


    private HardwareConfiguration getActiveConfig() {
        Document xml = PreferenceEditor.readPreferences();
        return null;
    }

    private void setActiveConfig() {
        Document preferences = PreferenceEditor.readPreferences();
        NodeList strings = preferences.getElementsByTagName("string");
        for (int i = 0; i < strings.getLength(); i++) {
            Node node = strings.item(i);
            if (node.getAttributes().getNamedItem("name").equals("pref_hardware_config_filename")) {
                node.getTextContent();

            }
        }
    }

    private void reloadHardwareMap() {

    }

    public HardwareConfigurationBuilder addDevice(String id, int port, DeviceType deviceType) {
        if (port < deviceType.portMin || port > deviceType.portMax) {
            throw new IllegalArgumentException(deviceType.name() + " devices may only use ports " + deviceType.portMin + " through " + deviceType.portMax + ".\nProvided port number: " + port);
        }
        devices.add(new Pair<>(id, deviceType));
        return this;
    }
}
