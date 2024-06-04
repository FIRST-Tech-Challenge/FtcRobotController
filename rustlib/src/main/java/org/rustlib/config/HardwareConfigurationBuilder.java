package org.rustlib.config;

import android.util.Pair;

import org.rustlib.core.RobotBase;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Objects;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.xml.transform.TransformerException;

public class HardwareConfigurationBuilder {
    private final ArrayList<Pair<String, DeviceType>> devices = new ArrayList<>();

    public static HardwareConfigurationBuilder get() {
        return new HardwareConfigurationBuilder();
    }

    public HardwareConfiguration build() {
        return new HardwareConfiguration();
    }

    private HardwareConfiguration getActiveConfig() {
        Document xml = PreferenceEditor.readPreferences();
        return null;
    }

    private Node getActiveConfigurationXMLNode() {
        Document preferencesDocument = PreferenceEditor.readPreferences();
        NodeList strings = preferencesDocument.getElementsByTagName("string");
        for (int i = 0; i < strings.getLength(); i++) {
            Node node = strings.item(i);
            if (Objects.equals(node.getAttributes().getNamedItem("name"), "pref_hardware_config_filename")) {
                return node;
            }
        }
        return null;
    }

    public File getActiveConfiguration() throws FileNotFoundException, NullPointerException {
        Node node = getActiveConfigurationXMLNode();
        assert node != null;
        String text = node.getTextContent();
        if (text.contains("LOCAL_STORAGE")) {
            Pattern pattern = Pattern.compile("name&quot;:&quot;(.*?)&quot;");
            Matcher matcher = pattern.matcher(text);
            return new File(Loader.localStorage, matcher.group(0) + ".xml");
        } else {
            throw new FileNotFoundException("Could not locate the active configuration file.");
        }
    }

    private void setActiveConfiguration(String configurationName) throws IOException, TransformerException {
        getActiveConfigurationXMLNode().setTextContent(String.format("{&quot;isDirty&quot;:false,&quot;location&quot;:&quot;LOCAL_STORAGE&quot;,&quot;name&quot;:&quot;%s&quot;,&quot;resourceId&quot;:0}", configurationName));
        RobotBase.getMainActivity().reloadHardwareMap();
    }

    public HardwareConfigurationBuilder addDevice(String id, int port, DeviceType deviceType) {
        if (port < deviceType.portMin || port > deviceType.portMax) {
            throw new IllegalArgumentException(deviceType.name() + " devices may only use ports " + deviceType.portMin + " through " + deviceType.portMax + ".\nProvided port number: " + port);
        }
        devices.add(new Pair<>(id, deviceType));
        return this;
    }

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
}
