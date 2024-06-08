package org.rustlib.config;

import org.rustlib.core.RobotBase;
import org.rustlib.utils.Triplet;
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

public class HardwareConfiguration {
    private final ArrayList<Triplet<String, Integer, DeviceType>> devices;
    private final String name;

    private HardwareConfiguration(Builder builder) {
        devices = builder.devices;
        name = builder.name;
    }

    public static Builder getBuilder() {
        return new Builder();
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

    public void setAsActiveConfiguration() {

    }

    public enum DeviceType {
        GOBILDA_5201_SERIES_MOTOR(0, 3, "goBILDA5201SeriesMotor"),
        REV_HD_HEX_MOTOR(0, 3, "RevRoboticsUltraplanetaryHDHexMotor"),
        SERVO(0, 5, "Servo"),
        SMART_SERVO(0, 5, "SmartServo"), // TODO: verify xml tag name
        BLINKIN_LED_DRIVER(0, 5, "RevBlinkinDriver"), // TODO: verify xml
        REV_DISTANCE_SENSOR(0, 5, ""); // TODO: check port max and xml tag name
        private final int portMin;
        private final int portMax;
        private final String xmlTagName;

        DeviceType(int portMin, int portMax, String xmlTagName) {
            this.portMin = portMin;
            this.portMax = portMax;
            this.xmlTagName = xmlTagName;
        }
    }

    public static class Builder {
        private ArrayList<Triplet<String, Integer, DeviceType>> devices = new ArrayList<>();
        private String name = "";

        public Builder addDevice(String id, int port, DeviceType deviceType) {
            if (port < deviceType.portMin || port > deviceType.portMax) {
                throw new IllegalArgumentException(deviceType.name() + " devices may only use ports " + deviceType.portMin + " through " + deviceType.portMax + ".\nProvided port number: " + port);
            }
            devices.add(new Triplet<>(id, port, deviceType));
            return this;
        }

        public Builder setConfigurationName(String name) {
            name = name.trim();
            if (name.contains(" ")) {
                throw new IllegalArgumentException("Configuration name must not contain whitespace");
            }
            this.name = name;
            return this;
        }

        public HardwareConfiguration build() {
            return new HardwareConfiguration(this);
        }
    }
}
