package org.rustlib.config;

import org.rustlib.core.RobotBase;
import org.rustlib.utils.FileUtils;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.TransformerException;

public class HardwareConfiguration {
    private final ArrayList<String> hardwareDeviceTags = new ArrayList<>();
    private final String name;

    private HardwareConfiguration(Builder builder) {
        name = builder.name;
    }

    public static SetConfigurationName getBuilder() {
        return new Builder();
    }

    private static Node getActiveConfigurationXMLNode() {
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

    public static File getActiveConfiguration() throws FileNotFoundException, NullPointerException {
        Node node = getActiveConfigurationXMLNode();
        assert node != null;
        String text = node.getTextContent();
        if (text.contains("LOCAL_STORAGE")) {
            Pattern pattern = Pattern.compile("name&quot;:&quot;(.*?)&quot;");
            Matcher matcher = pattern.matcher(text);
            return new File(FileUtils.localStorage, matcher.group(0) + ".xml");
        } else {
            throw new FileNotFoundException("Could not locate the active configuration file.");
        }
    }

    private void setActiveConfiguration(String configurationName) throws IOException, TransformerException {
        getActiveConfigurationXMLNode().setTextContent(String.format("{&quot;isDirty&quot;:false,&quot;location&quot;:&quot;LOCAL_STORAGE&quot;,&quot;name&quot;:&quot;%s&quot;,&quot;resourceId&quot;:0}", configurationName));
        RobotBase.getMainActivity().reloadHardwareMap();
    }

    private HardwareConfiguration getActiveConfig() {
        Document xml = PreferenceEditor.readPreferences();
        return null;
    }

    public void makeActiveConfiguration() {

    }

    private interface XmlDevice {
        String getXmlTagName();
    }

    public enum HubType {
        CONTROL_HUB,
        EXPANSION_HUB;
    }

    public enum Motors implements XmlDevice {
        GOBILDA_5201_SERIES_MOTOR("goBILDA5201SeriesMotor"),
        REV_ULTRAPLANETARY_HD_HEX_MOTOR("RevRoboticsUltraplanetaryHDHexMotor"),
        REV_40_HD_HEX_MOTOR("RevRobotics40HDHexMotor"),
        REV_20_HD_HEX_MOTOR("RevRobotics20HDHexMotor"),
        NEVEREST_40_MOTOR("NeveRest40Gearmotor"),
        NEVEREST_20_MOTOR("NeveRest20Gearmotor");
        private static final int portMax = 3;
        private final String xmlTagName;

        Motors(String xmlTagName) {
            this.xmlTagName = xmlTagName;
        }

        public String getXmlTagName() {
            return xmlTagName;
        }
    }

    public enum Servos implements XmlDevice {
        SERVO("Servo"),
        SMART_SERVO("SmartServo"), // TODO: verify xml tag name
        REV_BLINKIN_LED_DRIVER("RevBlinkinLedDriver");
        private static final int portMax = 5;
        private final String xmlTagName;

        Servos(String xmlTagName) {
            this.xmlTagName = xmlTagName;
        }

        public String getXmlTagName() {
            return xmlTagName;
        }
    }

    public enum DigitalDevices implements XmlDevice {
        REV_TOUCH_SENSOR("RevTouchSensor"),
        REV_DISTANCE_SENSOR(""); // TODO: find xml tag name
        private final String xmlTagName;

        DigitalDevices(String xmlTagName) {
            this.xmlTagName = xmlTagName;
        }

        public String getXmlTagName() {
            return xmlTagName;
        }
    }

    public enum PWMDevices implements XmlDevice {
        ;
        private final String xmlTagName;

        PWMDevices(String xmlTagName) {
            this.xmlTagName = xmlTagName;
        }

        public String getXmlTagName() {
            return xmlTagName;
        }
    }

    public enum AnalogInputDevices implements XmlDevice {
        ;
        private final String xmlTagName;

        AnalogInputDevices(String xmlTagName) {
            this.xmlTagName = xmlTagName;
        }

        public String getXmlTagName() {
            return xmlTagName;
        }
    }

    public enum I2CDevices implements XmlDevice {
        REV_DISTANCE_SENSOR(""); // TODO: check xml tag name
        private final String xmlTagName;

        I2CDevices(String xmlTagName) {
            this.xmlTagName = xmlTagName;
        }

        public String getXmlTagName() {
            return xmlTagName;
        }
    }

    public static class Device {
        private final String id;
        private final Enum deviceType;
        private final int port;
        private final int bus;
        private final HubType parentModule;
        private final String xmlTagName;

        public Device(String id, Enum deviceType, int port, int bus, String xmlTagName, HubType parentModule) {
            this.id = id;
            this.deviceType = deviceType;
            this.port = port;
            this.bus = bus;
            this.parentModule = parentModule;
            this.xmlTagName = xmlTagName;
        }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof Device)) {
                return false;
            }
            Device device = (Device) o;
            return id.equals(device.id) && deviceType.equals(device.deviceType)
                    && port == device.port
                    && bus == device.bus
                    && xmlTagName.equals(device.xmlTagName)
                    && parentModule.equals(device.parentModule);
        }
    }

    public static interface SetConfigurationName {
        ConfigureControlHub setConfigurationName(String name);
    }

    public static interface ConfigureControlHub {
        ConfigureExpansionHub configureControlHub(String name, int port);
    }

    public static interface ConfigureExpansionHub {
        Builder configureExpansionHub(String name, int port);
    }

    private static class XmlAttribute {
        private final String name;
        private final String content;

        private XmlAttribute(String name, Object content) {
            this.name = name;
            this.content = content.toString();
        }
    }

    public static class Builder implements SetConfigurationName, ConfigureControlHub, ConfigureExpansionHub {
        private final Set<Device> devices = new HashSet<>();
        private final Document xml;
        private String name = "";

        private Builder() {
            try {
                xml = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();
                xml.setXmlVersion("1.0");
                xml.setXmlStandalone(true);
                Element robot = xml.createElement("Robot");
                robot.setAttribute("type", "FirstInspires-FTC");
                xml.appendChild(robot);
            } catch (ParserConfigurationException e) {
                throw new RuntimeException("Could not construct Builder instance:\n" + e.getMessage());
            }
        }

        @Override
        public ConfigureControlHub setConfigurationName(String name) {
            name = name.trim();
            if (name.contains(" ")) {
                throw new IllegalArgumentException("Configuration name must not contain whitespace");
            }
            this.name = name;
            return this;
        }

//        Element usbDevice = xml.createElement("LynxUsbDevice");
//            usbDevice.setAttribute("name", name);
//            usbDevice.setAttribute("serialNumber", "(embedded)");
//            xml.appendChild(usbDevice);

        @Override
        public ConfigureExpansionHub configureControlHub(String name, int port) { // TODO: finish
            return this;
        }

        @Override
        public Builder configureExpansionHub(String name, int port) { // TODO: finish
            return this;
        }

        private void addHardwareDevice(String id, int port, XmlDevice deviceType, int bus, HubType parentModule) {
            devices.add(new Device(id, (Enum) deviceType, port, bus, deviceType.getXmlTagName(), parentModule));
        }

        public Builder addMotor(String id, int port, Motors motorType, HubType parentModule) {
            addHardwareDevice(id, port, motorType, 0, parentModule);
            return this;
        }

        public Builder addServo(String id, int port, Servos servoType, HubType parentModule) {
            addHardwareDevice(id, port, servoType, 0, parentModule);
            return this;
        }

        public Builder addDigitalDevice(String id, int port, DigitalDevices digitalDeviceType, HubType parentModule) {
            addHardwareDevice(id, port, digitalDeviceType, 0, parentModule);
            return this;
        }

        public Builder addPWMDevice(String id, int port, PWMDevices pwmDeviceType, HubType parentModule) {
            addHardwareDevice(id, port, pwmDeviceType, 0, parentModule);
            return this;
        }

        public Builder addAnalogInputDevice(String id, int port, AnalogInputDevices analogInputDeviceType, HubType parentModule) {
            addHardwareDevice(id, port, analogInputDeviceType, 0, parentModule);
            return this;
        }

        public Builder addI2CDevice(String id, int port, int bus, I2CDevices deviceType, HubType parentModule) {
            addHardwareDevice(id, port, deviceType, 0, parentModule);
            return this;
        }

        public Builder activate() {
            return this;
        }

        public HardwareConfiguration build() {
            devices.forEach((device) -> { // Check for devices with the same id
                devices.forEach((comparedDevice) -> {
                    if (comparedDevice != device && comparedDevice.id.equals(device.id)) {
                        throw new BadHardwareMapException(String.format("Found multiple devices with the same id:\n %s with id %s and %s with id %s", comparedDevice.deviceType.name(), comparedDevice.id, device.deviceType.name(), device.id));
                    }
                });
            });
            return new HardwareConfiguration(this);
        }
    }

    public static class BadHardwareMapException extends RuntimeException {
        public BadHardwareMapException(String message) {
            super(message);
        }
    }
}
