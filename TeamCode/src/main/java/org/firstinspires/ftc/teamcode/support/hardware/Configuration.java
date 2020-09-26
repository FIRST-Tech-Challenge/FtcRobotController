package org.firstinspires.ftc.teamcode.support.hardware;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.support.Logger;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.lang.reflect.Method;
import java.util.Date;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * Maintains robot configuration including <code>HardwareMap</code> used for initialization
 *  of hardware devices and map of <code>Configurable</code> components / devices.
 * Every <code>Configurable</code> component should <code>register()</code> itself upon
 *  initialization in order to have its settings updated.<br />
 * @see Configuration#register(Configurable)
 * @see Configuration#apply()
 */
public class Configuration extends Logger<Configuration> {
    private HardwareMap hardwareMap;
    private String robotName;
    private Map<String, Configurable> components;
    private JsonObject adjustments;
    private Date lastModified;

    /**
     * Constructs a new configuration for given robot. Invoked from
     *  <code>LinearOpMode</code> immediately after robot creation
     *  and prior to calling <code>Robot.configure()</code>
     * @param hardwareMap map of hardware devices as provided to <code>LinearOpMode</code>
     * @param robotName unique robot name, usually a result of calling <code>getSimpleName()</code> on robot class
     */
    public Configuration(HardwareMap hardwareMap, String robotName) {
        this.robotName = robotName;
        this.hardwareMap = hardwareMap;
        components = new HashMap<>();
    }

    /**
     * Map containing hardware devices as configured in robot controller
     */
    public HardwareMap getHardwareMap() {
        return this.hardwareMap;
    }

    /**
     * Registers given <code>Configurable</code> component or device with
     *  this <code>Configuration</code> so it can receive setting adjustments
     * @param component
     */
    public synchronized void register(Configurable component) {
        Configurable existing = components.put(component.getUniqueName(), component);
        if (existing!= null && existing != component) {
            throw new IllegalArgumentException(
                    "Another Configurable named " + component.getUniqueName() + " was already registered"
            );
        }
    }

    /**
     * Returns a previously registered <code>Configurable</code> component or device
     *  associated with given name or <code>null</code> if none exists
     * @param name component or device name
     */
    public Configurable get(String name) {
        return components.get(name);
    }

    /**
     * Returns a list of names for registered <code>Configurable</code> components / devices
     */
    public Set<String> getComponentNames() {
        return components.keySet();
    }

    /**
     * Returns a map of <code>Adjustable</code> properties for given component or device.
     * Map keys are property names and values are instances of <code>Adjustable</code>
     *  annotations holding minimum / maximum / step parameters for each property.
     * @param component component or device to return adjustable properties for
     */
    public Map<String, Adjustable> getAdjustableProperties(Configurable component) {
        Map<String, Adjustable> map = new LinkedHashMap<>();
        for (Method method : component.getClass().getMethods()) {
            Adjustable setting = method.getAnnotation(Adjustable.class);
            if (setting==null || !Reflection.isGetter(method)) continue;
            map.put(Reflection.asProperty(method), setting);
        }
        debug("Configurable settings for %s: %s", component.getUniqueName(), map.keySet());
        return map;
    }

    /**
     * Loads and applies <code>Adjustable</code> properties to <code>Configurable</code>
     *  devices / components defined in this robot configuration from a JSON file.
     * File is named robotName.json where <code>robotName</code> was specified during
     *  construction of this configuration and is loaded from settings folder first
     *  or, should that fail, from application assets.
     * Settings folder is defined in <code>AppUtil.ROBOT_SETTINGS</code> and is usually
     *  located under <code></code>/sdcard/FIRST/settings/</code> on the phone.
     * @return <code>true</code> if adjustments were successfully loaded and applied
     * @see AppUtil#ROBOT_SETTINGS
     */
    public boolean apply() {
        info("Applying adjustments for %s", robotName);

        File file = AppUtil.getInstance().getSettingsFile(robotName + ".json");
        try (InputStream is = file.exists() ?
                new FileInputStream(file) : hardwareMap.appContext.getAssets().open(robotName + ".json")
        ) {
            adjustments = (JsonObject) new JsonParser().parse(new InputStreamReader(is, "UTF-8"));
            if (file.exists()) this.lastModified = new Date(file.lastModified());
        } catch (Exception E) {
            // could not read preferences
            warn("Unable to read %s or load asset: %s", file.getAbsolutePath(), E.getMessage(), E);
            return false;
        }

        for (Map.Entry<String, Configurable> deviceEntry : components.entrySet()) {
            JsonObject deviceAdjustments = adjustments.getAsJsonObject(deviceEntry.getKey());
            if (deviceAdjustments == null) continue;

            for (Map.Entry<String, JsonElement> deviceSetting : deviceAdjustments.entrySet()) {
                Configurable device = deviceEntry.getValue();
                try {
                    Reflection.set(device, deviceSetting.getKey(), deviceSetting.getValue().getAsDouble());
                } catch (Exception E) {
                    warn("Unable to apply %s.%s = %.2f: %s",
                            deviceEntry.getKey(), deviceSetting.getKey(),
                            deviceSetting.getValue().getAsDouble(), E.getMessage(), E
                    );
                }
            } // for (device adjustments)
        } // for (devices)
        return true;
    }

    /**
     * Stores <code>Adjustable</code> properties for <code>Configurable</code>
     *  devices / components defined in this robot configuration to a JSON file.
     * File is named robotName.json where <code>robotName</code> was specified during construction
     *  of this configuration and is located in <code>AppUtil.ROBOT_SETTINGS</code> folder.
     * @return <code>true</code> if adjustments were successfully stored
     * @see Configuration#apply()
     */
    public boolean storе() {
        for (Map.Entry<String, Configurable> deviceEntry : components.entrySet()) {
            JsonObject deviceAdjustments = adjustments==null ? null : adjustments.getAsJsonObject(deviceEntry.getKey());

            for(String name : getAdjustableProperties(deviceEntry.getValue()).keySet()) {
                try {
                    double value = ((Number) Reflection.get(deviceEntry.getValue(), name)).doubleValue();
                    if (deviceAdjustments == null) deviceAdjustments = new JsonObject();
                    deviceAdjustments.addProperty(name, value);
                } catch (Exception E) {
                    warn("Unable to retrieve %s.%s: %s",
                            deviceEntry.getKey(), name, E.getMessage(), E
                    );
                    continue;
                }
            }
            if (deviceAdjustments == null) continue;
            if (adjustments == null) adjustments = new JsonObject();
            adjustments.add(deviceEntry.getKey(), deviceAdjustments);
        }
        if (adjustments==null) return false;

        info("Saving adjustments to %s.json", robotName);
        File file = AppUtil.getInstance().getSettingsFile(robotName + ".json");
        try (OutputStreamWriter writer = new OutputStreamWriter(new FileOutputStream(file),"UTF-8")) {
            SimpleGson.getInstance().toJson(adjustments, writer);
            this.lastModified = new Date();
        } catch (Exception E) {
            warn("Unable to write %s: %s", file.getAbsolutePath(), E.getMessage(), E);
            return false;
        }
        info("Saved adjustments to %s", file.getAbsolutePath());
        return true;
    }

    /**
     * Returns last modification date / time for the configuration adjustments file stored on this phone
     *  or <code>null</code> if file does not exist and adjustments were loaded from app assets
     */
    public Date getLastModified() {
        return this.lastModified;
    }

    /**
     * Removes configuration adjustments file stored on this phone if it exists
     */
    public void delete() {
        if (this.lastModified!=null) {
            File file = AppUtil.getInstance().getSettingsFile(robotName + ".json");
            file.delete();
            this.lastModified = null;
        }
    }
}
