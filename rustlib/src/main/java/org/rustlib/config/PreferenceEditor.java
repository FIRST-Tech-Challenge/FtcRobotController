package org.rustlib.config;

import org.firstinspires.ftc.robotcore.internal.network.InvalidNetworkSettingException;
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.rustlib.utils.FileUtils;
import org.w3c.dom.Document;

import java.io.File;
import java.io.IOException;

public class PreferenceEditor {
    private static final File preferences = new File("/data/data/com.qualcomm.ftcrobotcontroller/com.qualcomm.ftcrobotcontroller_preferences.xml");

    public static Document readPreferences() throws IOException {
        return FileUtils.readXML(preferences);
    }

    public static String readPreferencesAsString() throws IOException {
        return FileUtils.readString(preferences);
    }

    public static void configureNetwork(String name, String password) {
        try {
            NetworkConnectionHandler.getInstance().getNetworkConnection().setNetworkSettings(name, password, null);
        } catch (InvalidNetworkSettingException e) {
            throw new RuntimeException(e);
        }
    }

    public static void configureNetworkName(String name) {
        configureNetwork(name, null);
    }

    public static void configureNetworkPassword(String password) {
        configureNetwork(null, password);
    }
}
