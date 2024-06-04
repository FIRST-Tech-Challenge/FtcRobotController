package org.rustlib.config;

import org.w3c.dom.Document;

import java.io.File;

public class PreferenceEditor {
    static final File preferences = new File("/data/data/com.qualcomm.ftcrobotcontroller/com.qualcomm.ftcrobotcontroller_preferences.xml");

    public static Document readPreferences() throws RuntimeException {
        return Loader.readXML(preferences);
    }

    public static void setNetworkPassword(String password) {

    }

    public static void setNetworkName(String networkName) {

    }
}
