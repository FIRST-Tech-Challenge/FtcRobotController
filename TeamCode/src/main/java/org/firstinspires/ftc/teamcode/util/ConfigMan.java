package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;

/** Describes configurable interfaces which have configurable options passed to the constructor and with built in methods. */
public interface ConfigMan {
    /** sets the entire configMan hash map */
    HashMap<String, String> configMan = new HashMap<String, String>();
    /** allows for the configuration to be reset to its initial values, called at every constructor instance, allowing for default values. */
    void resetConfiguration();

    void setConfig(HashMap<String, String> config);

    String getConfig(String key);
    String addConfigData(String key, String data);
    /** replaces original data at key to new data at key. If no key under that name is found, the key is appended to the configMan and an error is thrown.*/
    void replaceConfigKey(String key, String data) throws Exception;
}