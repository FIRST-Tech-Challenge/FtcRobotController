package org.firstinspires.ftc.teamcode.Enhancement.Config;

import java.util.Hashtable;

/**
 * Abstract config superclass.
 */
public abstract class Config {

    Hashtable<String, String> getTable = new Hashtable<>();

    /**
     * Gets the value of key from a hashable.
     *
     * @param key the variable name
     * @return
     */
    public abstract Object get(String key);

    public abstract void set(String key, Object value);
}
