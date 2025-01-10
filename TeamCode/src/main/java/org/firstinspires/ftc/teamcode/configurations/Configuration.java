/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Generic configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

abstract public class Configuration {
    // Map to store hardware components by reference name
    protected final  Map<String, ConfMotor> mMotors = new LinkedHashMap<>();
    protected final  Map<String, ConfImu>   mImus   = new LinkedHashMap<>();
    protected final  Map<String, ConfServo> mServos = new LinkedHashMap<>();

    // FOR TUNING ONLY !!!
    protected final Map<String, ConfServo> mSingleServos = new LinkedHashMap<>();

    public static Configuration s_Current = new Test();

    // Method to retrieve a motor by its reference name
    public ConfMotor getMotor(String name) {
        if (mMotors.containsKey(name)) { return mMotors.get(name); }
        else                           { return null;               }
    }

    // Method to retrieve an imu by its reference name
    public ConfImu getImu(String name) {
        if (mImus.containsKey(name)) { return mImus.get(name); }
        else                         { return null;             }
    }

    // Method to retrieve a servo by its reference name
    public ConfServo getServo(String name) {
        if (mServos.containsKey(name)) { return mServos.get(name); }
        else                           { return null;            }
    }

    public Map<String, ConfServo>   getForTuning() { return mSingleServos; }

    // Method to retrieve a servo by its reference name
    public ConfServo getServoForTuning(String name) {
        if (mSingleServos.containsKey(name)) { return mSingleServos.get(name); }
        else                                 { return null;            }
    }

    // Abstract method for initializing specific configurations
    protected abstract void initialize();
    protected abstract void initializeTuning();

    // Constructor
    public Configuration() {
        initialize();
        initializeTuning();
    }
}
