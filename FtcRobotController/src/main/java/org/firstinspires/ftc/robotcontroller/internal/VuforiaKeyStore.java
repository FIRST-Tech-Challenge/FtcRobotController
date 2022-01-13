package org.firstinspires.ftc.robotcontroller.internal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;

public class VuforiaKeyStore {
    private static final VuforiaKeyStore instance;
    static { instance = new VuforiaKeyStore(); }

    public static VuforiaKeyStore getInstance() {
        return instance;
    }

    private static final String FILE_PATH = "/sdcard/FIRST/vuforiakey.txt";
    private static final int VUFORIA_KEY_LENGTH = 380;

    private static File getFile() { return new File(FILE_PATH); }

    private VuforiaKeyStore() {
        loadKeyFromFile();
    }

    private String vuforiaKey = "";
    public String getVuforiaKey() { return vuforiaKey; }

    private boolean loadKeyFromFile() {
        File file = getFile();
        try {
            byte[] fileBytes = new byte[VUFORIA_KEY_LENGTH];
            int bytesRead = new FileInputStream(file).read(fileBytes);
            if (bytesRead < VUFORIA_KEY_LENGTH) return false;
            this.vuforiaKey = new String(fileBytes);
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public void updateKey(String newKey) {
        // Validate newKey first
        if (newKey.length() < 380) return;

        File file = getFile();
        try {
            if (!file.exists()) file.createNewFile();
            new FileOutputStream(file).write(newKey.getBytes());
            this.vuforiaKey = newKey;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    void attachToFtcDashboard() {
        FtcDashboard.getInstance().addConfigVariable(
                this.getClass().getSimpleName(),
                "vuforiaKey",
                new ValueProvider<String>() {
                    @Override
                    public String get() {
                        return vuforiaKey;
                    }

                    @Override
                    public void set(String value) {
                        updateKey(value);
                    }
                },
                false
        );
    }


}
