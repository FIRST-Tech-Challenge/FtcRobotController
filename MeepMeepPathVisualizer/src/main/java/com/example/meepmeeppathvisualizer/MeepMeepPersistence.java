package com.example.meepmeeppathvisualizer;

import com.noahbres.meepmeep.MeepMeep;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Properties;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class MeepMeepPersistence {
    final String DEFAULT_FILE_PATH = "TeamCode/src/main/res/raw/meepmeep.properties";

    private final Properties properties;
    private final MeepMeep meepMeep;

    public MeepMeepPersistence(MeepMeep meepMeep) {
        this.properties = new Properties();
        this.meepMeep = meepMeep;

        reload();

        startPersistenceThread();

        Runtime.getRuntime().addShutdownHook(
                new Thread(this::save)
        );
    }

    private void startPersistenceThread() {
        Executors.newSingleThreadScheduledExecutor().schedule(
                (Runnable) this::save, 60L, TimeUnit.SECONDS
        );
    }

    public void save() {
        save(DEFAULT_FILE_PATH);
    }

    public void save(String path) {
        ensureFileExistence(path);

        properties.setProperty("windows_x", String.valueOf(meepMeep.getWindowFrame().getX()));
        properties.setProperty("windows_y", String.valueOf(meepMeep.getWindowFrame().getY()));

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(path))) {
            properties.store(writer, null);
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public void reload() {
        reload(DEFAULT_FILE_PATH);
    }

    public void reload(String path) {
        ensureFileExistence(path);

        try (BufferedReader reader = new BufferedReader(new FileReader(path))) {
            properties.load(reader);
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public void restore() {
        meepMeep.getWindowFrame().setLocation(
                Integer.parseInt(properties.getProperty("windows_x")),
                Integer.parseInt(properties.getProperty("windows_y"))
        );
    }

    @SuppressWarnings("ResultOfMethodCallIgnored")
    private void ensureFileExistence(String path) {
        try {
            new File(path).createNewFile();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}