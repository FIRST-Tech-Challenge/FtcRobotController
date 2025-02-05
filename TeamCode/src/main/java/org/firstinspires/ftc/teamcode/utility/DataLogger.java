package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.teamcode.constants.Constants.FileConstants.*;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

public final class DataLogger {
    private final ArrayList<String> data;
    private String logFileName;

    public DataLogger(String fileName) {
        this.logFileName = fileName;
        data = new ArrayList<>();
    }

    public void addLine(String line) {
        data.add(line);
    }

    public void clear() {
        String heading = data.get(0);
        data.clear();
        data.add(heading);
    }

    public void save() {
        String currentDate =
                new SimpleDateFormat("yyyyMMdd", Locale.CANADA).format(new Date());
        String currentTime =
                new SimpleDateFormat("HHmmss", Locale.CANADA).format(new Date());

        logFileName += currentDate + "_" + currentTime + ".txt";

        String pathToLogFile = SD_CARD_PATH + logFileName;

        try (FileWriter fileWriter = new FileWriter(pathToLogFile)) {
            for (String data : data) { fileWriter.write(data); }

            fileWriter.flush();
        } catch (IOException ignored) {}
    }
}
