package com.kalipsorobotics.utilities;

import android.util.Log;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Date;
import java.util.Locale;

public class KFileWriter {

    // Get the current date and time
    Date now;
    SimpleDateFormat formatter;
    String formattedDateTime;

    BufferedWriter writer;


    public KFileWriter(String name) {
        now = new Date();
        formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US);
        formattedDateTime = formatter.format(now);
        String filePath = name + "—" + formattedDateTime + ".csv";
        Log.d("KFileWriter", "Creating file: " + filePath);
        try {
            writer = new BufferedWriter(new FileWriter(name + "—" + formattedDateTime + ".csv"));
        } catch (IOException ioException) {
            Log.e("IOException", "Failed Initializing File Writer For " + name + "—" + formattedDateTime + ".csv",
                    ioException);
            throw new RuntimeException("Failed to initialize KFileWriter for " + name + "—" + formattedDateTime + ".csv", ioException);
        }

    }

    public void writeLine(String string) {

        try {
            writer.write(string);
            writer.newLine();
        } catch (IOException ioException) {
            Log.d("IOException", "Caught IOException While Writing");
        }
    }

    public void close() {
        try {
            writer.close();
        } catch (IOException e) {
            Log.d("IOException", "Caught IOException While Closing");
        }
    }

    public BufferedWriter getWriter() {
        return writer;
    }
}
