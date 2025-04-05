package com.kalipsorobotics.utilities;

import android.util.Log;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Date;

public class KFileWriter {

    private String name;

    // Get the current date and time
    Date now = new Date();
    SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
    String formattedDateTime = formatter.format(now);

    BufferedWriter writer;


    public KFileWriter(String name) {
        this.name = name;

        try {
            writer = new BufferedWriter(new FileWriter(name + "—" + formattedDateTime));
        } catch (IOException ioException) {
            Log.d("IOException", "Caught IOException While Initializing");
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
