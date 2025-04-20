package com.kalipsorobotics.utilities;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class KFileWriter {

    OpModeUtilities opModeUtilities;
    // Get the current date and time
    Date now;
    SimpleDateFormat formatter;
    String formattedDateTime;

    String name;
    BufferedWriter writer;


    public KFileWriter(String name, OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.name = name;
        now = new Date();
        formatter = new SimpleDateFormat("yyyy_MM_dd__HH_mm_ss_SSS", Locale.US);
        formattedDateTime = formatter.format(now);

        File path = new File(opModeUtilities.getHardwareMap().appContext.getExternalFilesDir(null), "OdometryLog");
        if(!path.exists()) {
            if (!path.mkdirs()) {
                Log.d("KFileWriter", "Failed To Make Directory");
                throw new RuntimeException("Failed To Make Directory");
            }
        }

        File file = new File(path, name + "—" + formattedDateTime + ".csv");

        try {
            writer = new BufferedWriter(new FileWriter(file));
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
