package org.firstinspires.ftc.teamcode.Functions.DataLogger;

import android.os.Environment;


import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Date;
import java.lang.StringBuilder;
import java.util.Calendar;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.AccelerationDetector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;

public class DataLoggerRemastered {
    private Writer writer;
    private StringBuffer lineBuffer;
    private StringBuilder lineRecord;
    private long msInitTime;
    private long nsInitTime;

    private static final String[] headerFields = new String[] {"runtime",
            "className", "leftMotor", "rightMotor", "leftMotorBack",
            "rightMotorBack", "currentDirection",  "voltage",
            "currentAnglePositive", "currentAngleRaw", "accelX", "accelY", "accelZ"};
    // private static final String CSV_HEADER = String.join(",", headerFields);
    private static final String CSV_HEADER2 = "runtime,className,leftMotor,rightMotor,leftMotorBack,rightMotorBack,currentDirection,voltage,currentAnglePositive,currentAngleRaw,accelX,accelY,accelZ";

    /* Movement variables */
    RotationDetector rotationDetector;
    AccelerationDetector accelerationDetector;
    Move move;

    /* Sensor variables */
    VoltageSensor voltageSensor;

    /* File variables */
    String dataLogFileName;
    String[] headers;
    String className;

    public DataLoggerRemastered(RotationDetector _rotationDetector,
                      VoltageSensor _voltageSensor,
                      Move _move,
                      AccelerationDetector _accelerationDetector,
                      String _className){
        Date today = new Date();
        Calendar cal= Calendar.getInstance();
        cal.setTime(today);
        int year =cal.get(Calendar.YEAR);
        int month =cal.get(Calendar.MONTH);
        int day =cal.get(Calendar.DAY_OF_MONTH);
        String YMD = year + "_" + month + "_" + day;
        dataLogFileName = new String(today.toString() + "_data" + ".csv");
        File directoryPath = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        File subDirectoryName = new File(directoryPath.getAbsolutePath() + "/" + YMD);
        File filePath = new File(directoryPath.getAbsolutePath() + "/" + YMD + "/" + dataLogFileName);

        String fileErrorMsg ="Missing directory";
        try {
            if(!directoryPath.exists()) {
                directoryPath.mkdir();
            }
            if(subDirectoryName.exists()) {
                subDirectoryName.mkdir();
            }
            fileErrorMsg = directoryPath.getCanonicalPath();
            if(!filePath.exists()) {
                filePath.createNewFile();
            }
        }
        catch (IOException e){
            System.out.println("-> First error: " + filePath.getAbsoluteFile() +
                    "\n" + fileErrorMsg + "\n" + e.toString());
        }
        fileErrorMsg = "File not found";
        try {
            writer = new FileWriter(filePath);
            lineBuffer = new StringBuffer(128);
           // addHeader(CSV_HEADER);

        }
        catch (IOException e) {
            System.out.println("-> Second error: " + filePath.getAbsoluteFile()
                    + "\n" + fileErrorMsg + "\n" + e.toString());
        }

        rotationDetector = _rotationDetector;
        voltageSensor = _voltageSensor;
        accelerationDetector = _accelerationDetector;
        move = _move;

        headers = new String[] {"runtime", "className", "leftMotor", "rightMotor",
                "leftMotorBack", "rightMotorBack", "currentDirection",
                "voltage", "currentAnglePositive", "currentAngleRaw",
                "accelX", "accelY", "accelZ"};
        className=_className;
    }

    private void flushLineBuffer(double _runtime){
        // long milliTime,nanoTime;

        try {
            lineBuffer.append('\n');
            writer.write(lineBuffer.toString());
            lineBuffer.setLength(0);
        }
        catch (IOException e){
            System.out.println("-> Flush error: " + e.toString());
        }
        /*
        milliTime   = System.currentTimeMillis();
        nanoTime    = System.nanoTime();
        addField(String.format(Locale.US, "%.3f",(milliTime - msInitTime) / 1.0E3));
        addField(String.format(Locale.US, "%.3f",(nanoTime - nsInitTime) / 1.0E6));
        nsInitTime = nanoTime;
        */

        addField(move.ReadMotor(1));
        addField(move.ReadMotor(2));
        addField(move.ReadMotor(3));
        addField(move.ReadMotor(4));
        addField(move.ReturnCurrentDirection());
        addField(voltageSensor.getVoltage());
        addField(rotationDetector.ReturnPositiveRotation());
        addField(rotationDetector.ReturnRotation());
        addField(accelerationDetector.rawAccel.x);
        addField(accelerationDetector.rawAccel.y);
        addField(accelerationDetector.rawAccel.z);
        /// addField(_runtime);
    }

    public void addHeader(String s){
        lineBuffer.append(s);
    }

    public void addField(String s) {
        if (lineRecord.length()>0) {
            lineRecord.append(',');
        }
        lineRecord.append(s);
    }

    public void addField(long l) {
        addField(Long.toString(l));
    }

    public void addField(float f) {
        addField(Float.toString(f));
    }

    public void addField(double d) {
        addField(Double.toString(d));
    }

    public void newLine(double _runtime) {
        flushLineBuffer(_runtime);
    }

}
