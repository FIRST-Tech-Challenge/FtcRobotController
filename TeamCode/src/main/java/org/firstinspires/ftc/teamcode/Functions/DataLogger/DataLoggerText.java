package org.firstinspires.ftc.teamcode.Functions.DataLogger;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.VoltageSensor;

//import org.apache.commons.csv.CSVFormat;
//import org.apache.commons.csv.CSVPrinter;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal.Pistol;
import org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem.PositionCalculator;
import org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem.SistemXY;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Date;


public class DataLoggerText {
    SistemXY systemXY;
    RotationDetector rotationDetector;
    VoltageSensor voltageSensor;
    Pistol pistol;
    Move move;
    PositionCalculator positionCalculator;
    String outputFileName;
    String outputFileName2;
    String[] headers;
    String className;

    /**
     * This method creates a new file with the name containing the current date, time and place where it's saved (Download folder).
     */
    public DataLoggerText(RotationDetector _rotationDetector, VoltageSensor _voltageSensor, Pistol _pistol, Move _move,
                          SistemXY _systemXY, PositionCalculator _positionCalculator, String _className){
        systemXY = _systemXY;
        rotationDetector = _rotationDetector;
        voltageSensor = _voltageSensor;
        positionCalculator = _positionCalculator;
        pistol = _pistol;
        move = _move;
        Date today = new Date();
        outputFileName = new String(today.toString()+"-simpleData"+".csv");
        outputFileName2 = new String("/Download/Data");
        headers = new String[] {"currentDirection", "pistolMotor", "voltage", "currentAnglePositive", "currentAngleRaw",
                "accelX", "accelY", "accelZ", "currentPosition", "sistemXYData", "accelData", "runtime", "className"};
        className=_className;
    }

    String[] currentDirection = new String[200];
    double[] pistolMotor = new double[200];
    double[] voltage = new double[200];
    double[] currentAnglePositive = new double[200];
    double[] currentAngleRaw = new double[200];
    double[] accelX = new double[200];
    double[] accelY = new double[200];
    double[] accelZ = new double[200];
    String[] currentPosition = new String[200];
    String[] sistemXYData = new String[200];
    String[] accelData = new String[200];
    double[] runtime = new double[200];

    int i = 0;

    /**
     * This method resets once there are 150 line of data code.
     * @param : _runtime
     * @return : (boolean) true - if the vectors are rested OR false - contrary
     */
    boolean AddLine(double _runtime){
        if(i>=150){

            currentDirection = new String[200];
            pistolMotor = new double[200];
            voltage = new double[200];
            currentAnglePositive = new double[200];
            currentAngleRaw = new double[200];
            accelX = new double[200];
            accelY = new double[200];
            accelZ = new double[200];
            currentPosition = new String[200];
            sistemXYData = new String[200];
            accelData = new String[200];
            runtime = new double[200];

            i = 0;
            return true;
        }
        else {
            currentDirection[i] = move.ReturnCurrentDirectionText();
            pistolMotor[i] = pistol.ReturnMotorSpeed();
            voltage[i] = voltageSensor.getVoltage();
            currentAnglePositive[i] = rotationDetector.ReturnPositiveRotation();
            currentAngleRaw[i] = rotationDetector.ReturnRotation();
            accelX[i] = positionCalculator.ReturnXAcc();
            accelY[i] = positionCalculator.ReturnYAcc();
            accelZ[i] = positionCalculator.ReturnZAcc();
            currentPosition[i] = positionCalculator.ReturnData();
            sistemXYData[i] = systemXY.TestData();
            accelData[i] = positionCalculator.ReturnAccelData();
            runtime[i] = _runtime;
            i++;
            return false;
        }
    }

    /**
     * This method creates directory and file if not found, writes the file once it has 148 lines data code.
     * @param : _runtime
     * @return : (String) error (if there's one) OR "Works well" (self explanatory)
     */
    public String WriteData(double _runtime){

        File directory = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        File file = new File(directory.getAbsolutePath()+"/"+outputFileName);
        String aux ="eroare";
        try {
            if(!directory.exists()) {
                directory.mkdir();
            }
            aux=directory.getCanonicalPath();
            if(!file.exists()) {
                file.createNewFile();
            }
        }
        catch (IOException e){
            return " First error: "+file.getAbsoluteFile()+"\n"+directory.getAbsolutePath()+"\n"+e.toString();
        }

        try {


            if(AddLine(_runtime)){
                Date today = new Date();
                outputFileName = new String(today.toString()+"-data"+".csv");
            }
            else {

                Writer writer = new FileWriter(file);
                //CSVPrinter csvPrinter = new CSVPrinter(writer, CSVFormat.DEFAULT.withHeader(headers));
                /*csvPrinter.printRecord(move.ReadMotor(1), move.ReadMotor(2), move.ReadMotor(3), move.ReadMotor(4),
                        pistol.ReturnMotorSpeed(), voltageSensor.getVoltage(), rotationDetector.ReturnPositiveRotation(), rotationDetector.ReturnRotation(),
                        positionCalculator.ReturnXAcc(), positionCalculator.ReturnYAcc(), positionCalculator.ReturnZAcc(), positionCalculator.ReturnX(),
                        positionCalculator.ReturnY(), _runtime);*/
                if (i >= 148)
                {
                    for (int j = 0; j <= i; j++)
                    {
//                        csvPrinter.printRecord(currentDirection[j],
//                                pistolMotor[j], voltage[j], currentAnglePositive[j], currentAngleRaw[j],
//                                accelX[j], accelY[j], accelZ[j], currentPosition[j],
//                                sistemXYData[j], accelData[j], runtime[j], className);
                    }
                }
                writer.close();
                //csvPrinter.close();
            }
            return " Works Well";

        }
        catch (IOException e) {
            return " Second error: \n"+aux+"\n"+e.toString();
        }
    }
}
