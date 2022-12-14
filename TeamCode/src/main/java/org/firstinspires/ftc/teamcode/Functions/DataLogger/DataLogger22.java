package org.firstinspires.ftc.teamcode.Functions.DataLogger;

import android.os.Environment;



import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Date;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.AccelerationDetector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;


public class DataLogger22 {

    RotationDetector rotationDetector;
    VoltageSensor voltageSensor;
    AccelerationDetector accelerationDetector;
    Move move;
    String outputFileName;
    String outputFileName2;
    String[] headers;
    String className;

    /**
     * This method creates a new file with the name containing the current date, time and place where it's saved (Download folder).
     */
    public DataLogger22(RotationDetector _rotationDetector, VoltageSensor _voltageSensor, Move _move,
                        AccelerationDetector _accelerationDetector, String _className){
        rotationDetector = _rotationDetector;
        voltageSensor = _voltageSensor;
        accelerationDetector = _accelerationDetector;
        move = _move;
        Date today = new Date();
        outputFileName = new String(today.toString()+"-data"+".csv");
        outputFileName2 = new String("/Download/Data");
        headers = new String[] {"runtime", "className", "leftMotor", "rightMotor", "leftMotorBack", "rightMotorBack", "currentDirection",
                "voltage", "currentAnglePositive", "currentAngleRaw", "accelX", "accelY", "accelZ"};
        className=_className;
    }
    double[] runtime = new double[200];
    double[] leftMotor = new double[200];
    double[] rightMotor = new double[200];
    double[] leftMotorBack = new double[200];
    double[] rightMotorBack = new double[200];
    double[] currentDirection = new double[200];
    double[] voltage = new double[200];
    double[] currentAnglePositive = new double[200];
    double[] currentAngleRaw = new double[200];
    double[] accelX = new double[200];
    double[] accelY = new double[200];
    double[] accelZ = new double[200];


    int i = 0;


    /**
     * This method resets once there are 150 line of data code.
     * @param : _runtime
     * @return : (boolean) true - if the vectors are rested OR false - contrary
     */
    boolean AddLine(double _runtime){
        if(i>=150){
            leftMotor = new double[200];
            rightMotor = new double[200];
            leftMotorBack = new double[200];
            rightMotorBack = new double[200];
            currentDirection = new double[200];
            voltage = new double[200];
            currentAnglePositive = new double[200];
            currentAngleRaw = new double[200];
            accelX = new double[200];
            accelY = new double[200];
            accelZ = new double[200];
            runtime = new double[200];

            i = 0;
            return true;
        }
        else {
            leftMotor[i] = move.ReadMotor(1);
            rightMotor[i] = move.ReadMotor(2);
            leftMotorBack[i] = move.ReadMotor(3);
            rightMotorBack[i] = move.ReadMotor(4);
            currentDirection[i] = move.ReturnCurrentDirection();
            voltage[i] = voltageSensor.getVoltage();
            currentAnglePositive[i] = rotationDetector.ReturnPositiveRotation();
            currentAngleRaw[i] = rotationDetector.ReturnRotation();
            accelX[i] = accelerationDetector.rawAccel.x;
            accelY[i] = accelerationDetector.rawAccel.y;
            accelZ[i] = accelerationDetector.rawAccel.z;
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
                outputFileName = new String(today.toString()+"-data22"+".csv");
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
                        /*
                        "runtime", "className", "leftMotor", "rightMotor", "leftMotorBack", "rightMotorBack", "currentDirection",
                "voltage", "currentAnglePositive", "currentAngleRaw", "accelX", "accelY", "accelZ"
                         */
                        //csvPrinter.printRecord(className, runtime[j], leftMotor[j], rightMotor[j], leftMotorBack[j], rightMotorBack[j], currentDirection[j],
                                //voltage[j], currentAnglePositive[j], currentAngleRaw[j], accelX[j], accelY[j], accelZ[j];
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
