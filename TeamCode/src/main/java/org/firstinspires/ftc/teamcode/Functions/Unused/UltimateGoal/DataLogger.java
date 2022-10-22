package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import android.os.Environment;

//import org.apache.commons.csv.CSVFormat;
//import org.apache.commons.csv.CSVPrinter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Date;

import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem.PositionCalculator;

import com.qualcomm.robotcore.hardware.VoltageSensor;

@Deprecated
public class DataLogger {

    RotationDetector rotationDetector;
    VoltageSensor voltageSensor;
    Pistol pistol;
    Move move;
    PositionCalculator positionCalculator;
    String outputFileName;
    String outputFileName2;
    String[] headers;
    String className;


    //creeaza un fisier nou cu numele datei si orei curente si locul unde se salveaza, adica in fisierul Download
    public DataLogger(RotationDetector _rotationDetector,
                      VoltageSensor _voltageSensor, Pistol _pistol, Move _move, PositionCalculator _positionCalculator, String _className){
        rotationDetector = _rotationDetector;
        voltageSensor = _voltageSensor;
        positionCalculator = _positionCalculator;
        pistol = _pistol;
        move = _move;

        Date today = new Date();
        outputFileName = new String(today.toString()+"-data"+".csv");
        outputFileName2 = new String("/Download/Data");
        headers = new String[] {"leftMotor", "rightMotor", "leftMotorBack", "rightMotorBack", "currentDirection", "pistolMotor",
                "voltage", "currentAnglePositive", "currentAngleRaw", "accelX", "accelY", "accelZ", "currentX", "currentY", "runtime", "className"};
        className=_className;
    }

    double[] leftMotor = new double[200];
    double[] rightMotor = new double[200];
    double[] leftMotorBack = new double[200];
    double[] rightMotorBack = new double[200];
    double[] currentDirection = new double[200];
    double[] pistolMotor = new double[200];
    double[] voltage = new double[200];
    double[] currentAnglePositive = new double[200];
    double[] currentAngleRaw = new double[200];
    double[] accelX = new double[200];
    double[] accelY = new double[200];
    double[] accelZ = new double[200];
    double[] currentX = new double[200];
    double[] currentY = new double[200];
    double[] runtime = new double[200];

    int i = 0;


    //returneaza true in cazul in care se reseteaza vectorii si false in caz contrariu
    //reseteaza odata ce are 150 linii de date
    boolean AddLine(double _runtime){
        if(i>=150){

            leftMotor = new double[200];
            rightMotor = new double[200];
            leftMotorBack = new double[200];
            rightMotorBack = new double[200];
            currentDirection = new double[200];
            pistolMotor = new double[200];
            voltage = new double[200];
            currentAnglePositive = new double[200];
            currentAngleRaw = new double[200];
            accelX = new double[200];
            accelY = new double[200];
            accelZ = new double[200];
            currentX = new double[200];
            currentY = new double[200];
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
            pistolMotor[i] = pistol.ReturnMotorSpeed();
            voltage[i] = voltageSensor.getVoltage();
            currentAnglePositive[i] = rotationDetector.ReturnPositiveRotation();
            currentAngleRaw[i] = rotationDetector.ReturnRotation();
            accelX[i] = positionCalculator.ReturnXAcc();
            accelY[i] = positionCalculator.ReturnYAcc();
            accelZ[i] = positionCalculator.ReturnZAcc();
            currentX[i] = positionCalculator.ReturnX();
            currentY[i] = positionCalculator.ReturnY();
            runtime[i] = _runtime;
            i++;
            return false;
        }
    }


    // returneaza textul erorii in cazul existentei unei erori, iar in caz contrariu returneaza textul "Works well"
    // creeaza directorul si fisierul daca nu sunt gasite
    // scrie fisierul odata ce are 148 de linii de date
    public String writeData(double _runtime){

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
//                        csvPrinter.printRecord(leftMotor[j], rightMotor[j], leftMotorBack[j], rightMotorBack[j], currentDirection[j],
//                                pistolMotor[j], voltage[j], currentAnglePositive[j], currentAngleRaw[j],
//                                accelX[j], accelY[j], accelZ[j], currentX[j],
//                                currentY[j], runtime[j], className);
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
