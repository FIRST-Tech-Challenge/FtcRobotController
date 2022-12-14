package org.firstinspires.ftc.teamcode.Functions.DataLogger;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//import org.apache.commons.csv.CSVFormat;
//import org.apache.commons.csv.CSVPrinter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Date;

/**
 * This class is used for the input and export of the buttons pressed by the drivers (debug).
 */

/**
 *  USAGE OF THIS CLASS
 *  1. Initialise class with main name class (ex: initialising in main3, means you'll use main3), then gamepad1 and gamepad2
 *  2. Declare a private variable (private ElapsedTime runtime = new ElapsedTime()), then runtime.reset();
 *  runtime.startTime(); in void start
 *  3. Move WriteData(runtime) in void Loop()
 *  4. Move WriteEnd(runtime) in void Stop()
 *  5. In Init() (after initialisation) or in Loop() (before WriteData()) for every button - what it does
 *  (ex: dataLoggerController.functions.gamepad1a="robotul coboara bratul 1")
 *  LAST STEP is important if we want to recreate matches in UNITY OR DIRECTLY ON THE ROBOT.
 */
@Disabled
public class DataLoggerController {

    Gamepad gamepad1;
    Gamepad gamepad2;
    VoltageSensor voltageSensor;
    String outputFileName;
    String outputFileName2;
    String[] headers;
    String className;
    int fileNumber =0;


    public class Functions{
        public String gamepad1A ="", gamepad1B ="", gamepad1X ="", gamepad1Y ="";
        public String gamepad1RightStickX ="", gamepad1RightStickY ="", gamepad1LeftStickX ="", gamepad1LeftStickY ="", gamepad1RightStickButton ="";
        public String gamepad1DpadUp ="", gamepad1DpadDown ="", gamepad1DpadLeft ="", gamepad1DpadRight ="";
        public String gamepad1LeftBumper ="", gamepad1LeftTrigger ="", gamepad1RightBumper ="", gamepad1RightTrigger ="";
        public String gamepad2A ="", gamepad2B ="", gamepad2X ="", gamepad2Y ="";
        public String gamepad2RightStickX ="", gamepad2RightStickY ="", gamepad2LeftStickX ="", gamepad2LeftStickY ="", gamepad2RightStickButton ="";
        public String gamepad2DpadUp ="", gamepad2DpadDown ="", gamepad2DpadLeft ="", gamepad2DpadRight ="";
        public String gamepad2LeftBumper ="", gamepad2LeftTrigger ="", gamepad2RightBumper ="", gamepad2RightTrigger ="";
    }

    public Functions functions = new Functions();

    void DeclareHeaders(){
        headers = new String[] {"classname","runtime", "voltage", "gamepad1.a-"+functions.gamepad1A, "gamepad1.b-"+functions.gamepad1B, "gamepad1.x-"+functions.gamepad1X, "gamepad1.y-"+functions.gamepad1Y,
                "gamepad1.right_stick_x-"+functions.gamepad1RightStickX, "gamepad1.right_stick_y-"+functions.gamepad1RightStickY,
                "gamepad1.left_stick_x-"+functions.gamepad1LeftStickX, "gamepad1.left_stick_y-"+functions.gamepad1LeftStickY, "gamepad1.right_stick_button-"+functions.gamepad1RightStickButton,
                "gamepad1.dpad_up-"+functions.gamepad1DpadUp, "gamepad1.dpad_down-"+functions.gamepad1DpadDown, "gamepad1.dpad_left-"+functions.gamepad1DpadLeft,
                "gamepad1.dpad_right-"+functions.gamepad1DpadRight, "gamepad1.left_bumper-"+functions.gamepad1LeftBumper, "gamepad1.left_trigger-"+ gamepad1LeftTrigger,
                "gamepad1.right_bumper-"+ gamepad1RightBumper, "gamepad1.right_trigger-"+ gamepad1RightTrigger,

                "gamepad2.a-"+functions.gamepad2A, "gamepad2.b-"+functions.gamepad2B, "gamepad2.x-"+functions.gamepad2X, "gamepad2.y-"+functions.gamepad2Y,
                "gamepad2.right_stick_x-"+functions.gamepad2RightStickX, "gamepad2.right_stick_y-"+functions.gamepad2RightStickY,
                "gamepad2.left_stick_x-"+functions.gamepad2LeftStickX, "gamepad2.left_stick_y-"+functions.gamepad2LeftStickY, "gamepad2.right_stick_button-"+functions.gamepad2RightStickButton,
                "gamepad2.dpad_up-"+functions.gamepad2DpadUp, "gamepad2.dpad_down-"+functions.gamepad2DpadDown, "gamepad2.dpad_left-"+functions.gamepad2DpadLeft,
                "gamepad2.dpad_right-"+functions.gamepad2DpadRight, "gamepad2.left_bumper-"+functions.gamepad2LeftBumper, "gamepad2.left_trigger-"+ gamepad2LeftTrigger,
                "gamepad2.right_bumper-"+ gamepad2RightBumper, "gamepad2.right_trigger-"+ gamepad2RightTrigger,
        };
    }

    /**
     * This method creates a new file with the name containing the current date, time and place where it's saved (Download/Controller folder).
     */
    public DataLoggerController(String _className, Gamepad _gamepad1, Gamepad _gamepad2, VoltageSensor _voltageSensor){

        voltageSensor = _voltageSensor;
        gamepad1 = _gamepad1;
        gamepad2 = _gamepad2;
        Date today = new Date();
        outputFileName = new String(today.toString()+"-"+_className+"-controller-"+ fileNumber +".csv");
        outputFileName2 = new String("/Download/Data/Controller");
        DeclareHeaders();
        className=_className;
    }

    double[] runtime = new double[150];
    double[] voltage = new double[150];

    boolean[] gamepad1A = new boolean[150];
    boolean[] gamepad1B = new boolean[150];
    boolean[] gamepad1X = new boolean[150];
    boolean[] gamepad1Y = new boolean[150];

    double[] gamepad1RightStickX = new double[150];
    double[] gamepad1RightStickY = new double[150];
    double[] gamepad1LeftStickX = new double[150];
    double[] gamepad1LeftStickY = new double[150];
    boolean[] gamepad1RightStickButton = new boolean[150];

    boolean[] gamepad1DpadUp = new boolean[150];
    boolean[] gamepad1DpadDown = new boolean[150];
    boolean[] gamepad1DpadLeft = new boolean[150];
    boolean[] gamepad1DpadRight = new boolean[150];

    boolean[] gamepad1LeftBumper = new boolean[150];
    double[] gamepad1LeftTrigger = new double[150];
    boolean[] gamepad1RightBumper = new boolean[150];
    double[] gamepad1RightTrigger = new double[150];

    boolean[] gamepad2A = new boolean[150];
    boolean[] gamepad2B = new boolean[150];
    boolean[] gamepad2X = new boolean[150];
    boolean[] gamepad2Y = new boolean[150];

    double[] gamepad2RightStickX = new double[150];
    double[] gamepad2RightStickY = new double[150];
    double[] gamepad2LeftStickX = new double[150];
    double[] gamepad2LeftStickY = new double[150];
    boolean[] gamepad2RightStickButton = new boolean[150];

    boolean[] gamepad2DpadUp = new boolean[150];
    boolean[] gamepad2DpadDown = new boolean[150];
    boolean[] gamepad2DpadLeft = new boolean[150];
    boolean[] gamepad2DpadRight = new boolean[150];

    boolean[] gamepad2LeftBumper = new boolean[150];
    double[] gamepad2LeftTrigger = new double[150];
    boolean[] gamepad2RightBumper = new boolean[150];
    double[] gamepad2RightTrigger = new double[150];

    int i = 0;

    /**
     * This method resets once there are 100 line of data code.
     * @param : _runtime
     * @return : (boolean) true - if the vectors are reseted OR false - contrary
     */
    boolean AddLine(double _runtime){
        if(i>=100){

            runtime = new double[150];
            voltage = new double[150];

            gamepad1A = new boolean[150];
            gamepad1B = new boolean[150];
            gamepad1X = new boolean[150];
            gamepad1Y = new boolean[150];

            gamepad1RightStickX = new double[150];
            gamepad1RightStickY = new double[150];
            gamepad1LeftStickX = new double[150];
            gamepad1LeftStickY = new double[150];
            gamepad1RightStickButton = new boolean[150];

            gamepad1DpadUp = new boolean[150];
            gamepad1DpadDown = new boolean[150];
            gamepad1DpadLeft = new boolean[150];
            gamepad1DpadRight = new boolean[150];

            gamepad1LeftBumper = new boolean[150];
            gamepad1LeftTrigger = new double[150];
            gamepad1RightBumper = new boolean[150];
            gamepad1RightTrigger = new double[150];

            gamepad2A = new boolean[150];
            gamepad2B = new boolean[150];
            gamepad2X = new boolean[150];
            gamepad2Y = new boolean[150];

            gamepad2RightStickX = new double[150];
            gamepad2RightStickY = new double[150];
            gamepad2LeftStickX = new double[150];
            gamepad2LeftStickY = new double[150];
            gamepad2RightStickButton = new boolean[150];

            gamepad2DpadUp = new boolean[150];
            gamepad2DpadDown = new boolean[150];
            gamepad2DpadLeft = new boolean[150];
            gamepad2DpadRight = new boolean[150];

            gamepad2LeftBumper = new boolean[150];
            gamepad2LeftTrigger = new double[150];
            gamepad2RightBumper = new boolean[150];
            gamepad2RightTrigger = new double[150];

            i = 0;

            runtime[i] = _runtime;
            voltage[i] = voltageSensor.getVoltage();;

            gamepad1A[i] = gamepad1.a;
            gamepad1B[i] = gamepad1.b;
            gamepad1X[i] = gamepad1.x;
            gamepad1Y[i] = gamepad1.y;

            gamepad1RightStickX[i] = gamepad1.right_stick_x;
            gamepad1RightStickY[i] = gamepad1.right_stick_y;
            gamepad1LeftStickX[i] = gamepad1.left_stick_x;
            gamepad1LeftStickY[i] = gamepad1.left_stick_y;
            gamepad1RightStickButton[i] = gamepad1.right_stick_button;

            gamepad1DpadUp[i] = gamepad1.dpad_up;
            gamepad1DpadDown[i] = gamepad1.dpad_down;
            gamepad1DpadLeft[i] = gamepad1.dpad_left;
            gamepad1DpadRight[i] = gamepad1.dpad_right;

            gamepad1LeftBumper[i] = gamepad1.left_bumper;
            gamepad1LeftTrigger[i] = gamepad1.left_trigger;
            gamepad1RightBumper[i] = gamepad1.right_bumper;
            gamepad1RightTrigger[i] = gamepad1.right_trigger;

            gamepad2A[i] = gamepad2.a;
            gamepad2B[i] = gamepad2.b;
            gamepad2X[i] = gamepad2.x;
            gamepad2Y[i] = gamepad2.y;

            gamepad2RightStickX[i] = gamepad2.right_stick_x;
            gamepad2RightStickY[i] = gamepad2.right_stick_y;
            gamepad2LeftStickX[i] = gamepad2.left_stick_x;
            gamepad2LeftStickY[i] = gamepad2.left_stick_y;
            gamepad2RightStickButton[i] = gamepad2.right_stick_button;

            gamepad2DpadUp[i] = gamepad2.dpad_up;
            gamepad2DpadDown[i] = gamepad2.dpad_down;
            gamepad2DpadLeft[i] = gamepad2.dpad_left;
            gamepad2DpadRight[i] = gamepad2.dpad_right;

            gamepad2LeftBumper[i] = gamepad2.left_bumper;
            gamepad2LeftTrigger[i] = gamepad2.left_trigger;
            gamepad2RightBumper[i] = gamepad2.right_bumper;
            gamepad2RightTrigger[i] = gamepad2.right_trigger;

            i = 1;
            return true;
        }
        else {

            runtime[i] = _runtime;
            voltage[i] = voltageSensor.getVoltage();;

            gamepad1A[i] = gamepad1.a;
            gamepad1B[i] = gamepad1.b;
            gamepad1X[i] = gamepad1.x;
            gamepad1Y[i] = gamepad1.y;

            gamepad1RightStickX[i] = gamepad1.right_stick_x;
            gamepad1RightStickY[i] = gamepad1.right_stick_y;
            gamepad1LeftStickX[i] = gamepad1.left_stick_x;
            gamepad1LeftStickY[i] = gamepad1.left_stick_y;
            gamepad1RightStickButton[i] = gamepad1.right_stick_button;

            gamepad1DpadUp[i] = gamepad1.dpad_up;
            gamepad1DpadDown[i] = gamepad1.dpad_down;
            gamepad1DpadLeft[i] = gamepad1.dpad_left;
            gamepad1DpadRight[i] = gamepad1.dpad_right;

            gamepad1LeftBumper[i] = gamepad1.left_bumper;
            gamepad1LeftTrigger[i] = gamepad1.left_trigger;
            gamepad1RightBumper[i] = gamepad1.right_bumper;
            gamepad1RightTrigger[i] = gamepad1.right_trigger;

            gamepad2A[i] = gamepad2.a;
            gamepad2B[i] = gamepad2.b;
            gamepad2X[i] = gamepad2.x;
            gamepad2Y[i] = gamepad2.y;

            gamepad2RightStickX[i] = gamepad2.right_stick_x;
            gamepad2RightStickY[i] = gamepad2.right_stick_y;
            gamepad2LeftStickX[i] = gamepad2.left_stick_x;
            gamepad2LeftStickY[i] = gamepad2.left_stick_y;
            gamepad2RightStickButton[i] = gamepad2.right_stick_button;

            gamepad2DpadUp[i] = gamepad2.dpad_up;
            gamepad2DpadDown[i] = gamepad2.dpad_down;
            gamepad2DpadLeft[i] = gamepad2.dpad_left;
            gamepad2DpadRight[i] = gamepad2.dpad_right;

            gamepad2LeftBumper[i] = gamepad2.left_bumper;
            gamepad2LeftTrigger[i] = gamepad2.left_trigger;
            gamepad2RightBumper[i] = gamepad2.right_bumper;
            gamepad2RightTrigger[i] = gamepad2.right_trigger;
            i++;
            return false;
        }
    }

    /**
     * This method creates directory and file if not found, writes the file once it has 98 lines data code.
     * @param : _runtime
     * @return : (String) error (if there's one) OR "Works well" (self explanatory)
     */
    public String WriteData(double _runtime){
        File directory = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS);
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
                fileNumber++;
                outputFileName = new String(today.toString()+"-"+className+"-controller-"+ fileNumber +".csv");
            }
            else {
                Writer writer = new FileWriter(file);
                DeclareHeaders();
                //CSVPrinter csvPrinter = new CSVPrinter(writer, CSVFormat.DEFAULT.withHeader(headers));
                if (i >= 98)
                {
                    for (int j = 0; j <= i; j++)
                    {
//                        csvPrinter.printRecord(className, voltage[i], runtime[j], gamepad1A[j], gamepad1B[j], gamepad1X[j], gamepad1Y[j], gamepad1A[j], gamepad1RightStickX[j],
//                                gamepad1RightStickX[j], gamepad1RightStickY[j], gamepad1LeftStickX[j], gamepad1LeftStickY[j], gamepad1RightStickButton[j],
//                                gamepad1DpadUp[j], gamepad1DpadDown[j], gamepad1DpadLeft[j], gamepad1DpadRight[j],
//                                gamepad1LeftBumper[j], gamepad1LeftTrigger[j], gamepad1RightBumper[j], gamepad1RightTrigger[j],
//                                gamepad2A[j], gamepad2B[j], gamepad2X[j], gamepad2Y[j],
//                                gamepad2RightStickX[j], gamepad2RightStickY[j], gamepad2LeftStickX[j], gamepad2LeftStickY[j], gamepad2RightStickButton[j],
//                                gamepad2DpadUp[j], gamepad2DpadDown[j], gamepad2DpadLeft[j], gamepad2DpadRight[j],
//                                gamepad2LeftBumper[j], gamepad2LeftTrigger[j], gamepad2RightBumper[j], gamepad2RightTrigger[j]);
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


    // E important sa punem in functia STOP() asta ca sa nu pierdem date
    //INTREABA DURDEU
    /**
     *
     * @param _runtime
     * @return
     */
    public String WriteEnd(double _runtime){
        File directory = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS);
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
            if(!AddLine(_runtime)){
                Writer writer = new FileWriter(file);
                DeclareHeaders();
                //CSVPrinter csvPrinter = new CSVPrinter(writer, CSVFormat.DEFAULT.withHeader(headers));
                for (int j = 0; j <= i; j++)
                {
//                    csvPrinter.printRecord(className, runtime[j], gamepad1A[j], gamepad1B[j], gamepad1X[j], gamepad1Y[j], gamepad1A[j], gamepad1RightStickX[j],
//                            gamepad1RightStickX[j], gamepad1RightStickY[j], gamepad1LeftStickX[j], gamepad1LeftStickY[j], gamepad1RightStickButton[j],
//                            gamepad1DpadUp[j], gamepad1DpadDown[j], gamepad1DpadLeft[j], gamepad1DpadRight[j],
//                            gamepad1LeftBumper[j], gamepad1LeftTrigger[j], gamepad1RightBumper[j], gamepad1RightTrigger[j],
//                            gamepad2A[j], gamepad2B[j], gamepad2X[j], gamepad2Y[j],
//                            gamepad2RightStickX[j], gamepad2RightStickY[j], gamepad2LeftStickX[j], gamepad2LeftStickY[j], gamepad2RightStickButton[j],
//                            gamepad2DpadUp[j], gamepad2DpadDown[j], gamepad2DpadLeft[j], gamepad2DpadRight[j],
//                            gamepad2LeftBumper[j], gamepad2LeftTrigger[j], gamepad2RightBumper[j], gamepad2RightTrigger[j]);
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
