package org.firstinspires.ftc.teamcode.utils.BT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

/**
 * this class uses records game controller values (buttons and joysticks) each  time addRecord() is called.
 * the recording is written to the provided log file when done() is called. this should only be called once.
 * this class assumes that the cycle time of the main loop is consistent across runs.
 * maxIterations should be loopFrequency[Hz]*RecordLength[Sec]
 */
public class BTRecordingController extends BTController{

    ArrayList<double[]> axesLines;
    ArrayList<boolean[]> buttonsLines;
    protected int iterationCnt;
    protected int maxIterations;
    protected FileWriter fileWriter;
    private final Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    public BTRecordingController(Gamepad gamepad, File log, int maxIterations) throws IOException {
        super(gamepad);
        axesLines= new ArrayList<>(maxIterations);
        buttonsLines= new ArrayList<>(maxIterations);
        if(log.isDirectory()) {
            throw new IOException("a directory was provided instead of a file");
        }
        if(!log.exists()){
            boolean created=log.createNewFile();
            if(!created){
                throw new IOException("could not create the file: \""+log.getName()+"\" at path: "+log.getParent());
            }
        }
        fileWriter = new FileWriter(log);
    }

    public void addRecord(){
        axesLines.add(Arrays.stream(m_axesSuppliers).mapToDouble(DoubleSupplier::getAsDouble).toArray());
        boolean[] buttons=new boolean[m_buttonsSuppliers.length];
        for (int i = 0; i < buttons.length; i++) {
            buttons[i]=m_buttonsSuppliers[i].getAsBoolean();
        }
        buttonsLines.add(buttons.clone());
        iterationCnt++;
    }

    public void done()  {
        StringBuilder s= new StringBuilder();
        for (int i = 0; i < axesLines.size(); i++) {
            s.append(doubleArrToString(axesLines.get(i)));
            s.append(boolArrToString(buttonsLines.get(i)));
            s.append("\n");
        }
        if(iterationCnt>maxIterations){
            dashboardTelemetry.addLine("the number recorded controller inputs iterations exceeded expectation. iteration count is: "+iterationCnt+" expected: "+maxIterations);
        }
        dashboardTelemetry.addData("recorded iterations:",iterationCnt);

        try {
            fileWriter.write(s.toString());
            fileWriter.close();
        } catch (Exception e){
            dashboardTelemetry.addLine(e.toString());
        }
    }


    public static String boolArrToString(boolean[] arr){
        StringBuilder s= new StringBuilder(arr.length*2);
        for (int i = 0; i < arr.length; i++) {
            s.append(arr[i] ? "1," : "0,");
        }
        return s.toString();
    }

    public static String doubleArrToString(double[] arr){
        StringBuilder s= new StringBuilder(arr.length*16);
        for (int i = 0; i < arr.length; i++) {
            s.append(String.valueOf(arr[i]));
            s.append(",");
        }
        return s.toString();
    }
}
