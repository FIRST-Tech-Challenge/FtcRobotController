package org.firstinspires.ftc.teamcode.utils.BT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BTRecordedController extends BTController {
    protected BufferedReader bufferedReader;
    protected double[] m_axes;
    protected boolean[] m_buttons;
    protected int iterationCnt;
    protected int maxIterations;
    protected boolean readErrorAccured;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public BTRecordedController(Gamepad gamepad, BufferedReader bufferedReader, int maxIterations) {
        super(gamepad);
        this.bufferedReader = bufferedReader;
        this.maxIterations = maxIterations;
        this.iterationCnt = 0;
        this.readErrorAccured=false;

        this.m_axes = new double[Axes.values().length];
        super.m_axesSuppliers = new DoubleSupplier[Axes.values().length];

        this.m_buttons = new boolean[Buttons.values().length];
        super.m_buttonsSuppliers = new BooleanSupplier[Buttons.values().length];

        for (int i = 0; i < Axes.values().length; i++) {
            int finalI = i;
            m_axesSuppliers[i] = () -> m_axes[finalI];
        }
        for (int i = 0; i < Buttons.values().length; i++) {
            int finalI = i;
            m_buttonsSuppliers[i] = () -> m_buttons[finalI];
        }

        //later this could be change to true and i want the user to see the change so i clear the data now.
        dashboardTelemetry.addData("readErrorAccured",false);

    }

    public BTRecordedController(Gamepad gamepad) throws Exception {
        super(gamepad);
        throw new Exception("not implemented, you must provide a file to read");
    }

    public void next()  {
        if ((iterationCnt < maxIterations) || readErrorAccured) {
            iterationCnt++;
            String[] line = new String[0];
            try {
                line = bufferedReader.readLine().split(",");
                m_axes = stringArrToDouble(line, Axes.values().length, 0, Axes.values().length);
                m_buttons = stringArrToBool(line, Buttons.values().length, Axes.values().length, Buttons.values().length + Axes.values().length);
            } catch (IOException e) {
                //we don't want the code to crash so we just log and then read only default values.
                readErrorAccured=true;
                dashboardTelemetry.addData("readErrorAccured",true);
                dashboardTelemetry.addLine("could not read next line. potentially configured more iterations then in recording. max configured iterations:"+maxIterations+" current iteration:"+iterationCnt+"\n"+e.toString());
            }
        } else {
            Arrays.fill(m_axes, 0);
            Arrays.fill(m_buttons,false);
            dashboardTelemetry.addLine("controller's recording finished. now using default values (0,false)");
        }
    }

    public static double[] stringArrToDouble(String[] arr, int len, int start, int stop) {
        double[] res = new double[len];
        for (int i = start; i < stop; i++) {
            res[i] = Double.parseDouble(arr[i]);
        }
        return res;
    }

    public static boolean[] stringArrToBool(String[] arr, int len, int start, int stop) {
        boolean[] res = new boolean[len];
        for (int i = start; i < stop; i++) {
            res[i - start] = arr[i].equals("1");
        }
        return res;
    }

}
