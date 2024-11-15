package org.firstinspires.ftc.teamcode.subsystems.swerve;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.apache.commons.lang3.ArrayUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class voltageToAngleConstants {
    // (Angle, Voltage)
    /* TODO: Something is very wrong with filewriter it's not putting it on the control hub, it's putting it straight in this repository
     */
    double smallToBigPulley = (double) 15 / 38; // small:big --> small deg to big deg
    int[] rotations; // full small pulley rotations, added to how much degrees of current rotation
    ArrayList<AnalogInput> encoders = new ArrayList<>(); // list of objects, comes from user getting hardware map inputs
    public String logFilePath = String.format("%s/FIRST/wheelAngles.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    public File dataLog = AppUtil.getInstance().getSettingsFile(logFilePath);
    double[] voltages; // these are from the analog inputs
    public double[] sm; // small pulley angle values NO ROTATIONS
    double[] lastSm; //last small pulley angle for counting up/down rotations
    double[] angle; // big pulley angle values
    double[] differenceMs;
    public double[] offsets; // the offsets added to it to tell it where's zero, depends every reset, fix it.
    String[] offsetStrings;
    double x1class;
    double x1classR;
    String[] lastAngStringsWriting; // last angle, but as strings
    String[] smallAngString;
    String[] smallRotString;
    String[] valuesReading = null; // used for reading at the beginning of opmodes
    String lastLineValue = null;
    String fileDataRaw;
    public String[] writingFinal;
    OpMode opMode; // for telemetry when done reading
    public voltageToAngleConstants(OpMode opMode, HardwareMap hw, String[] encoderNames) {
        this.opMode = opMode;


        for (String encoderName : encoderNames) {
            encoders.add(hw.get(AnalogInput.class, encoderName));
        }
        voltages = new double[encoderNames.length];
        lastSm = new double[encoderNames.length];
        differenceMs = new double[encoderNames.length];
        offsets = new double[encoderNames.length];
//        lastVoltage = new double[encoderNames.length];

        lastAngStringsWriting = new String[encoderNames.length];
        smallAngString = new String[encoderNames.length];
        smallRotString = new String[encoderNames.length];
        offsetStrings = new String[encoderNames.length];

        fileDataRaw = ReadWriteFile.readFile(dataLog);
    }
    public void init_loop() {
        // Supposed to check for the last value in the csv file for most recent rotations
        // TODO: check this very much
        // Also, what if we just rewrote over and over on the same line? how?
        lastLineValue = fileDataRaw;
        lastLineValue = lastLineValue.replace("[", "");
        lastLineValue = lastLineValue.replace("]", "");
        valuesReading = lastLineValue.split(", ");
        // breaks right here, "NumberFormatException: empty String"
        angle = Arrays.stream(Arrays.copyOfRange(valuesReading, 0, 4)).mapToDouble(Double::parseDouble).toArray();
        rotations = Arrays.stream(Arrays.copyOfRange(valuesReading, 4, 8)).mapToInt(Integer::parseInt).toArray();
        sm = Arrays.stream(Arrays.copyOfRange(valuesReading, 8, 12)).mapToDouble(Double::parseDouble).toArray();
        offsets = Arrays.stream(Arrays.copyOfRange(valuesReading, 12, 16)).mapToDouble(Double::parseDouble).toArray();
        System.arraycopy(valuesReading, 12, offsetStrings, 0, offsets.length);
        // these get reprinted later as initialized
        System.arraycopy(sm, 0, lastSm, 0, 4);
        opMode.telemetry.addLine("Done Reading");
        opMode.telemetry.addData("Big Angles", Arrays.toString(angle));
        opMode.telemetry.addData("Small rotations", Arrays.toString(rotations));
        opMode.telemetry.addData("Small Angles", Arrays.toString(sm));
        opMode.telemetry.update();
        if (fileDataRaw != null && !fileDataRaw.isEmpty()) {
            lastLineValue = fileDataRaw.replace("[", "").replace("]", "");
            valuesReading = lastLineValue.split(", ");
            // Proceed with parsing
        } else {
            opMode.telemetry.addLine("No previous data found, initializing default values.");
            // Set default values if necessary
        }

//
    }
    Map<Double,Double> bl = new LinkedHashMap<Double, Double>() {{ // Voltage up, degrees down
        put(0.0, 114.0);
        put(0.228, 90.0);
        put(0.777, 45.0);
        put(1.047, 0.0);
        put(1.048, 360.0); //synthetic
        put(1.457, 315.0);
        put(1.892, 270.0);
        put(2.271, 225.0);
        put(2.686, 180.0);
        put(3.147, 135.0);
        put(3.307, 114.0);
    }};
    Map<Double,Double> fr = new LinkedHashMap<Double, Double>() {{ // Voltage up, degrees up
        put(0.0, 18.0);
        put(0.134, 0.0); //synthetic
        put(0.135, 360.0);
        put(0.536, 315.0);
        put(0.918, 270.0);
        put(1.335, 225.0);
        put(1.78, 180.0);
        put(2.181, 135.0);
        put(2.584, 90.0);
        put(3.056, 45.0);
        put(3.307, 18.0);
    }};
    Map<Double,Double> fl = new LinkedHashMap<Double, Double>() {{ // Voltage up, degrees down.
        put(0.0, 90.0);
        put(0.415, 45.0);
        put(0.875, 0.0);
        put(0.876, 360.0);
        put(1.28, 315.0);
        put(1.692, 270.0);
        put(2.107, 225.0);
        put(2.499, 180.0);
        put(2.89, 131.0);
        put(3.307, 90.0);
    }};
    Map<Double,Double> br = new LinkedHashMap<Double, Double>() {{ // voltage up deg down (not side dependent)
        put(0.0, 55.0);
        put(0.073, 45.0);
        put(0.482, 0.0);
        put(0.483, 360.0);
        put(0.916, 315.0);
        put(1.367, 270.0);
        put(1.681, 225.0);
        put(2.213, 180.0);
        put(2.529, 135.0);
        put(3.046, 90.0);
        put(3.307, 55.0);
    }};
    List<Map<Double, Double>> modulesTable = new ArrayList<Map<Double, Double>>() {{
        add(0, fl);
        add(1, fr);
        add(2, bl);
        add(3, br);
    }}; // list of maps lol
    public void getTelemetry(Telemetry t) {
        t.addData("smBR", sm[3]);
        t.addData("smRotBR", rotations[3]);
        t.addData("angBR", angle[3]);
        t.addData("angFL", angle[0]);
        t.addData("angFR", angle[1]);
        t.addData("angBL", angle[2]);
        t.addData("smRotBl", rotations[2]);
        t.addData("smRotFl", rotations[0]);
        t.addData("smRotFR", rotations[1]);
        t.addData("smBL", sm[2]);
        t.addData("smFL", sm[0]);
        t.addData("smFR", sm[1]);
        t.addData("smRawFR", voltages[1]);
        t.addData("smRawFL", voltages[0]);
        t.addData("smRawBR", voltages[3]);
        t.addData("smRawBL", voltages[2]);
        t.addData("lastSM", differenceMs[3]);
        t.addData("x1 class", x1class);
        t.addData("modules", modulesTable.size());
        t.update();
    }
    public void loop() {
//        ArrayList<Object> writingFinal;
        /*      writing final looks like this
       Big Angle            M1, M2... MN
       Small rotation       ...
       Small Angle          ...
        */

        for (int m = 0; m < modulesTable.size(); m++) {
            voltages[m] = encoders.get(m).getVoltage();
            smallPulleyAngleAccumulator(voltages[m], m);
            updateBigPulleyCalculator(m);
            lastAngStringsWriting[m] = Double.toString(angle[m]);
            smallAngString[m] = Double.toString(sm[m]);
            smallRotString[m] = Integer.toString(rotations[m]);
        }
        // go through for each module, get the newest voltage, update angle measurement, update last angles


        writingFinal = ArrayUtils.addAll(lastAngStringsWriting,smallRotString);
        writingFinal = ArrayUtils.addAll(writingFinal, smallAngString);
        writingFinal = ArrayUtils.addAll(writingFinal, offsetStrings);
        ReadWriteFile.writeFile(dataLog, Arrays.toString(writingFinal));
        Arrays.fill(writingFinal, null);
        opMode.telemetry.addData("Big Angles", Arrays.toString(angle));
        opMode.telemetry.addData("Small rotations", Arrays.toString(rotations));
        opMode.telemetry.addData("Small Angles", Arrays.toString(sm));
        System.arraycopy(sm, 0, lastSm, 0, 4);
        // write to the txt everything
        // everything being big pulley angle > small pulley full rotations > small pulley angle pose
    }
    public double[] getBigPulleyAngles() {
        return angle;
    }
    public double voltsToAngle(double voltage, int module) {
        double out;
        Map<Double, Double> targetTable = modulesTable.get(module);
        Set<Double> keySet = targetTable.keySet();
        Collection<Double> valueSet = targetTable.values();
        // both of the below are doubles but it won't let me
        Object[] values = valueSet.toArray();
        Object[] keys = keySet.toArray();
        if (keySet.contains(voltage)) {
            return targetTable.get(voltage);
        } else {
            for (int i = 0; i < keys.length; i++) {
                if (voltage < Double.parseDouble(keys[i].toString()) ) {
                    double y1, x1, y2, x2, m;
                    x2 = Double.parseDouble(keys[i].toString());
                    x1 = Double.parseDouble(keys[i-1].toString());
                    y1 = Double.parseDouble(values[i-1].toString());
                    y2 = Double.parseDouble(values[i].toString());
                    m = (y2 - y1)/(x2 - x1);
                    // point slope of the line b/w the points its between
                    out = (m * (voltage - x1)) + y1;
//                    if (module == 0) {
//                        x1class = x2;
//                    } else if (module == 1) {
//                        x1classR = x2;
//                    }


                    // input x as the voltage into the formula
                    return out;
                    // TODO: somehow this is a little wonky
                }
            }
        }
        return 0;
    }
    public void smallPulleyAngleAccumulator(double inputVoltage, int module) {
        sm[module] = voltsToAngle(inputVoltage, module);
        double difference = sm[module] - lastSm[module];
        differenceMs[module] = difference;
        // these are never being active??
        if (Math.abs(difference) > 180) {
//            if (module != 1) {
                if (sm[module] < lastSm[module]) {
                    rotations[module]++;
                } else {
                    rotations[module]--;
                }
//            } else {
//                if (sm[module] < lastSm[module]) {
//                    rotations[module]--;
//                } else {
//                    rotations[module]++;
//                }
//            }
        }
        // this updates the small pulley things
    }
    public void updateBigPulleyCalculator(int m) {
        switch (m) {
            case 0:
                double degreesRawFL = sm[m] + (rotations[m] * 360) + offsets[0];
                double tempAngFL = ((degreesRawFL * smallToBigPulley) ) % 360;
                if (tempAngFL < 0) {
                    tempAngFL += 360;
                }
                angle[m] = tempAngFL;
                break;
            case 1:
                double frOffset = -130.5;
                double degreesRawFR = sm[m] + (rotations[m] * 360) + offsets[1];
                double tempAngFR = ((degreesRawFR * smallToBigPulley) ) % 360;
                if (tempAngFR < 0) {
                    tempAngFR += 360;
                }
                angle[m] = tempAngFR;
                break;
            case 2:
                double blOffset = -113;
                double degreesRawBL = sm[m] + (rotations[m] * 360) + offsets[2];
                double tempAngBL= ((degreesRawBL * smallToBigPulley) )% 360;
                if (tempAngBL < 0) {
                    tempAngBL += 360;
                }
                angle[m] = tempAngBL;
                break;
            case 3:
                double brOffset = -345;
                double degreesRawBR = sm[m] + (rotations[m] * 360) + offsets[3];
                double tempAngBR = ((degreesRawBR * smallToBigPulley) ) % 360;
                if (tempAngBR < 0) {
                    tempAngBR += 360;
                }
                angle[m] = tempAngBR;
                break;
        }
    }


}
