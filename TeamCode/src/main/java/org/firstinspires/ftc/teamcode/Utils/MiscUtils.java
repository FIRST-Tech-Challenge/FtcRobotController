package org.firstinspires.ftc.teamcode.Utils;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;

public class MiscUtils {

    public static boolean Gsix = false;
    public static boolean Psix = false;

    /**
     * Clamps a number between 2 values
     * @param number Number to clamp
     * @param clampLow the lower bound
     * @param clampHigh the higher bound
     * @return the clamped number
     * **/
    public static float Clamp(float number, float clampLow, float clampHigh){
        //Clamping for stuff!
        if(number < clampHigh){
            if(number < clampLow){
                return clampLow;
            }
        }else{
            return clampHigh;
        }
        return number;
    }

    /**
     * Clamps a number between 2 values
     * @param number Number to clamp
     * @param clampLow the lower bound
     * @param clampHigh the higher bound
     * @return the clamped number
     * **/
    public static double Clamp(double number, double clampLow, double clampHigh){
        //Clamping for stuff!
        if(number < clampHigh){
            if(number < clampLow){
                return clampLow;
            }
        }else{
            return clampHigh;
        }
        return number;
    }

    public static final String dataFolder = Environment.getExternalStorageDirectory().getPath()+"/FIRST/data";

    /**
     * Writes to a file
     * @param path File path to write to
     * @param data Data to write to the file
     * @param tem Telemetry to complain to
     */
    public static void writeFile(String path, byte[] data, Telemetry tem) {
        try {
            File file = new File(path);
            if (!file.exists()) {
                tem.addData("woops", "File did not exist");
                boolean created = file.createNewFile();
                tem.addData("woops", "Successful creation: "+created);
            }
            FileOutputStream out = new FileOutputStream(path);
            out.write(data);
        } catch(IOException e) {
            tem.addData("woops",e.toString());
        }
    }
    /**
     * Writes to a file
     * @param path File path to write to
     * @param data Data to write to the file
     */
    public static void writeFile(String path, byte[] data) {
        try {
            File file = new File(path);
            if (!file.exists()) {
                boolean created = file.createNewFile();
            }
            FileOutputStream out = new FileOutputStream(path);
            out.write(data);
        } catch(IOException e) {
        }
    }

    /**
     * Writes a piece of labeled piece of data to the specified file.
     * @param path File path to write to
     * @param id Label for the data
     * @param data The data to write
     */
    public static void writeToConfig(String path, byte id, byte data) {
        boolean check = false;
        byte[] data2 = readFile(path);
        for(int i = 0; i < data2.length; i+=2) {
            if(data2[i] == id) {
                check = true;
                data2[i+1] = data;
                writeFile(path,data2);
            }
        }
        if(!check) {
            byte[] data3 = new byte[data2.length+2];
            int i = 0;
            for (byte d : data2) {
                data3[i] = d;
                i++;
            }
            data3[data2.length] = id;
            data3[data2.length+1] = data;
            writeFile(path,data3);
        }
    }

    /**
     * Reads a file
     * @param path File path to read from
     * @param tem Telemetry to complain to
     * @return Returns everything in the specified file
     */
    public static byte[] readFile(String path, Telemetry tem) {
        try {
            File file = new File(path);
            FileInputStream in = new FileInputStream(path);
            byte[] data = new byte[(int) file.length()];
            in.read(data);
            in.close();
            return data;
        }  catch (IOException e) {
            tem.addData("woops",e.toString());
            return new byte[0];
        }
    }

    /**
     * Reads a file
     * @param path File path to read from
     * @return Returns everything in the specified file
     */
    public static byte[] readFile(String path) {
        try {
            File file = new File(path);
            FileInputStream in = new FileInputStream(path);
            byte[] data = new byte[(int) file.length()];
            in.read(data);
            in.close();
            return data;
        }  catch (IOException e) {
            return new byte[0];
        }
    }

    /**
     *Reads data with a specific label
     * @param path The file path to read
     * @param ID The label to look for
     * @param tem Telemetry to complain to
     * @return The data with the specified label
     */
    public static byte readConfig(String path, byte ID, Telemetry tem) {
        byte[] data = readFile(path,tem);
        for(int i = 0; i < data.length; i+=2) {
            if(data[i] == ID) {
                return data[i+1];
            }
        }
        tem.addData("woops","ID not found");
        return -1;
    }

    /**
     *Reads data with a specific label
     * @param path The file path to read
     * @param ID The label to look for
     * @return The data with the specified label
     */
    public static byte readConfig(String path, byte ID) {
        byte[] data = readFile(path);
        for(int i = 0; i < data.length; i+=2) {
            if(data[i] == ID) {
                return data[i+1];
            }
        }
        return -1;
    }

    /**
     * Checks if a file exists
     * @param path The file path to check
     * @return if the file exists
     */
    public static boolean checkFile(String path) {
        File file = new File(path);
        return file.exists();
    }

    /**
     * Gets memory usage in KB
     * @return memory usage in KB
     */
    public static float getMemoryUsage() {
        return (float) (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) /1024;
    }

    /**
     * Gets memory usage in percent
     * @return memory usage in percent
     */
    public static float getMemoryUsageP() {
        return (float)Runtime.getRuntime().totalMemory()/Runtime.getRuntime().freeMemory();
    }

    /**
     * Gets the distance between 2 angles (from a1 to a2)
     * @param a1 the first angle in degrees
     * @param a2 the second angle in degrees
     */
    public static float getAngleDifferenceDegrees(float a1, float a2) {
        float op1 = a2-a1;
        float op2 = 360-(a1-a2);
        if(Math.abs(op1) <= Math.abs(op2)) {
            return op1;
        } else {
            return op2;
        }
    }

    /**
     * Outputs all robopath files in a certain folder.
     * @param f the folder to look in.
     * @return robopath files.
     */
    public static File[] getRobopathsIn(File f) {
        File[] files = new File[0];
        if(f.exists()) {
            if(f.isFile()) {
                f = f.getParentFile();
            }
            if(f != null) files = f.listFiles(new RobopathFilter());
        }
        return files;
    }

    /**
     * Check if the battery is low
     * @param h hardwareMap.
     * @return true if battery voltage is > 12, false if otherwise.
     */
    public static boolean checkBattery(HardwareMap h) {
        float out = 100;
        for(VoltageSensor vs : h.voltageSensor) {
            double t = vs.getVoltage();
            if(t > 0 && t < out) out = (float) t;
        }
        if(out > 12) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Convert degrees to servo values
     * @param servo a {@link ServoType} that represents the type of servo.
     * @param deg angle in degrees.
     * @return servo value.
     */
    public static float servoConvert(ServoType servo, float deg) {
        if (servo == ServoType.ThreeHundredDegrees) {
            return deg/300;
        } else if (servo == ServoType.FiveTurn) {
            return deg/1800;
        }

        return 0;
    }

    public enum ServoType{
        ThreeHundredDegrees,
        FiveTurn
    }

    //Season specific


    public static float CannonGreenVperP(float P) {
        return (float) (((78.4f*Math.pow(P,2))+(247.6f*P)+0.03f)*1f)*1.21f;
    }
    public static float CannonPurpleVperP(float P) {
        return (float) (((78.4f*Math.pow(P,2))+(247.6f*P)+0.03f)*0.84f)*1.14f;
    }

    public static void shootR(Servo chain) {
        chain.setPosition(1);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {}
        chain.setPosition(0.39);
    }

    public static void shootL(Servo chain) {
        chain.setPosition(0.95);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {}
        chain.setPosition(0.35);
    }

    public static void shootG(CRServo chain, DistanceSensor LCDS) throws InterruptedException {

        chain.setPower(0.25);
        while(LCDS.getDistance(DistanceUnit.MM) > 70) {}
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 251) {}
        chain.setPower(0);

        /*if(Gsix) {
            chain.setPower(1);
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 640) {}
            chain.setPower(0);
            Gsix = false;
        } else {
            chain.setPower(1);
            long start = System.currentTimeMillis();
            while(System.currentTimeMillis()-start<600) {}
            chain.setPower(0);
            Gsix = true;
        }*/
    }

    public static void shootP(CRServo chain, ColorSensor RCCS) throws InterruptedException {

        chain.setPower(0.25);
        while(RCCS.alpha() < 100) {}
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 150) {}
        chain.setPower(0);

        /*if(Psix) {
            chain.setPower(1);
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 640) {}
            chain.setPower(0);
            Psix = false;
        } else {
            chain.setPower(1);
            long start = System.currentTimeMillis();
            while(System.currentTimeMillis()-start<580) {}
            chain.setPower(0);
            Psix = true;
        }*/
    }

    /** Sets Turret 0 to center
     * minimum is -245
     * max is 155
     * **/
    public static void ZeroTurret(DcMotorEx turret) {
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setVelocity(-200);
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis()-start < 600);
        while(Math.abs(turret.getVelocity()) > 5);
        turret.setVelocity(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(245);
        while(turret.isBusy());
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(0);
    }


    public static double getAverage(double[] array) {
        if (array == null || array.length == 0) {
            return 0.0; // Returns 0.0 if the list is empty or null
        }

        double total = 0;

        for (double v : array) {
            total += v;
        }

        return total / array.length;
    }

    public static double dotProduct(double[] a, double[] b){
        double sum = 0;

        for (int i = 0; i < a.length; i++){
            sum += a[i] * b[i];
        }

        return sum;
    }

    public static int getIndexOfMax(double[] array) {
        if (array == null || array.length == 0) {
            return -1; // Handle empty or null array
        }

        int maxIndex = 0; // Start assuming the first element is the largest

        for (int i = 1; i < array.length; i++) {
            if (array[i] > array[maxIndex]) {
                maxIndex = i; // Update index of the highest value
            }
        }

        return maxIndex;
    }

    public static int getIndexOfMax(double[] array,int start, int end) {
        if (array == null || array.length == 0 || start > end || start > array.length-2) {
            return -1; // Handle empty or null array
        }

        int maxIndex = start; // Start assuming the first element is the largest

        for (int i = 1+start; i < Math.min(array.length,end); i++) {
            if (array[i] > array[maxIndex]) {
                maxIndex = i; // Update index of the highest value
            }
        }

        return maxIndex;
    }

    public static double[] mutiplyDouble(double[] doubles, double multiplied){
        double[] result = new double[doubles.length];
        for (int i = 0; i < doubles.length; i++){
            result[i] = doubles[i] * multiplied;
        }
        return result;
    }

    public static double[] addDoubles(double[] double_one, double[] double_two, boolean addition){
        double[] result = new double[double_one.length];
        for (int i = 0; i < double_one.length; i++){
            if (addition){
                result[i] = double_one[i] + double_two[i];
            }else{
                result[i] = double_one[i] - double_two[i];
            }
        }
        return result;
    }

    public static double totalAddedNumber(double[] double_one){
        double total = 0;

        for (double number : double_one){
            total += number;
        }

        return total;
    }

    public static boolean hasNan(double[] array){
        for (double number : array){
            if (Double.isNaN(number)){
                return true;
            }
        }

        return false;
    }

    public static int findIndexOf(Object[] array, Object value){
        if (array == null || array.length == 0) {
            return -1; // Handle empty or null array
        }

        for (int i = 0; i < array.length; i++) {
            if (array[i].equals(value)) {
                return i; // Return the index of the found value
            }
        }

        return -1; // Value not found
    }

    public static double getDegreesFromMotor(double currentPosition, double countsPerRevolution){
        return (currentPosition / countsPerRevolution) * 360 * 43 / 14;
    }

    public static double getMotorPositionFromDegrees(double degrees, double countsPerRevolution){
        return (countsPerRevolution / 360) * degrees * 43 / 14;
    }
}
