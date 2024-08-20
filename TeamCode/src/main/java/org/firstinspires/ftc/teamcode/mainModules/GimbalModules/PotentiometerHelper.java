package org.firstinspires.ftc.teamcode.mainModules.GimbalModules;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;
import java.util.TreeMap;

public class PotentiometerHelper {

    private TreeMap<Double, Double> potDataMap;

    // Constructor to load the data when the class is instantiated
    public PotentiometerHelper() {
        potDataMap = loadPotentiometerData("/sdcard/potentiometerLogs1000/potentiometer_data_12.99V.txt");
    }

    // Function to load the potentiometer data from a file into a TreeMap
    private TreeMap<Double, Double> loadPotentiometerData(String filePath) {
        TreeMap<Double, Double> map = new TreeMap<>();
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] parts = line.split(",");
                double key = Double.parseDouble(parts[0]);
                double value = Double.parseDouble(parts[1]);
                map.put(key, value);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return map;
    }

    // Function to compare from the tables and find the closest match
    public double compareFromTables(double potMeterValue) {
        // Check if the exact value exists
        if (potDataMap.containsKey(potMeterValue)) {
            return potDataMap.get(potMeterValue);
        }

        // Find the closest value using floorEntry and ceilingEntry
        Map.Entry<Double, Double> lowerEntry = potDataMap.floorEntry(potMeterValue);
        Map.Entry<Double, Double> higherEntry = potDataMap.ceilingEntry(potMeterValue);

        if (lowerEntry == null) {
            assert higherEntry != null;
            return higherEntry.getValue();
        }
        if (higherEntry == null) return lowerEntry.getValue();

        // Choose the closest value
        if ((potMeterValue - lowerEntry.getKey()) <= (higherEntry.getKey() - potMeterValue)) {
            return lowerEntry.getValue();
        } else {
            return higherEntry.getValue();
        }
    }
}
