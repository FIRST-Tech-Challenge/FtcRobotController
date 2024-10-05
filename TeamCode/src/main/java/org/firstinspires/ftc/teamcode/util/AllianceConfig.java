package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class AllianceConfig
{
    public static class AllianceConfigData
    {
        public static final String TeamNumberStr = "TeamNumber";
        public static final String AllianceStr = "AllianceColor";
        public static final String LocationStr = "InitialLocation";
        public static final String PathStr = "PathStr";
        
        public String TeamNumber = "12345";
        public String Alliance = AllianceConfig.RED;
        public String Location = AllianceConfig.LEFT;

        //Blue Alliance Robots must start Completely In Tile A2 or A4, red Alliance Robots must
        //start Completely In Tile F2 or F4. See Appendix B for Tile nomenclature.

       public String PathRoute = "0";// 1, 2, 3
    }

    public static final String RED = "RED";
    public static final String BLUE = "BLUE";
    public static final String LEFT = "LEFT";
    public static final String RIGHT = "RIGHT";
    
    private static final String fileName = "/TeamConfig.json";
    
    private static String directoryPath = Environment.getExternalStorageDirectory().getPath() +
            Constants.teamConfigFolder;
    
    public static String SaveConfigToFile(AllianceConfigData newConfig)
    {
        JSONObject InitData = new JSONObject();
        try
        {
            InitData.put(AllianceConfigData.TeamNumberStr, newConfig.TeamNumber);
            InitData.put(AllianceConfigData.AllianceStr, newConfig.Alliance);
            InitData.put(AllianceConfigData.LocationStr, newConfig.Location);
            InitData.put(AllianceConfigData.PathStr, newConfig.PathRoute);
            
            // Convert JsonObject to String Format
            String userString = InitData.toString();
            //telemetry.addLine(userString);
            // Define the File Path and its Name
            File directory = new File(directoryPath);
            directory.mkdir();
            FileWriter fileWriter = new FileWriter(
                    directoryPath + fileName);
            
            fileWriter.write(userString);
            fileWriter.close();
        } catch (Exception e)
        {
            return null;
        }
        return "Saved";
    }
    
    public static String[] ReadConfigFromFile(AllianceConfigData config)
    {
        String[] data = new String[4];
        try
        {
            FileReader fileReader = new FileReader(
                    directoryPath + fileName);
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            StringBuilder stringBuilder = new StringBuilder();
            String line = bufferedReader.readLine();
            while (line != null)
            {
                stringBuilder.append(line).append("\n");
                line = bufferedReader.readLine();
            }
            bufferedReader.close();
            fileReader.close();
            // This responce will have Json Format String
            String responce = stringBuilder.toString();
            
            JSONObject jsonObject = new JSONObject(responce);
            data[0] = ((jsonObject.get(AllianceConfigData.TeamNumberStr).toString()));
            data[1] = ((jsonObject.get(AllianceConfigData.AllianceStr).toString()));
            data[2] = ((jsonObject.get(AllianceConfigData.LocationStr).toString()));
            data[3] = ((jsonObject.get(AllianceConfigData.PathStr).toString()));
    
            config.TeamNumber = data[0];
            config.Alliance = data[1];
            config.Location = data[2];
            config.PathRoute = data[3];
            
        } catch (Exception e)
        {
            return null;
        }
        return data;
    }
}
