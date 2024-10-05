package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class CameraConfig
{
    public static class CameraConfigData
    {
        public static final String ConePointAX_Str = "ctax";
        public static final String ConePointAY_Str = "ctay";
        public static final String ConePointBX_Str = "ctbx";
        public static final String ConePointBY_Str = "ctby";
        
        public int ConePointAX = 0;
        public int ConePointAY = 0;
        public int ConePointBX = 0;
        public int ConePointBY = 0;

        public static final String Red_Str = "red";
        public static final String Blue_Str = "blue";
        public static final String Yellow_Str = "yellow";
        
        public int Red = 180;
        public int Blue = 142;
    
        public static final String ConeWidth_Str = "cwidth";

        public int ConeWidth = 250;

    
        public static final String PathCenterX_Str = "center";
        public int PathCenterX = Constants.CameraViewWidth/2;
        
        public void Copy(CameraConfigData source)
        {
            this.ConePointAX = source.ConePointAX;
            this.ConePointAY = source.ConePointAY;
            this.ConePointBX = source.ConePointBX;
            this.ConePointBY = source.ConePointBY;
            this.Red = source.Red;
            this.Blue = source.Blue;
            this.ConeWidth = source.ConeWidth;
            this.PathCenterX = source.PathCenterX;
        }
    }
    
    private static final String fileName = "/CameraTarget.json";
    
    private static final String directoryPath = Environment.getExternalStorageDirectory().getPath() +
            Constants.cameraConfigFolder;
    
    public static String SaveConfigToFile(CameraConfigData newConfig)
    {
        JSONObject InitData = new JSONObject();
        try
        {
            InitData.put(CameraConfigData.ConePointAX_Str, newConfig.ConePointAX);
            InitData.put(CameraConfigData.ConePointAY_Str, newConfig.ConePointAY);
            InitData.put(CameraConfigData.ConePointBX_Str, newConfig.ConePointBX);
            InitData.put(CameraConfigData.ConePointBY_Str, newConfig.ConePointBY);
//
//            InitData.put(CameraConfigData.PolePointAX_Str, newConfig.PolePointAX);
//            InitData.put(CameraConfigData.PolePointAY_Str, newConfig.PolePointAY);
//            InitData.put(CameraConfigData.PolePointBX_Str, newConfig.PolePointBX);
//            InitData.put(CameraConfigData.PolePointBY_Str, newConfig.PolePointBY);
    
            InitData.put(CameraConfigData.Red_Str, newConfig.Red);
            InitData.put(CameraConfigData.Blue_Str, newConfig.Blue);
//            InitData.put(CameraConfigData.Yellow_Str, newConfig.Yellow);
            
            InitData.put(CameraConfigData.ConeWidth_Str, newConfig.ConeWidth);
//            InitData.put(CameraConfigData.PoleWidth_Str, newConfig.PoleWidth);
    
            InitData.put(CameraConfigData.PathCenterX_Str, newConfig.PathCenterX);
            
            // Convert JsonObject to String Format
            String userString = InitData.toString();
            //telemetry.addLine(userString);
            // Define the File Path and its Name
            File directory = new File(directoryPath);
//            if(!directory.mkdir())
//                return null;
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
    
    public static int[] ReadConfigFromFile(CameraConfigData config)
    {
        int[] data = new int[8];
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
            // This response will have Json Format String
            String response = stringBuilder.toString();
            
            JSONObject jsonObject = new JSONObject(response);
            data[0] = Integer.parseInt(jsonObject.get(CameraConfigData.ConePointAX_Str).toString());
            data[1] = Integer.parseInt(jsonObject.get(CameraConfigData.ConePointAY_Str).toString());
            data[2] = Integer.parseInt(jsonObject.get(CameraConfigData.ConePointBX_Str).toString());
            data[3] = Integer.parseInt(jsonObject.get(CameraConfigData.ConePointBY_Str).toString());
//            data[4] = Integer.parseInt(jsonObject.get(CameraConfigData.PolePointAX_Str).toString());
//            data[5] = Integer.parseInt(jsonObject.get(CameraConfigData.PolePointAY_Str).toString());
//            data[6] = Integer.parseInt(jsonObject.get(CameraConfigData.PolePointBX_Str).toString());
//            data[7] = Integer.parseInt(jsonObject.get(CameraConfigData.PolePointBY_Str).toString());
            data[4] = Integer.parseInt(jsonObject.get(CameraConfigData.Red_Str).toString());
            data[5] = Integer.parseInt(jsonObject.get(CameraConfigData.Blue_Str).toString());
//            data[10] = Integer.parseInt(jsonObject.get(CameraConfigData.Yellow_Str).toString());
            data[6] = Integer.parseInt(jsonObject.get(CameraConfigData.ConeWidth_Str).toString());
//            data[12] = Integer.parseInt(jsonObject.get(CameraConfigData.PoleWidth_Str).toString());
            data[7] = Integer.parseInt(jsonObject.get(CameraConfigData.PathCenterX_Str).toString());
    
            config.ConePointAX = data[0];
            config.ConePointAY = data[1];
            config.ConePointBX = data[2];
            config.ConePointBY = data[3];
//            config.PolePointAX = data[4];
//            config.PolePointAY = data[5];
//            config.PolePointBX = data[6];
//            config.PolePointBY = data[7];
            config.Red = data[4];
            config.Blue = data[5];
//            config.Yellow = data[10];
            config.ConeWidth = data[6];
//            config.PoleWidth = data[12];
            config.PathCenterX = data[7];
            
        } catch (Exception e)
        {
            return null;
        }
        return data;
    }
}
