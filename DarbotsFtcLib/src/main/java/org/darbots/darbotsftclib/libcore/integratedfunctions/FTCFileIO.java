package org.darbots.darbotsftclib.libcore.integratedfunctions;


import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;

public class FTCFileIO {
    public static String readFile(File file){
        String FileContent = ReadWriteFile.readFile(file);
        return FileContent;
    }

    public static Bitmap readBitmapFile(File file){
        FileInputStream stream = null;
        Bitmap resultBitmap = null;
        try {
            stream = new FileInputStream(file);
            resultBitmap = BitmapFactory.decodeStream(stream);
        }catch(Exception e){

        }finally{
            if(stream != null){
                try {
                    stream.close();
                }catch(Exception e){

                }
            }
        }
        return resultBitmap;
    }

    public static void writeFile(File file, String content){
        ReadWriteFile.writeFile(file,content);
    }

    public static void writeBitmapFile(File file, Bitmap content, int qualityPct ,Bitmap.CompressFormat CompressFormat){
        FileOutputStream stream = null;
        try{
            if(!file.isFile()){
                file.createNewFile();
            }
            stream = new FileOutputStream(file);
            content.compress(CompressFormat,qualityPct,stream);
        }catch(Exception e){

        }finally{
            try {
                if (stream != null) {
                    stream.flush();
                    stream.close();
                }
            }catch(Exception e){

            }
        }
    }

    public static File getSettingFile(String fileName){
        return AppUtil.getInstance().getSettingsFile(fileName);
    }
    public static File getFirstFolderFile(String filename){
        return new File(AppUtil.FIRST_FOLDER,filename);
    }
    public static File getSnapshotFolder(){
        File firstFolder = getFirstFolder();
        File snapshotFolder = new File(firstFolder,"Snapshots");
        if(!snapshotFolder.isDirectory()){
            snapshotFolder.mkdirs();
        }
        return snapshotFolder;
    }
    public static File getSnapshotFolderFile(String filename){
        return new File(getSnapshotFolder(),filename);
    }
    public static File getFirstFolder(){
        return AppUtil.FIRST_FOLDER;
    }
    public static File getLogFolder(){
        return AppUtil.LOG_FOLDER;
    }

    public static File getLogFolderFile(String fileName){
        return new File(AppUtil.LOG_FOLDER,fileName);
    }

}
