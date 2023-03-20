package org.firstinspires.ftc.teamcode.robots.taubot.util;
import android.os.Environment;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.time.LocalTime;
import java.util.Scanner;

import com.acmerobotics.roadrunner.geometry.Pose2d;


public class PositionLogger {
    private FullObjectOutputStream out;
    private ObjectInputStream in;
    private String dir;
    private String filename;
    private String log;
    public PositionLogger(String name) {
        dir = Environment.getExternalStorageDirectory() + "/FIRST/";
        filename = dir + name + ".txt";
        System.out.println(filename);
        try {
            out = new FullObjectOutputStream(new FileOutputStream(filename));
            log = "START OF POSITION LOG - " + name;
            log += "\n START TIME - " + LocalTime.now();
            out.writeObject(log);
            out.flush();
            out.close();
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }

    }

    public void updateLog(Pose2d pos) throws IOException{
        log = "TIME - " + LocalTime.now() + " POSITION - " + pos.toString();
        out = new FullObjectOutputStream(new FileOutputStream(filename));
        out.writeObject(log);
        out.close();
        out.flush();
    }

    public String returnLastTime () {
        if(returnLastLog().contains("POSITION - "))
            return returnLastLog().substring(7, returnLastLog().indexOf(" POSITION - "));
        return "no positions logged yet";
        //returns as a string, could be annoying but didn't know what else to return
    }

    public Pose2d returnLastPose () {
        //currently returns empty pose if nothing's in log
        Pose2d pose = new Pose2d();
        String easyLog = returnLastLog().substring(returnLastLog().indexOf("N - ") + 5);
        easyLog = easyLog.replaceAll("[^\\d.-]", " ");
        System.out.println(easyLog);
        if(returnLastLog().contains("POSITION - ")) {
            Scanner sc = new Scanner(easyLog);
            pose = new Pose2d(sc.nextDouble(), sc.nextDouble(), Math.toRadians(sc.nextDouble()));
        }
        return pose;

    }

    public String getFilename() {
        return filename;
    }

    public String returnLastLog() {
        try {
            ObjectInputStream in = new ObjectInputStream(new FileInputStream(filename));
            return (String)in.readObject();
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }
        return "file could not be read";
    }

}

class FullObjectOutputStream extends ObjectOutputStream{
    public FullObjectOutputStream(OutputStream out) throws IOException {
        super(out);
    }

    public boolean enableReplaceObject(boolean allowed) {
        super.enableReplaceObject(allowed);
        return allowed;
    }

    public Object replaceObject(Object obj) throws IOException {
        super.replaceObject(obj);
        return obj;
    }
}
