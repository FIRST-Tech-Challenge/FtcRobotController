package org.firstinspires.ftc.teamcode.robots.taubot.util;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.Position;

import android.os.Environment;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.Scanner;

import com.acmerobotics.roadrunner.geometry.Pose2d;


public class PositionLogger {
    private FullObjectOutputStream out;
    private ObjectInputStream in;
    private String dir = Environment.getExternalStorageDirectory() + "/FIRST/";
    private String filename;
    private String log;
    public PositionLogger(String name) {
        filename = dir + name + ".txt";
        try {
             out = new FullObjectOutputStream(new FileOutputStream(filename));
             log = "POSITION LOG - " + name;
             log += "\n START DATE AND TIME - " + LocalTime.now();
             out.writeObject(log);
             out.flush();
             out.enableReplaceObject(true);
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }

    }

    public void updateLog(Pose2d pos){
        log = "DATE AND TIME - " + LocalTime.now() + " POSITION - " + pos.toString();
        out.replaceObject(log);
    }

    public String returnLastTime () {
        if(returnLastLog().contains("POSITION - "))
            return returnLastLog().substring(returnLastLog().indexOf("TIME - " + 7), returnLastLog().indexOf(" POSITION - "));
        return "no positions logged yet";
        //returns as a string, could be annoying but didn't know what else to return
    }

    public Pose2d returnLastPose () {
        //currently returns empty pose if nothing's in log
        Pose2d pose = new Pose2d();
        if(returnLastLog().contains("POSITION - ")) {
            Scanner sc = new Scanner(returnLastLog());
            //todo - not sure if time picks up as a double token
            //currently assuming it doesn't
            if (returnLastLog().contains("POSITION - ")) {
                pose = new Pose2d(sc.nextDouble(), sc.nextDouble(), sc.nextDouble());
            }
        }
            return pose;

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

    public Object replaceObject(Object obj) {
        try {
            super.replaceObject(obj);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return obj;
    }
}
