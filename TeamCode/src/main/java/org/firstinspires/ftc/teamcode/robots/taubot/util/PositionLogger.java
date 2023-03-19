package org.firstinspires.ftc.teamcode.robots.taubot.util;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.Position;
import java.time.LocalDateTime;
import java.util.Scanner;

import com.acmerobotics.roadrunner.geometry.Pose2d;


public class PositionLogger {
    FullObjectOutputStream out;
    ObjectInputStream in;
    String filename;
    String log;
    public PositionLogger(String name) {
        filename = name + ".txt";
        try {
            FileOutputStream temp = new FileOutputStream(filename + ".txt");
             out = new FullObjectOutputStream(temp);
             log = "POSITION LOG - " + filename;
             log += "\n START DATE AND TIME - " + LocalDateTime.now();
             out.writeObject(log);
             out.flush();
             out.enableReplaceObject(true);
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }

    }

    public void update(Pose2d pos){
        log = "DATE AND TIME - " + LocalDateTime.now() + " POSITION - " + pos.toString();
        out.replaceObject(log);
    }

    public String returnLastDateTime () {
        if(returnLastLog().contains("POSITION - "))
            return returnLastLog().substring(returnLastLog().indexOf("TIME - " + 7), returnLastLog().indexOf(" POSITION - "));
        return "no positions logged yet";
    }

    public Pose2d returnLastPose () {
        Pose2d pose = new Pose2d();
        Scanner sc = new Scanner(returnLastLog());
        sc.nextDouble();
        if(returnLastLog().contains("POSITION - ")) {
            pose = new Pose2d(sc.nextDouble(), sc.nextDouble(), sc.nextDouble());
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
