package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Point;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;

import java.io.Serializable;


public class AutoDot implements Serializable {
    //Character.toString ((char) i);
//    char character = 'a';
//    int ascii = (int) character;
    public static int asciiA = 65;
    public static int asciiZ = 90;
    private String dotName = "A";
    private boolean selected;
    private int x;
    private int y;
    private double heading = BotMoveProfile.DEFAULT_HEADING;


    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static AutoDot deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, AutoDot.class);
    }

    @Override
    public String toString() {
        return this.getPoint().toString();
    }

    public String getDotName() {
        return dotName;
    }

    public void setDotName(String dotName) {
        this.dotName = dotName;
    }

    public boolean isSelected() {
        return selected;
    }

    public void setSelected(boolean selected) {
        this.selected = selected;
    }

    public int getX() {
        return x;
    }

    public void setX(int x) {
        this.x = x;
    }

    public int getY() {
        return y;
    }

    public void setY(int y) {
        this.y = y;
    }

    public Point getPoint(){
        return new Point(x, y);
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }
}
