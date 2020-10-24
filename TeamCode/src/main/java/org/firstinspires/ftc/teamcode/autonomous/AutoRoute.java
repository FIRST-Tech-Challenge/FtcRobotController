package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Point;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.Serializable;
import java.util.ArrayList;

public class AutoRoute implements Serializable, Cloneable {
    public static String NAME_BLUE = "Blue";
    public static String NAME_RED = "Red";
    public static String NAME_NEW = "New";
    private int nameIndex = 0;
    private String name = NAME_BLUE;
    private boolean selected;
    private boolean temp;
    private ArrayList<AutoStep> steps = new ArrayList<>();
    private int startX;
    private int startY;
    private long lastRunTime = 0;

    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static AutoRoute deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, AutoRoute.class);
    }

    public AutoRoute clone(){
        AutoRoute cloned = new AutoRoute();
        cloned.setName(this.getName());
        cloned.setNameIndex(this.getNameIndex());
        cloned.setStartX(this.getStartX());
        cloned.setStartY(this.getStartY());
        cloned.setSelected(this.isSelected());
        cloned.setTemp(this.isTemp());
        for(int i = 0; i < steps.size(); i++){
            cloned.steps.add(this.steps.get(i));
        }
        return cloned;
    }

    public String getRouteName()
    {
        if (this.isTemp()){
            return  String.format("%s-%d-%s", name, nameIndex, NAME_NEW);
        }
        return  String.format("%s-%d", name, nameIndex);
    }


    public ArrayList<AutoStep> getSteps() {
        return steps;
    }

    public void setSteps(ArrayList<AutoStep> steps) {
        this.steps = steps;
    }

    public boolean isSelected() {
        return selected;
    }

    public void setSelected(boolean selected) {
        this.selected = selected;
    }

    public int getNameIndex() {
        return nameIndex;
    }

    public void setNameIndex(int nameIndex) {
        this.nameIndex = nameIndex;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getStartX() {
        return startX;
    }

    public void setStartX(int startX) {
        this.startX = startX;
    }

    public int getStartY() {
        return startY;
    }

    public void setStartY(int startY) {
        this.startY = startY;
    }

    public Point getStart(){
        return new Point(startX, startY);
    }

    public boolean isTemp() {
        return temp;
    }

    public void setTemp(boolean temp) {
        this.temp = temp;
    }

    public long getLastRunTime() {
        return lastRunTime;
    }

    public void setLastRunTime(long lastRunTime) {
        this.lastRunTime = lastRunTime;
    }
}

