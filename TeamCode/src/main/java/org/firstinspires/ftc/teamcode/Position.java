package org.firstinspires.ftc.teamcode;

import java.util.Hashtable;

// An object class to encode the position and type of a VuMark based off ConceptVuf...Webcam.

public class Position {
    String type = "";
    float x, y, z, roll, pitch, heading;

    Position(String type, float x, float y, float z, float roll, float pitch, float heading) {
        this.type = type;
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.heading = heading;
    }

    public String getType() {
        return type;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
        return z;
    }

    public float getRoll() {
        return roll;
    }

    public float getPitch() {
        return pitch;
    }

    public float getHeading() {
        return heading;
    }

    public boolean equals(Position P) {
        return type.equals(P.getType()) && x == P.getX() && y == P.getY() && z == P.getZ() && roll == P.getRoll() && pitch == P.getPitch() && heading == P.getHeading();
    }

    public Hashtable<String,String> toHashtable() {
        Hashtable<String, String> dict = new Hashtable<>();

        dict.put("type",type);
        dict.put("x",Double.toString(x));
        dict.put("y",Double.toString(y));
        dict.put("z",Double.toString(z));
        dict.put("roll",Double.toString(roll));
        dict.put("pitch",Double.toString(pitch));
        dict.put("heading",Double.toString(heading));

        return dict;
    }

    // Outputs a hashtable with deltas of the numerical components of two Positions.
    public static Hashtable<String, Float> calculateDelta(Position p1, Position p2) {
        Hashtable<String, Float> dict = new Hashtable<>();

        dict.put("dx",p1.getX()-p2.getX());
        dict.put("dy",p1.getY()-p2.getY());
        dict.put("dz",p1.getZ()-p2.getZ());
        dict.put("dR",p1.getRoll()-p2.getRoll());
        dict.put("dP",p1.getPitch()-p2.getPitch());
        dict.put("dH",p1.getHeading()-p2.getHeading());

        return dict;
    }
}
