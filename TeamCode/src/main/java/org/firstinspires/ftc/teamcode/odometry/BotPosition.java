package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;

import java.io.Serializable;

public class BotPosition implements Serializable {
    public static String BOT_LAST_POSITION = "bot-last-position.json";
    private int posX;
    private int posY;
    private double heading;

    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static BotPosition deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, BotPosition.class);
    }

    public int getPosX() {
        return posX;
    }

    public void setPosX(int posX) {
        this.posX = posX;
    }

    public int getPosY() {
        return posY;
    }

    public void setPosY(int posY) {
        this.posY = posY;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }
}
