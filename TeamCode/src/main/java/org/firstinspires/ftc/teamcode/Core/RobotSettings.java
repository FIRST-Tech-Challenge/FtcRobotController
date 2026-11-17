package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.config.Config;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

@Config
public class RobotSettings {
    public enum Alliance {
        RED(new Pose2D(DistanceUnit.METER, -1.432, 1.5, AngleUnit.DEGREES, 136.4)),
        BLUE(new Pose2D(DistanceUnit.METER, -1.432, -1.5, AngleUnit.DEGREES, 223.6));

        private Pose2D pose2D;

        Alliance(Pose2D pos) {
            this.pose2D = pos;
        }

        public Pose2D getGoalPos() {
            return pose2D;
        }
    }

    public enum DistanceMethod {
        LIMELIGHT_ONLY,
        PINPOINT_ONLY,
        LIMELIGHT_AND_PINPOINT
    }

    public enum StartPos {
        CLOSE_RED(new Pose2D(DistanceUnit.METER, -1.419, 1.224, AngleUnit.DEGREES, 40.4)),
        FAR_RED(new Pose2D(DistanceUnit.METER, 1.64, 0.44, AngleUnit.DEGREES, 90)),
        CLOSE_BLUE(new Pose2D(DistanceUnit.METER, -1.419, -1.224, AngleUnit.DEGREES, 319.6)),
        FAR_BLUE((new Pose2D(DistanceUnit.METER, 1.64, -0.16, AngleUnit.DEGREES, 180)));

        private Pose2D pose2D;

        StartPos(Pose2D pos) {
            this.pose2D = pos;
        }

        public Pose2D getPose2D() {
            return pose2D;
        }
    }

    public Alliance alliance;
    public DistanceMethod distanceMethod;
    public StartPos startPosState;

    private static final String FILENAME = "RobotSettings.json";

    // Writes file
    public void save() {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        try (FileWriter writer = new FileWriter(file)) {
            gson.toJson(this, writer);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // reads file and loads into program
    // TODO: add a safety condition here it self so no null object refs
    public static RobotSettings load() {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        Gson gson = new Gson();
        try (FileReader reader = new FileReader(file)) {
            return gson.fromJson(reader, RobotSettings.class);
        } catch (IOException e) {
            // If file not found or error, return default settings
            return new RobotSettings();
        }
    }

    public RobotSettings() {
        alliance = Alliance.RED;
        distanceMethod = DistanceMethod.LIMELIGHT_ONLY;
        startPosState = StartPos.CLOSE_RED;
    }

}