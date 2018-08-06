package com.acmerobotics.roadrunner.trajectory;

import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.acmerobotics.splinelib.trajectory.TrajectoryConfig;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.fasterxml.jackson.module.kotlin.KotlinModule;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.io.InputStream;

public class AssetsTrajectoryLoader {
    private static final ObjectMapper MAPPER = new ObjectMapper(new YAMLFactory());

    static {
        MAPPER.registerModule(new KotlinModule((512)));
    }

    public static TrajectoryConfig loadConfig(String name) throws IOException {
        InputStream inputStream = AppUtil.getDefContext().getAssets().open("trajectory/" + name + ".yaml");
        return MAPPER.readValue(inputStream, TrajectoryConfig.class);
    }

    public static Trajectory load(String name) throws IOException {
        return loadConfig(name).toTrajectory();
    }
}
