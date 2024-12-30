package com.kalipsorobotics.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OpModeUtilities {

    private static OpModeUtilities single_instance = null;

    private static HardwareMap hardwareMap;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    public OpModeUtilities(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public static synchronized OpModeUtilities getInstance(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        if (single_instance == null) {
            single_instance = new OpModeUtilities(hardwareMap, opMode, telemetry);
        } else {
            setHardwareMap(hardwareMap);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    private static void setHardwareMap(HardwareMap hardwareMap){
        OpModeUtilities.hardwareMap = hardwareMap;
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

}