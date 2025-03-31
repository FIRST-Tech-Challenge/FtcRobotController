package com.kalipsorobotics.utilities;

import android.os.Process;

import com.kalipsorobotics.localization.WheelOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

public class OpModeUtilities {

    private final HardwareMap hardwareMap;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    public OpModeUtilities(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public static void shutdownExecutorService(ExecutorService executorService) throws InterruptedException {
        executorService.shutdownNow();
        executorService.awaitTermination(1, TimeUnit.SECONDS);
    }

    public static void runOdometryExecutorService(ExecutorService executorService, WheelOdometry wheelOdometry) {
        try {
            executorService.submit(() -> {
                Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);
                while (!Thread.currentThread().isInterrupted()) {
                    wheelOdometry.updatePositionAll();
                }

            });
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    }

}