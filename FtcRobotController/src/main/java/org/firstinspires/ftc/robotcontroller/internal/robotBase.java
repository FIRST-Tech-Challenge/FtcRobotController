package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.HardwareMapper;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public abstract class robotBase {

    protected Telemetry telemetry;
    protected HardwareMapper mapper;
    protected LinearOpMode opMode;
    protected HardwareMap hardwaremap;

     public robotBase(Telemetry telemetry, HardwareMapper mapper, LinearOpMode opMode, HardwareMap hardwaremap){
        this.telemetry = telemetry;
        this.mapper = mapper;
        this.opMode = opMode;
        this.hardwaremap = hardwaremap;

    }

    public HardwareMap getHardwaremap() {
        return hardwaremap;
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public HardwareMapper getMapper() {
        return mapper;
    }

    public void init(){}




    abstract public void stop();
}
