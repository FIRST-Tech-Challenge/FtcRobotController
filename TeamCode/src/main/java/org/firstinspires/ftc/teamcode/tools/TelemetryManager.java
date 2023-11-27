package org.firstinspires.ftc.teamcode.tools;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryManager {
    private static Telemetry telemetry;

    public static void setTelemetry(Telemetry tel){
        telemetry = tel;
    }

    public static Telemetry getTelemetry(){
        return  telemetry;
    }

}
