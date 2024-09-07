package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public abstract class AbstractRobot {
    public ArrayList<AbstractSubsystem> subsystems;
    public final OpMode opMode;
    public final Telemetry telemetry;

    public void onInit(){
    }

    public AbstractRobot(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        subsystems = new ArrayList<>();

    }
    public void driverLoop()
    {
        for(AbstractSubsystem  system : subsystems)
        {
            system.driverLoop();
        }
    }
}
