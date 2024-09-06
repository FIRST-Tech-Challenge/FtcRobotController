package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public abstract class AbstractRobot {
    public ArrayList<AbstractSubsystem> subsystems;
    public final OpMode opMode;
    public final Telemetry telemetry;

    public CuttleRevHub ctrlHub;
    public CuttleRevHub expHub;

    public CuttleMotor leftFrontMotor ;
    public CuttleMotor rightFrontMotor;
    public CuttleMotor rightBackMotor ;
    public CuttleMotor leftBackMotor  ;
    public MecanumController chassis;

    public void onInit(){
        leftFrontMotor  = ctrlHub.getMotor(1);
        rightFrontMotor = ctrlHub.getMotor(2);
        leftBackMotor   = ctrlHub.getMotor(3);
        rightBackMotor  = ctrlHub.getMotor(4);

        leftBackMotor .setDirection(Direction.REVERSE);
        leftFrontMotor.setDirection(Direction.REVERSE);

        chassis = new MecanumController(rightFrontMotor,rightBackMotor,leftFrontMotor,leftBackMotor);
    }

    public AbstractRobot(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        subsystems = new ArrayList<>();

    }

}
