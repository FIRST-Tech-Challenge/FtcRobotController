package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;

import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

public abstract class AbstractOpMode extends GamepadOpMode {
    private AbstractRobot robot;
    public abstract AbstractRobot instantiateRobot();

    public CuttleRevHub ctrlHub;
    public CuttleRevHub expHub;

    public CuttleMotor leftFrontMotor ;
    public CuttleMotor rightFrontMotor;
    public CuttleMotor rightBackMotor ;
    public CuttleMotor leftBackMotor  ;
    public MecanumController drive;

    public AbstractRobot getRobot() {
        return robot;
    }

    public void onInit() {
        leftFrontMotor  = ctrlHub.getMotor(1);
        rightFrontMotor = ctrlHub.getMotor(2);
        leftBackMotor   = ctrlHub.getMotor(3);
        rightBackMotor  = ctrlHub.getMotor(4);

        leftBackMotor .setDirection(Direction.REVERSE);
        leftFrontMotor.setDirection(Direction.REVERSE);

        drive = new MecanumController(rightFrontMotor, rightBackMotor, leftFrontMotor, leftBackMotor);
    }

    public void onStop() {
        super.stop();
    }
}
