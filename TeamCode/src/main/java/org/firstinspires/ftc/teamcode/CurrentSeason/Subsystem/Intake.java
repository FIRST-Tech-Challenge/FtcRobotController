package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.PeppyFeetFiend;

public class Intake extends AbstractSubsystem {
    PeppyFeetFiend robot;
    CuttleMotor motor;
    CuttleServo servo;
CuttleRevHub ctrlHub;
    public Intake(AbstractRobot robot, int motorPort, int servoPort) {
        super(robot);
        this.robot = (PeppyFeetFiend) robot;
        ctrlHub = new CuttleRevHub(hardwareMap,CuttleRevHub.HubTypes.CONTROL_HUB);

        motor =  ctrlHub.getMotor(motorPort);
        servo = ctrlHub.getServo(servoPort);

    }

    @Override
    public void init() {
        motor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        motor.setPower(gamepad2.right_stick_y);

    }

    @Override
    public void driverLoop() {
        //use for telemetry
    }

    @Override
    public void stop() {

    }
}