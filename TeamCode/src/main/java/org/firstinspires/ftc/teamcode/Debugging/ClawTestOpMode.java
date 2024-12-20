package org.firstinspires.ftc.teamcode.Debugging;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Wrappers.Controller;
import org.firstinspires.ftc.teamcode.Systems.Claw;

@TeleOp
public class ClawTestOpMode extends OpMode {

    private Hardware hardware = new Hardware();
    private Controller controller;

    private Claw claw;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new Controller(gamepad1, Controller.xBox);
        claw = new Claw(hardware);
        claw.servo.setPos(claw.openPos);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        if (controller.getRB()){
            claw.servo.setPos(claw.closedPos);
        } else {
            claw.servo.setPos(claw.openPos);
        }

        telemetry.addData("Encoder Voltage", claw.servo.getPos());
    }

}
