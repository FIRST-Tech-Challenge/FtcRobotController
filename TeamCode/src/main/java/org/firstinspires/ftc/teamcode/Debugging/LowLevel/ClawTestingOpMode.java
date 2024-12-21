package org.firstinspires.ftc.teamcode.Debugging.LowLevel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Wrappers.Controller;
import org.firstinspires.ftc.teamcode.Systems.Mechaisms.Claw;

@TeleOp
public class ClawTestingOpMode extends OpMode {

    private Hardware hardware = new Hardware();
    private Controller controller;

    private Claw claw;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new Controller(gamepad1, Controller.xBox);
        claw = new Claw(hardware);
        claw.setPosition(DepositConstants.clawOpenPos);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        if (controller.getRB()){
            claw.setPosition(DepositConstants.clawClosedPos);
        } else {
            claw.setPosition(DepositConstants.clawOpenPos);
        }

        telemetry.addData("Encoder Voltage", claw.getPosition() );
    }

}
