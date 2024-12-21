package org.firstinspires.ftc.teamcode.Debugging;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Wrappers.Controller;
import org.firstinspires.ftc.teamcode.Systems.Deposit;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain;

@TeleOp
public class DepositTestingOpMode extends OpMode {
    private Hardware hardware = new Hardware();
    private Deposit deposit;
    private Drivetrain drivetrain;
    private Controller controller;


    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new Controller(gamepad1, Controller.xBox);
        deposit = new Deposit(hardware);
        drivetrain = new Drivetrain(hardware, controller);

    }

    @Override
    public void loop() {
        drivetrain.update();
        deposit.update();

        if (controller.getA()){

            deposit.goToTransfer();

        } else if (controller.getB()) {
            deposit.goToSpecIntake();

        } else if (controller.getY()) {
            deposit.goToSampleDeposit();
        } else if (controller.getX()) {


            if (deposit.getTargetState() != Deposit.TargetState.specDepositReady) {
                deposit.goToSpecDepositReady();
            } else if (deposit.getTargetState() == Deposit.TargetState.specDepositReady) {
                deposit.goToSpecClipped();
            }

        }

        drivetrain.command();
        deposit.command();

    }
}
