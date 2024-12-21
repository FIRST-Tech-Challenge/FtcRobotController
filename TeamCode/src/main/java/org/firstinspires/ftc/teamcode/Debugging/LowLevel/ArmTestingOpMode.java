package org.firstinspires.ftc.teamcode.Debugging.LowLevel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Wrappers.Controller;
import org.firstinspires.ftc.teamcode.Systems.Mechaisms.Arm;

@TeleOp
public class ArmTestingOpMode extends OpMode {

    private Hardware hardware = new Hardware();
    private Controller controller;
    private Arm arm;

    private double targetPosition = 0;


    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new Controller(gamepad1, Controller.xBox);
        arm = new Arm(hardware);
        arm.setPosition(targetPosition);
    }

    @Override
    public void loop() {

        if (controller.getA()){
            targetPosition = DepositConstants.armRightTransferPos;
        } else if (controller.getB()){
            targetPosition = DepositConstants.armRightSpecDepositPos;
        } else if (controller.getX()) {
            targetPosition = DepositConstants.armRightSampleDepositPos;
        }   else if (controller.getY()) {
            targetPosition = DepositConstants.armRightSpecIntakePos;
        } else if (controller.getRB()) {
            targetPosition = 0;
        }
        arm.setPosition(targetPosition);

        telemetry.addData("Right Encoder Position", arm.getPosition());
        telemetry.update();
        }

    }


