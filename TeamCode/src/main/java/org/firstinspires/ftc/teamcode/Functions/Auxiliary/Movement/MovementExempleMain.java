package org.firstinspires.ftc.teamcode.Functions.Auxiliary.Movement;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.functions.auxiliary.AuxiliaryTemplate;

@TeleOp(name="MovementExempleMain", group="TEST")
public class MovementExempleMain extends OpMode {

    DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    Movement movementExample;
    @Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("FL");
        rightFrontMotor = hardwareMap.dcMotor.get("FR");
        leftFrontMotor = hardwareMap.dcMotor.get("FL");
        rightFrontMotor = hardwareMap.dcMotor.get("FR");
        movementExample = new Movement(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up){
            movementExample.MoveFull(MovementFunction.Move.UP);
        }
        else if(gamepad1.dpad_down){
            movementExample.MoveFull(MovementFunction.Move.DOWN);
        }

    }
}
