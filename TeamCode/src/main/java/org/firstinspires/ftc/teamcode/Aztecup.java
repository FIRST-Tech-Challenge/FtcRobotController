package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Aztecup")
public class Aztecup extends LinearOpMode {

    private DcMotor motorderecho;
    private DcMotor motorizquierdo;

    @Override
    public void runOpMode() {

        initHardware();
        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            loopOperations();
        }
    }

    public void initHardware() {
        motorizquierdo  = hardwareMap.get(DcMotor.class, "motor izquierdo");
        motorderecho = hardwareMap.get(DcMotor.class, "motor derecho");


        motorderecho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorizquierdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorderecho.setDirection(DcMotor.Direction.REVERSE);
        motorizquierdo.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loopOperations() {

        double motor_D = -gamepad1.left_stick_y;
        double motor_I = -gamepad1.left_stick_y;
        double giro = gamepad1.right_stick_x;

        double potenciaIzquierda = motor_I + giro;
        double potenciaDerecha = motor_D - giro;

        motorizquierdo.setPower(potenciaIzquierda);
        motorderecho.setPower(potenciaDerecha);

        telemetry.addData("Motor Izquierdo", "%.2f", potenciaIzquierda);
        telemetry.addData("Motor Derecho", "%.2f", potenciaDerecha);
        telemetry.update();
    }


}
