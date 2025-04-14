package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "aztecup")
public class aztecup extends LinearOpMode {

    private DcMotor motorderecho;
    private DcMotor motorizquierdo;

    @Override
    public void init() {

        motorizquierdo  = hardwareMap.get(DcMotor.class, "motor izquierdo");
        motorderecho = hardwareMap.get(DcMotor.class, "motor derecho");

        motorderecho.setDirection(DcMotor.RunMode.RUN_USING_ENCODER);
        motorizquierdo.setDirection(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData(">", "Robot Ready.  Press START.");    //
    }

    @Override
    public void loop() {
        double motor_D;
        double motor_I;
        double giro;

        motor_D = -gamepad1.left_stick_y;
        motor_I = -gamepad1.left_stick_y;
        giro = + gamepad1.right_stick_x;

        motorizquierdo.setPower(motor_I - giro);
        motorderecho.setPower(motor_D + giro);

        telemetry.addData("izquierda",  "%.2f", motor_I);
        telemetry.addData("derecha", "%.2f", motor_D);
    }


    @Override
    public void stop() {
    }
}
