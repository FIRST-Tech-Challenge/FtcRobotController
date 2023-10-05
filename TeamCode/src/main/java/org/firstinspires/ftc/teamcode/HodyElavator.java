package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="HodyElavator")
public class HodyElavator extends LinearOpMode{
    // Definitions
    DcMotor m_elevator;

    @Override
    public void runOpMode() {
        m_elevator = hardwareMap.get(DcMotor.class, "elevator");
        m_elevator.setDirection(DcMotor.Direction.FORWARD);
        m_elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive()) {
            telemetry.addData("status", "running");
            telemetry.addData("status", m_elevator.getCurrentPosition());
            telemetry.update();


            double y = -gamepad1.left_stick_y;


            if (y != 0) {
                m_elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m_elevator.setPower(y);

            } else if (gamepad1.a) {
                m_elevator.setTargetPosition(0);


                if (gamepad1.a) {
                    m_elevator.setTargetPosition(0);
                    m_elevator.setPower(.5);


                }
            }
        }
    }
}
