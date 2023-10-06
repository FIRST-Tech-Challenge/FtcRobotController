package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name= "EverettElevator")
public class EverettElevator extends LinearOpMode{
    // Definitions
     DcMotor m_elevator;

    @Override
    public void runOpMode() {

        while (opModeIsActive()) {
        }m_elevator = hardwareMap.get(DcMotor.class, "elevator");
        m_elevator.setDirection(DcMotor.Direction.FORWARD);
        m_elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Running");
        telemetry.addData("Position", m_elevator.getCurrentPosition());
        telemetry.update();
        double y = gamepad1.left_stick_y;

        if(y != 0){
            m_elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m_elevator.setPower(y);
    }
        else{
            m_elevator.setPower(0);
        }
}
