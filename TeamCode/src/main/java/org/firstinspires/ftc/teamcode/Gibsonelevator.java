package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name="elevatorGib")
public class Gibsonelevator extends LinearOpMode{
    // Definitions
    DcMotor m_elevator;

    @Override
    public void runOpMode() {
        m_elevator = hardwareMap.get(DcMotor.class, "elevator");
        m_elevator.setDirection(DcMotor.Direction.FORWARD);
        m_elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("status", "running");
            telemetry.addData("position", m_elevator.getCurrentPosition());
            telemetry.update();

            double y = -gamepad1.left_stick_y;

            if(y != 0){
                m_elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m_elevator.setPower(y);
            }
            else if(gamepad1.a || gamepad1.b){

                if(gamepad1.a){
                    //m_elevator.setTargetPosition();
                }
                else if(gamepad1.b) {
                    //m_elevator.setTargetPosition();
                }

                m_elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m_elevator.setPower(0.5);
            }
            else {
                m_elevator.setPower(0);
                }
            }
        }
    }
