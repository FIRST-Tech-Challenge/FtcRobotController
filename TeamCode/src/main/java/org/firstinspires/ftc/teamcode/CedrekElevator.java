package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Empty Opmode")
public class CedrekElevator extends LinearOpMode{
    DcMotor m_elevator;
    // Definitions

    @Override
    public void runOpMode() {

        m_elevator = hardwareMap.get(DcMotor.class, "elevator");
        m_elevator.setDirection(DcMotor.Direction.FORWARD);
        m_elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // m_elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initialization code
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Position", m_elevator.getCurrentPosition());
            telemetry.update();

            double y = -gamepad1.left_stick_y;

            if(y != 0){
                m_elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m_elevator.setPower(y);
            }
            else if (gamepad1.a || gamepad1.b){

                if (gamepad1.a) {
                    //m_elevator.setTargetPosition();
                }
                else if (gamepad2.b) {
                    //m_elevator.setTargetPosition();
                }
                    m_elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    m_elevator.setPower(0.5);
                }
                else{
                    m_elevator.setPower(0);
                }



            }

        }
        } 

