package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.ArmSystem.Intake.State.INTAKING;
import static org.firstinspires.ftc.teamcode.components.ArmSystem.Intake.State.OUTTAKING;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ArmSystem;

@Autonomous (name = "arm", group = "Autonomous")
public class ArmOpMode extends OpMode {

    DcMotor motor1;
    DcMotor motor2;

    /** Initialization */
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class,"arm_right");
        motor2 = hardwareMap.get(DcMotor.class,"arm_left");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop(){

        telemetry.addData("Motor1: ", motor1.getCurrentPosition());
        telemetry.addData("Motor2: ", motor2.getCurrentPosition());
        telemetry.update();

    }

}

