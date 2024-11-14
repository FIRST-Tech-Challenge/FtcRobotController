package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ScissorsMisumiTest extends OpMode {
    public DcMotor scissors;
    public int OUT = 32;
    public int IN = 1;
    public enum SCISSORS_STATE {
        IN,
        OUT
    }
    public SCISSORS_STATE state;
    public ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        scissors = hardwareMap.get(DcMotor.class, "scissors");
        state = SCISSORS_STATE.IN;
    }

    @Override
    public void loop() {
        if(timer.seconds() > 2 && state == SCISSORS_STATE.IN){
            scissors.setPower(0.6);
            scissors.setTargetPosition(OUT);
            scissors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            state = SCISSORS_STATE.OUT;
            timer.reset();
        }
        else if(timer.seconds() > 2 && state == SCISSORS_STATE.OUT){
            scissors.setPower(0.9);
            scissors.setTargetPosition(IN);
            state = SCISSORS_STATE.IN;
            scissors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            timer.reset();
        }

        telemetry.addData("Encoder position: ", scissors.getCurrentPosition());
        telemetry.addData("Direction: ", gamepad1.left_stick_y);
    }
}
