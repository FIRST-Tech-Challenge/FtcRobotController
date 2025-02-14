package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp
public class ScissorsMisumiTest extends OpMode {
    public DcMotor scissors;
    public int OUT = 32;
    public int IN = 1;
    public enum SCISSORS_STATE {
        IN,
        OUT,
        OFF
    }
    public SCISSORS_STATE state;
    public SCISSORS_STATE last_state;
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
        else if(state == SCISSORS_STATE.OFF){
            scissors.setPower(0);
        }

        if(gamepad1.x && state != SCISSORS_STATE.OFF){
            last_state = state;
            state = SCISSORS_STATE.OFF;
        }
        else {
            state = last_state;
        }
        telemetry.addData("Encoder position: ", scissors.getCurrentPosition());
        telemetry.addData("Direction: ", gamepad1.left_stick_y);
    }
}
