package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;
import java.util.List;


public class Movement extends OpMode {
    //lf rf lb rb
    public enum Direction {
        FORWARD(new int[]{1,1,1,1}),
        BACKWARD(new int[]{-1,-1,-1,-1}),
        LEFT(new int[]{-1,1,1,-1}),
        RIGHT(new int[]{1,-1,-1,1});


        private int[] dir;
        Direction(int[] dir){
            this.dir = dir;
        }

    };


    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;

    Gamepad pad;

    DcMotor[] motors = new DcMotor[4];

    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class,"lf");
        rf = hardwareMap.get(DcMotor.class,"rf");
        lb = hardwareMap.get(DcMotor.class,"lb");
        rb = hardwareMap.get(DcMotor.class,"rb");
        motors = new DcMotor[]{lf,rf,lb,rb};
    }

    @Override
    public void loop() {
        float speedCorrection = (float)Math.sqrt(Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y,2));
        //-> normalize : 대각선 이동시 이동 속도 조절
        move(Direction.FORWARD,gamepad1.left_stick_y / speedCorrection);
        move(Direction.RIGHT,gamepad1.left_stick_x / speedCorrection);
    }

    public void move(Direction direction, float speed){
        for(int i = 0; i < 4; i++){
            motors[i].setPower(direction.dir[i] * speed);
        }
    }
}
