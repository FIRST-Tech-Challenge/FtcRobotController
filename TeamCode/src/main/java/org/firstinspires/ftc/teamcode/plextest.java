package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
class plextest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor fr = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor bl = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor br = hardwareMap.get(DcMotor.class, "motorBR");
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        MovementClass move = new MovementClass(this);
        waitForStart();
//        while(opModeIsActive()){
//            if(gamepad1.dpad_down)
//                move.back(1);
//            if(gamepad1.dpad_up)
//                move.forward(1);
//            if(gamepad1.dpad_left)
//                move.left(1);
//            if(gamepad1.dpad_right)
//                move.right(1);
//            if(gamepad1.x)
//                move.turnleft(1);
//            if(gamepad1.b)
//                move.turnright(1);
//            if(gamepad1.dpad_left && gamepad1.dpad_up)
//                move.diagonalleftfront(1);
//            if(gamepad1.dpad_left && gamepad1.dpad_down)
//                move.diagonalleftback(1);
//            if(gamepad1.dpad_right && gamepad1.dpad_up)
//                move.diagonalrightfront(1);
//            if(gamepad1.dpad_down && gamepad1.dpad_right)
//                move.diagonalrightback(1);
//            else
//                move.stop();
        switch(gamepad1){

        }
            //motorBL - spate stanga
            //motorBR - spate dreapta
            //motorFR - fata dreapta
            //motorFL - fata stanga
        }

    }

}