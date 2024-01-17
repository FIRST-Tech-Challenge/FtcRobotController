package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev.Zayne;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

@Disabled
@TeleOp
public class ZayneTeleop extends LinearOpMode {

    private DcMotor[] motorList;
    private IMU gamePad;//Just name it imu // zanye - nuh uh
    private ZayneTest motorPairing;//I would just call it something like zayne // zanye

    @Override
    public void runOpMode() throws InterruptedException{
        Gamepad g1 = new Gamepad();

        motorList = new DcMotor[] {

                hardwareMap.dcMotor.get("L_Motor_1"),//Why is this the same name as below?
                hardwareMap.dcMotor.get("R_Motor_1"),
                hardwareMap.dcMotor.get("L_Motor_2"),//Just call them Motor 1 through 4 or something like that
                hardwareMap.dcMotor.get("R_Motor_2")

        };

        gamePad = hardwareMap.get(IMU.class, "gamePad");
        motorPairing =  new ZayneTest(motorList, gamePad);

        waitForStart();

        if(isStopRequested()) {//Can be if(isStopRequested()) return; // ye but thats a going arround way of righting it bc isStopRequest is a true or false value, so you dont need it as if will just give the same true or false value
            return;
        }

        while (opModeIsActive()){
           motorPairing.movo(g1.left_stick_x, g1.left_stick_y, g1.right_stick_x);//Swap left_stick_x and left_stick_y // kk
        }
    }
}
