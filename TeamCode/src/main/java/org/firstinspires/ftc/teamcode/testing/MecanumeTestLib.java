package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class MecanumeTestLib extends LinearOpMode {

    Motor lf,lr,rf,rr;
    MecanumDrive drive;
    GamepadEx driverOp;
    Path ceva;

    @Override
    public void runOpMode() throws InterruptedException {

        try{

            lf = new Motor(hardwareMap, "leftFront");
            lr = new Motor(hardwareMap, "leftRear");
            rf = new Motor(hardwareMap, "rightFront");
            rr = new Motor(hardwareMap, "rightRear");

            drive = new MecanumDrive(lf, rf, lr, rr);
            driverOp = new GamepadEx(gamepad1);

            waitForStart();

            double speed = -1;

            while(!isStopRequested()){
                drive.driveRobotCentric(
                    driverOp.getLeftX()*speed,
                    driverOp.getLeftY()*speed,
                    driverOp.getRightX()*speed
                );
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){

        }
    }
}
