package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.JarlsCHasse;

@Autonomous
public class Jarlsnerg_odometry extends OpMode {

    JarlsCHasse drivetrain = null;
    Odometry_Info odo = null;

    public void driveToTarPos() {

        while (odo.curPosIsTarPos()) {

            if (odo.cur0 != odo.tar0) {
                drivetrain.turn0Clockwise();
            }
            if (odo.curX != odo.tarX) {
                if (odo.curX <= odo.tarX) {
                    drivetrain.moveX(true);
                }
                if (odo.curX >= odo.tarX) {
                    drivetrain.moveX(false);
                }
            }
            if (odo.curY != odo.tarY) {
                if (odo.curY <= odo.tarY) {
                    drivetrain.moveY(true);
                }
                if (odo.curY >= odo.tarY) {
                    drivetrain.moveY(false);
                }
            }

        }
    }

    @Override
    public void init() {

        JarlsCHasse driveTrain = new JarlsCHasse(hardwareMap);
        Odometry_Info odo = new Odometry_Info(hardwareMap);

    }

    @Override
    public void loop() {



    }

}
