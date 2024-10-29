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

        while (odo.curPosIsTarPos() == false) {

            if (odo.cur0 != odo.tar0) {
                drivetrain.turn0Clockwise(true);
            }
            else{
                drivetrain.rXmov = 0;
            }
            if (odo.curX != odo.tarX) {
                if (odo.curX <= odo.tarX) {
                    drivetrain.moveX(true, odo.cur0);
                }
                else if(odo.curX >= odo.tarX){
                    drivetrain.moveX(false, odo.cur0);
                }
                else{
                    drivetrain.Xmov = 0;
                }
            }
            if (odo.curY != odo.tarY) {
                if (odo.curY <= odo.tarY) {
                    drivetrain.moveY(true, odo.cur0);
                }
                else if (odo.curY >= odo.tarY) {
                    drivetrain.moveY(false, odo.cur0);
                }
                else{
                    drivetrain.Ymov = 0;
                }
            }

        }
        drivetrain.HALT();
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
