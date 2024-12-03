package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.JarlsCHasse;

@Autonomous
public class Jarlsnerg_odometry extends OpMode{

    JarlsCHasse drivetrain;
    Odometry_Info odom;

    @Override
    public void init() {

        drivetrain = new JarlsCHasse(hardwareMap);
        odom = new Odometry_Info(hardwareMap);

        odom.setTargetPos(400, 0, 90);
        odom.resetEncoders();



    }

    public void driveToTarPos(Odometry_Info odop) {
            if (odop.cur0 != odop.tar0) {
                drivetrain.turn0Clockwise(true);
            }
            else{
                drivetrain.rXmov = 0;
            }
            if (odop.curX != odop.tarX) {
                if (odop.curX < odop.tarX) {
                    drivetrain.moveX(true, odop.cur0);
                }
                else if(odop.curX > odop.tarX){
                    drivetrain.moveX(false, odop.cur0);
                }
                else{
                    drivetrain.Xmov = 0;
                }
            }
            if (odop.curY != odop.tarY) {
                if (odop.curY < odop.tarY) {
                    drivetrain.moveY(true, odop.cur0);
                }
                else if (odop.curY > odop.tarY) {
                    drivetrain.moveY(false, odop.cur0);
                }
                else{
                    drivetrain.Ymov = 0;
                }
            }

        if(odop.curPosIsTarPos()) {
            drivetrain.HALT();
        }
    }

    @Override
    public void loop() {

        telemetry.addData("X target", odom.tarX);
        odom.updateCurPos();
        drivetrain.coordinateBasedState();
        driveToTarPos(odom);
    }

}
