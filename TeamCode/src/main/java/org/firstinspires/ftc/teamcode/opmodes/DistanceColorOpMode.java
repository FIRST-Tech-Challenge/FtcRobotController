package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmoe.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeeOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard7;

@TeleOp()
public class DistanceColorOpMode extends OpMode {
    ProgrammingBoard7 board = new ProgrammingBoard7();
    @Override
    public void init() {
        board.init(hardware);
    }

    @Override
    public void loop() {
        telemetry.addData("Amount red", board.getAmountRed());


        telemetry.addData("Amount green", board.getAmountGreen());

        telemetry.addData("Amount blue", board.getAmountBlue());
        telemetry.addData("Distance (cm)", board.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (in)", board.getDistance(DistanceUnit.INCH));

        int green = board.getAmountGreen()/(board.getAmountGreen()+board.getAmountBlue()+board.getAmountRed());
        int blue = board.getAmountBlue()/(board.getAmountGreen()+board.getAmountBlue()+board.getAmountRed());
        int red = board.getAmountRed()/(board.getAmountGreen()+board.getAmountBlue()+board.getAmountRed());
        

        if (green > blue && green > red && green >= .60) {
            telemetry.addData("Color is", "Green");
        }
        else if (red > blue && red > green) {
            telemetry.addData("Color is", "Red");
        }
        else if () {
            telemetry.addData("Color is", "Blue");
        }
    }
}