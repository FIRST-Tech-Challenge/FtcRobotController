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
        telemetry.addData("Distance (cm)", board.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (in)", board.getDistance(DistanceUnit.INCH));
    }
}