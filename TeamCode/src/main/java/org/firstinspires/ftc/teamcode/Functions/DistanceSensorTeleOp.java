package org.firstinspires.ftc.teamcode.Functions;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Functions.DistanceSensorFunction;

public class DistanceSensorTeleOp {

    @TeleOp(name = "DistanceSeonsorTeleOp", group = "TEST")

    public class DistanceSensorTOp extends OpMode {

        public void init() {
        }



        @Override
        public void loop() {
            double valoare = 1;

            telemetry.addData("Distanta: ",valoare);
        }

        @Override
        public void stop() {

        }
    }
}