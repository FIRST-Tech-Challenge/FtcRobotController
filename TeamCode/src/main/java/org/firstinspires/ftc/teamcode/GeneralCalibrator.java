package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.sun.tools.javac.util.Pair;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@TeleOp
public class GeneralCalibrator extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        String[] names = {
                "grabber",
                "puffer",
                "placerRight",
                "armRight",
                "grabberRight"
        };

        ServoSystem[] systems = {
                new ServoSystem(0, 1, new Pair<>(names[0], false)),
                new ServoSystem(0, 1, new Pair<>(names[1], false)),
                new ServoSystem(0, 1, new Pair<>(names[2], true), new Pair<String, Boolean>("placerLeft" , false)),
                new ServoSystem(0, 1, new Pair<>(names[3], false), new Pair<String, Boolean>("armLeft"    , true)),
                new ServoSystem(0, 1, new Pair<>(names[4], false), new Pair<String, Boolean>("grabberLeft", true))
        };

        int i = 0;


        while (opModeIsActive() && !gamepad1.a){
            if (B_Pressed(gamepad1.b)) {
                i++;
            }
            if (X_Pressed(gamepad1.x)){
                i--;
            }
            i = (i + systems.length) % systems.length;

            telemetry.addLine(names[i]);
            telemetry.update();
        }
        sleep(500);

        while (opModeIsActive()){
            try {
                calibrate(-gamepad1.left_stick_y, gamepad1.a, telemetry, systems[i].servos);
            } catch (InterruptedException e){
                telemetry.addLine("stopped");
                telemetry.update();
            }
        }
    }


    public double position = 0;
    public double sensitivity = 1;
    public void calibrate(double wantedPosition, boolean controllerButton, Telemetry output, ArrayList<Servo> servos) throws InterruptedException{
        wantedPosition *= sensitivity;
        wantedPosition += position;

        for (Servo servo : servos) {
            servo.setPosition(wantedPosition);
        }

        if (A_Pressed(controllerButton)) {
            position = wantedPosition;
            sensitivity /= 2;
            Thread.sleep(500);
        }

        output.addData("position", wantedPosition);
        output.update();
    }

    private static boolean A_wasPressedLastTime = true;
    public static boolean A_Pressed(boolean button) {
        if (button) {
            if (A_wasPressedLastTime) {
                A_wasPressedLastTime = false;
                return true;
            }
        } else {
            A_wasPressedLastTime = true;
        }
        return false;
    }
    private static boolean B_wasPressedLastTime = true;
    public static boolean B_Pressed(boolean button) {
        if (button) {
            if (B_wasPressedLastTime) {
                B_wasPressedLastTime = false;
                return true;
            }
        } else {
            B_wasPressedLastTime = true;
        }
        return false;
    }
    private static boolean X_wasPressedLastTime = true;
    public static boolean X_Pressed(boolean button) {
        if (button) {
            if (X_wasPressedLastTime) {
                X_wasPressedLastTime = false;
                return true;
            }
        } else {
            X_wasPressedLastTime = true;
        }
        return false;
    }

    private class ServoSystem{
        // min and max positions
        public double minPosition;
        public double maxPosition;

        public ArrayList<Servo> servos;

        public ServoSystem(double minPosition, double maxPosition, Pair<String, Boolean>... namesAndToFlip){
            servos = new ArrayList<>();

            for(Pair<String, Boolean> nameAndToFlip : namesAndToFlip){

                servos.add(hardwareMap.servo.get(nameAndToFlip.fst));

                if (nameAndToFlip.snd){
                    servos.get(servos.size() - 1).setDirection(Servo.Direction.REVERSE);
                }
            }

            this.minPosition = minPosition;
            this.maxPosition = maxPosition;
        }
    }
}
