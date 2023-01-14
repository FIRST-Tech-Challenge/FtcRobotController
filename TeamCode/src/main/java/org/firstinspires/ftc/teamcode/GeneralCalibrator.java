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

        ServoSystem[] systems = {
                new ServoSystem(0, 1, new Pair<>("grabber", false)),
                new ServoSystem(0, 1, new Pair<>("puffer", false)),
                new ServoSystem(0, 1, new Pair<>("placerRight", true), new Pair<String, Boolean>("placerLeft" , false)),
                new ServoSystem(0, 1, new Pair<>("armRight", false), new Pair<String, Boolean>("armLeft"    , true)),
                new ServoSystem(0, 1, new Pair<>("grabberRight", true), new Pair<String, Boolean>("grabberLeft", false))
        };

        int i = 0;


        while (opModeIsActive() && gamepad1.a){
            if (firstPress(gamepad1.right_bumper)){
                i++;
            } else if (firstPress(gamepad1.left_bumper)){
                i--;
            }
            i = (i + systems.length) % systems.length;

            for (Servo servo : systems[i].servos) {
                telemetry.addLine(servo.getDeviceName());
            }
            telemetry.update();
        }

        while (opModeIsActive()){
            try {
                calibrate(-gamepad1.left_stick_y, gamepad1.a, telemetry, systems[i].servos);
            } catch (InterruptedException e){
                telemetry.addLine("stopped");
                telemetry.update();
            }
        }
    }


        public static double position = 0;
    public static double sensitivity = 1;
    public static void calibrate(double wantedPosition, boolean controllerButton, Telemetry output, ArrayList<Servo> servos) throws InterruptedException{
        wantedPosition *= sensitivity;
        wantedPosition += position;

        for (Servo servo : servos) {
            servo.setPosition(wantedPosition);
        }

        if (firstPress(controllerButton)) {
            position = wantedPosition;
            sensitivity /= 2;
            Thread.sleep(500);
        }

        output.addData("position", wantedPosition);
        output.update();
    }

    private static boolean wasPressedLastTime = true;
    public static boolean firstPress(boolean button) {
        if (button) {
            if (wasPressedLastTime) {
                wasPressedLastTime = false;
                return true;
            }
        } else {
            wasPressedLastTime = true;
        }
        return false;
    }

    private class ServoSystem{
        // min and max positions
        public double minPosition;
        public double maxPosition;

        public ArrayList<Servo> servos;

        public ServoSystem(double minPosition, double maxPosition, Pair<String, Boolean>... namesAndToFlip){
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
