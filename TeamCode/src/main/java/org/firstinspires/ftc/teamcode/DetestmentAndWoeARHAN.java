package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class DetestmentAndWoeARHAN extends LinearOpMode {
    static private DcMotor FLW;
    static private DcMotor BLW;
    static private DcMotor FRW;
    static private DcMotor BRW;

    @Override
    public void runOpMode() throws InterruptedException {
        FLW = hardwareMap.get(DcMotor.class, "FLW");
        BLW = hardwareMap.get(DcMotor.class, "BLW");
        BRW = hardwareMap.get(DcMotor.class, "BRW");
        FRW = hardwareMap.get(DcMotor.class, "FRW");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        // Example of controlling movements and then stopping after each step.
        powerWheels(150, "left");
        powerWheels(900, "forward");
        //Place specimen
        powerWheels(20,"back");
        powerWheels(400, "right");
        powerWheels(30,"forward");
        powerWheels(20,"left");
        powerWheels(900,"backward");
        powerWheels(900,"forward");
        powerWheels(10,"right");
        powerWheels(900,"backward");
        turnRobot(200,"right");
        powerWheels(20,"right");
        //Pick up specimen
        turnRobot(200,"right");
        powerWheels(240,"left");
        powerWheels(900,"forward");
        //Place specimen
        powerWheels(10,"back");
        powerWheels(400,"right");
        powerWheels(20,"forward");
        powerWheels(20,"right");
        powerWheels(600,"backward");

        // Stop motors after completing the parking routine
        disablePower();
        telemetry.addData("Status", "Completed");
        telemetry.update();
    }



    public void powerWheels(int miliseconds, String direction) {
        switch (direction) {
            case "forward":
                FLW.setPower(0.3);
                BLW.setPower(0.3);
                FRW.setPower(0.3);
                BRW.setPower(-0.3);
                sleep(miliseconds);
                disablePower();
                break;
            case "backward":
                FLW.setPower(-0.3);
                BLW.setPower(-0.3);
                FRW.setPower(-0.3);
                BRW.setPower(0.3);
                sleep(miliseconds);
                disablePower();
                break;
            case "right":
                FLW.setPower(0.3);
                BLW.setPower(-0.3);
                FRW.setPower(-0.3);
                BRW.setPower(-0.3);
                sleep(miliseconds);
                disablePower();
                break;
            case "left":
                FLW.setPower(-0.3);
                BLW.setPower(0.3);
                FRW.setPower(0.3);
                BRW.setPower(0.3);
                sleep(miliseconds);
                disablePower();
                break;
        }
    }

    public void turnRobot(int miliseconds, String direction) {
        switch (direction) {
            case "left":
                FLW.setPower(-0.3);
                BLW.setPower(-0.3);
                FRW.setPower(0.3);
                BRW.setPower(-0.3);
                sleep(miliseconds);
                disablePower();
                break;
            case "right":
                FLW.setPower(0.3);
                BLW.setPower(0.3);
                FRW.setPower(-0.3);
                BRW.setPower(0.3);
                sleep(miliseconds);
                disablePower();
                break;
        }
    }

    public static void disablePower() {
        FLW.setPower(0);
        BLW.setPower(0);
        FRW.setPower(0);
        BRW.setPower(0);
    }

    public void updatePhoneConsole() {
        telemetry.addData("FLW Power", FLW.getPower());
        telemetry.addData("BLW Power", BLW.getPower());
        telemetry.addData("FRW Power", FRW.getPower());
        telemetry.addData("BRW Power", BRW.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
