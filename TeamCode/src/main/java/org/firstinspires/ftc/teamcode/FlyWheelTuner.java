package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
class PIDVals {
    public static double kVal = 0;
    public static double kp = 40;
    public static double ki = 0.1;
    public static double kd = 0;
    public static double flywheelSpeed = 0.36;
}

@TeleOp(name = "FlyWheelTuner")
public class FlyWheelTuner extends OpMode {
    private Motor flywheel;
    private Motor hitter;
    private Motor intake1, intake2;
    private double direction = -1;
    private double flywheelSpeed = 0;
    private double intakeSpeed = 0;
    private boolean pressedA = false;
    private boolean pressedB = false;
    private boolean pressedX = false;
    private boolean pressedY = false;
    private boolean pressedRB = false;
    private boolean pressedUp = false;
    private boolean pressedDown = false;
    private boolean pressedLeft = false;
    private boolean pressedRight = false;

    private KChoice choice = KChoice.KP;
    String flywheelRunMode = "RAW";



//    FtcDashboard dashboard = FtcDashboard.getInstance();

    private enum KChoice {
        KP,
        KI,
        KD
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheel = new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setFeedforwardCoefficients(0, 0.03);
        flywheel.setVeloCoefficients(PIDVals.kp, PIDVals.ki, PIDVals.kd);

        hitter = new Motor(hardwareMap, "h");
        hitter.setRunMode(Motor.RunMode.PositionControl);
        hitter.setPositionCoefficient(0.004);
        hitter.setPositionTolerance(13.6);

        intake1 = new Motor(hardwareMap, "in1");
        intake2 = new Motor(hardwareMap, "in2");
        intake1.setRunMode(Motor.RunMode.VelocityControl);
        intake2.setRunMode(Motor.RunMode.VelocityControl);
        intake1.setVeloCoefficients(0.05, 0, 0);
        intake2.setVeloCoefficients(0.05, 0, 0);
    }

    @Override
    public void loop() {

        if(gamepad1.a) {
            pressedA = true;
        } else if(pressedA) {
            pressedA = false;
            choice = KChoice.KP;
        } else if(gamepad1.x) {
            pressedX = true;
        } else if(pressedX) {
            pressedX = false;
            choice = KChoice.KD;
        } else if(gamepad1.y) {
            pressedY = true;
        } else if(pressedY) {
            pressedY = false;
            choice = KChoice.KI;
        }

//        switch(choice) {
//            case KD:
//                PIDVals.kVal = PIDVals.kd;
//                break;
//            case KI:
//                PIDVals.kVal = PIDVals.ki;
//                break;
//            case KP:
//                PIDVals.kVal = PIDVals.kp;
//                break;
//        }

        if(gamepad1.b) {
            pressedB = true;
        } else if(pressedB) {
            if(flywheelSpeed != 0) {
                flywheel.setRunMode(Motor.RunMode.RawPower);
                flywheelRunMode = "RAW";
                flywheelSpeed = 0;
            } else {
                flywheel.setRunMode(Motor.RunMode.VelocityControl);
                flywheelRunMode = "VelocityControl";
                flywheelSpeed = PIDVals.flywheelSpeed;
            }
            pressedB = false;
        }

        if(gamepad1.left_bumper) {
            hitter.setTargetPosition(150);
        }
        else {
            hitter.setTargetPosition(0);
        }
        if(!hitter.atTargetPosition()) {
            hitter.set(0.7);
        } else {
            hitter.set(0);
        }

        if(gamepad1.right_trigger >= 0.1) {
            intakeSpeed = 0.7;
        } else if(gamepad1.left_trigger >= 0.1) {
            intakeSpeed = -0.7;
        } else {
            intakeSpeed = 0;
        }

        if(gamepad1.dpad_up) {
            pressedUp = true;
        } else if(pressedUp) {
            pressedUp = false;
            PIDVals.kVal += 0.001;
        } else if(gamepad1.dpad_down) {
            pressedDown = true;
        } else if(pressedDown) {
            pressedDown = false;
            PIDVals.kVal -= 0.001;
        } else if(gamepad1.dpad_left) {
            pressedLeft = true;
        } else if(pressedLeft) {
            pressedLeft = false;
            PIDVals.kVal -= 0.005;
        } else if(gamepad1.dpad_right) {
            pressedRight = true;
        } else if(pressedRight) {
            pressedRight = false;
            PIDVals.kVal += 0.005;
        }

//        switch(choice) {
//            case KD:
//                PIDVals.kd = PIDVals.kVal;
//                break;
//            case KI:
//                PIDVals.ki = PIDVals.kVal;
//                break;
//            case KP:
//                PIDVals.kp = PIDVals.kVal;
//                break;
//        }


        flywheel.setFeedforwardCoefficients(0, 0.03);
        flywheel.setVeloCoefficients(PIDVals.kp, PIDVals.ki, PIDVals.kd);

        double speed = direction * flywheelSpeed;

        flywheel.set(speed);

        telemetry.addData("Flywheel RunMode", flywheelRunMode);
        telemetry.addData("Flywheel Desired Speed", PIDVals.flywheelSpeed);
        telemetry.addData("Flywheel Speed", flywheel.get());
        telemetry.addData("Flywheel Set Speed", speed);
        telemetry.addData("Flywheel Velocity", flywheel.encoder.getRawVelocity());
        telemetry.addData("Flywheel Corrected Velocity", flywheel.getCorrectedVelocity());
        telemetry.addData("Flywheel Raw Velocity", ((DcMotorEx)flywheel.motor).getVelocity());
        telemetry.addData("Flywheel Position", flywheel.getCurrentPosition());
        telemetry.addData("Hitter Position", hitter.getCurrentPosition());
        telemetry.addData("FlyWheel PID", flywheel.getVeloCoefficients());
        telemetry.addData("kP", PIDVals.kp);
        telemetry.addData("kI", PIDVals.ki);
        telemetry.addData("kD", PIDVals.kd);


    }
}