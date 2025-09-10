
package org.firstinspires.ftc.teamcode.Tools;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;


import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Constants {
    public static final double PID_X_P = 0.1;
    public static final double PID_X_I = 1;
    public static final double PID_X_D = 1;
    public static final double PID_Y_P = 0.1;
    public static final double PID_Y_I = 1;
    public static final double PID_Y_D = 1;
    public static final double PID_THETA_P = 0.1;
    public static final double PID_THETA_I = 1;
    public static final double PID_THETA_D = 1;
    public static double absoluteArmZero =0.306;
    public static double armOffset;
    public static double ArmUpPosition = 4000;
    public static double ElevatorsUpPosition = 2000;
    public static double ElevatorsDownPosition = 200;
    public static double ArmDownPosition = 200;
    public static Object SetPoints;
    public static PID pivotPID = new PID( 0.006, 0.001, 0);
    public static PID slowedPivotPID = new PID( 0.006, 0.001, 0);

    public static PID elevatorPID = new PID(0.01 ,0,0);
    public static double nextX;
    public static double nextY;
    public static double nextTheta;
    public static final double MAX_TICKS = -2090;
    public static final double MIN_TICKS = 31;
    public static final double DEAD_ZONE = 0.1;

    public static double DegreesToEncoderTicks(double degrees) {
        double TPR = 5700.4;
        return ((degrees / (360) * TPR));
    }

    public static double EncoderTicksToDegrees(double encoders) {
        double TPR = 5700.4;
        return (encoders * (360) / 5700.4);
    }
    public static double InchesToEncoderTicks(double inches) {
        double circumference = 4.71238898038;
        double ticksPerRotation = 28;
        double gearRatio = 1;
        return ((inches / circumference) * ticksPerRotation * gearRatio);
    }


    public static double EncodersTicksToDegrees(double encoders) {
        return -((encoders) * (360) / 5700.4);
    }

    public static void BRAKE(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public static double ServoInputToDegrees(double degrees) {
        final double MIN_PWM = 1000;
        final double MAX_PWM = 2000;
        final double MIN_DEGREES = 0;
        final double MAX_DEGREES = 300;

        // Map servo input to degrees
        return MIN_DEGREES + (degrees - MIN_PWM) * (MAX_DEGREES - MIN_DEGREES) / (MAX_PWM - MIN_PWM);
    }



    public static double GravityTerm(double degreesInput) {



        double degrees = degreesInput+90;
        double gravityConstant = 2;
        double gravityForce = gravityConstant * Math.cos(Math.toRadians(degrees));

        System.out.println("gravity"+gravityForce);

        return Math.abs(gravityForce);
    }


    // Static variable to maintain toggle state across calls

    public static double getDegrees(double getPosition) {
        return -((getPosition / (1333/90) + 21));
    }

    public static double setPowerToPercentage(double percentage) {
        if (percentage > 1) {
            percentage = 100;
        } else if (percentage < -1) {
            percentage = -100;
        }
        return percentage / 100;
    }

    public static double getOffsetFromVoltage(double voltage){
        return 5.03 + -4950*voltage + -4731*Math.pow(voltage, 2) + -2098*Math.pow(voltage, 3) + -286*Math.pow(voltage, 4);
    }
    public static double yCorrected(double AY) {
        return AY - ((0.172 * AY) + 0.00307);
    }

    public static double xCorrected(double AX) {
        return AX - (0.181 * AX + -0.00049);
    }

    public static HashMap<String, Supplier<Command>> commandMap = new HashMap<>();
    public static HashMap<String, BooleanSupplier> conditionMap = new HashMap<>();

    public static class AprilTagData {
        public double positionX;
        public double positionY;
        public double size;
        public double tagangle;

        public AprilTagData(double positionX, double positionY, double size, double tagangle) {

            this.positionY = positionY;
            this.size = size;
            this.tagangle = tagangle;
        }
    }

    public static double yOffset(double x) {
        return -0.276 + 0.394 * x + (-0.114 * x * x) + (0.01 * x * x * x);
    }

    public static final Map<Integer, AprilTagData> aprilTagMap = new HashMap<>();

    static {
        aprilTagMap.put(14, new AprilTagData(3.048, 3.66, 0.1016, 0));
        aprilTagMap.put(15, new AprilTagData(3.66, 1.83, 0.1016, 0));
        aprilTagMap.put(16, new AprilTagData(3.048, 0, 0.1016, 0));
    }

    public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = Math.PI;

}