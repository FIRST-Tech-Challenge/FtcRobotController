package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Constants {

    public static final class DriveTrainConstants {

        public static final String frontLeftMotor = "frontLeft0";
        public static final String frontRightMotor = "frontRight1";
        public static final String backLeftMotor = "backLeft2";
        public static final String backRightMotor = "backRight3";

        public static final double strafingBalancer = 1.1;

        public static final double ticksPerRevolution = 537.6; //Ticks per revolution on the NeveRest Orbital 20 Gearmotor

        public static final double controlHubOffset = 0;
    }

    public static final class WristConstants {
        public static final String leftWrist2 = "leftWrist2";
        public static final String rightWrist3 = "rightWrist3";
        public static final double intakeAngle = 0.05;
        public static final double transferAngle = 0;
        public static final double barAngle = 0.25;
        public static final Servo.Direction wristInvertL = Servo.Direction.FORWARD;
        public static final Servo.Direction wristInvertR = Servo.Direction.REVERSE;
    }

    public static final class IntakeConstants {

        public static final String leftIntake0 = "leftIntake0";
        public static final String rightIntake1 = "rightIntake1";
    }

    public static final class ExtensionConstants {
        public static final String extension = "extension3";
        public static final double retracted = 0.5; //TODO FUIND THIS VALUE
        public static final double extended = 0;  //TODO FIND THIS VALUE
    }

    public static final class ArmConstants {
        public static final String arm = "arm4";
        public static final double exchangeAngle = 0.99; //TODO FIND THIS VALUE
        public static final double dropAngle = 0.4; //TODO FIND THIS VALUE
    }

    public static final class ClawConstants {
        public static final String claw = "claw5";
        public static final double open = 1;    //TODO FIND THIS
        public static final double closed = 0;  //TODO FIND THIS TANGLE TOO
    }

    public static final class ElevatorConstants {
        public static final String elevator = "elevator0";
        public static final int highBasket = 2000;
        public static final int lowBasket = 0;
        public static final int exchange = 0;
    }

}

