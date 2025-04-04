package org.firstinspires.ftc.team13590;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.jetbrains.annotations.NotNull;

public class RRactions {
    RobotHardware robotHW;
    public RRactions(RobotHardware robot){
        robotHW = robot;
    }

    // elbow object class
    public class Elbow {
        // class vars
        private DcMotor elbow;

        // class constructor & hardware mapper
        public Elbow(HardwareMap hardwareMap){
            elbow = hardwareMap.get(DcMotor.class, "elbow_drive");
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbow.setDirection(DcMotor.Direction.REVERSE);
        }

        // actual action class/ do-er
        public class elbowToDegree implements Action {
            private boolean initialized = false;
            private double Tpos; // in counts
            public elbowToDegree(double angle){
                Tpos = angle*robotHW.ARM_COUNTS_PER_DEGREE;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.velocityElbowHandler(Tpos);
                return Math.abs(elbow.getCurrentPosition() - Tpos) > 10;
            }

        }

        // usable method/ action class shortcut
        public Action elbowToDeg(double angle){
            return new elbowToDegree(angle);
        }
    }

    public class Extender {
        // class vars
        private DcMotor extender;

        // class constructor & hardware mapper
        public Extender(HardwareMap hardwareMap){
            extender = hardwareMap.get(DcMotor.class, "extension_drive");
            extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extender.setDirection(DcMotor.Direction.FORWARD);
        }

        // actual action class/ do-er
        public class extenderToInches implements Action {
            private double Tpos; // in counts
            public extenderToInches(double inch){
                // return the smaller of the two (effectively limit to maximum count)
                Tpos = Math.min(inch * robotHW.EXTENSION_COUNTS_PER_INCH, robotHW.EXTENSION_MAXIMUM_COUNT);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.velocityExtensionHandler(Tpos);
                return Math.abs(extender.getCurrentPosition() - Tpos) > 10;
            }

        }

        // usable method/ action class shortcut
        public Action extenderToInch(double inch){
            return new extenderToInches(inch);
        }
    }

    public class Pincher {
        // class vars
        private Servo pincher;

        // class constructor & hardware mapper
        public Pincher(HardwareMap hardwareMap){
            pincher = hardwareMap.get(Servo.class, "claw_pinch");
        }

        // actual action class/ do-er(s) -->
        public class ClosePincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pincher.setPosition(robotHW.CLAW_CLOSE);
                return false;
            }
        }
        public class OpenPincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pincher.setPosition(robotHW.CLAW_OPEN);
                return false;
            }
        }
        // <--

        // usable method/ action class shortcut
        public Action closeClaw() {
            return new ClosePincher();
        }
        public Action openClaw() {
            return new OpenPincher();
        }
    }

    public class Yaw {
        // class vars
        private Servo yaw;

        // class constructor & hardware mapper
        public Yaw(HardwareMap hardwareMap){
            yaw = hardwareMap.get(Servo.class, "claw_yaw");
        }

        // actual action class/ do-er(s) -->
        public class RotateYaw implements Action {
            private double Tpos;
            public RotateYaw(double Tpos){
                this.Tpos = Tpos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                yaw.setPosition(Tpos);
                return false;
            }
        }

        // usable method/ action class shortcut
        public Action rotateClaw(double servoPos) {
            return new RotateYaw(servoPos);
        }
    }

    public class Axial {
        // class vars
        private Servo axial;

        // class constructor & hardware mapper
        public Axial(HardwareMap hardwareMap){
            axial = hardwareMap.get(Servo.class, "claw_axial");
        }

        // actual action class/ do-er(s) -->
        public class RotateAxial implements Action {
            private double Tpos;
            public RotateAxial(double Tpos){
                this.Tpos = Tpos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                axial.setPosition(Tpos);
                return false;
            }
        }

        public class AdaptAxial implements Action {

            public AdaptAxial(){

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.calibrateClaw(robotHW.ELBOW_PERPENDICULAR);
                return false;
            }
        }

        // usable method/ action class shortcut
        public Action rotateAxial(double servoPos) {
            return new RotateAxial(servoPos);
        }
        public Action adaptAxial(){
            return new AdaptAxial();
        }
    }
}
