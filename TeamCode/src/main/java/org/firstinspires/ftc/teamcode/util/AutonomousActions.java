package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.EmergencyArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PincherSubsystem;

public class AutonomousActions {
    private static final FTCDashboardPackets dbp = new FTCDashboardPackets("AutoActions");

    @Deprecated
    public static class Pincher {
        private PincherSubsystem pincherSubsystem;

        public Pincher(HardwareMap hardwareMap) {
            try {
                ServoEx pincher1 = RobotHardwareInitializer.ServoComponent.FINGER_1.getEx(hardwareMap, 0, 45);
                ServoEx pincher2 = RobotHardwareInitializer.ServoComponent.FINGER_2.getEx(hardwareMap, 0, 45);
                pincherSubsystem  = new PincherSubsystem(pincher1, pincher2);
            } catch (Exception e) {
                //e.printStackTrace();
                dbp.info("ERROR IN PINCHER (AUTO) SYSTEM");
                dbp.send(true);
                throw new RuntimeException(e);
            }

            pincherSubsystem.closeFinger();
        }

        public class OpenPincher implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    pincherSubsystem.openFinger();
                    initialized = true;
                }
                return !pincherSubsystem.isFingerReady();
            }
        }

        public Action openPincher() {
            return new OpenPincher();
        }

        public class ClosePincher implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    pincherSubsystem.closeFinger();
                    initialized = true;
                }
                return !pincherSubsystem.isFingerReady();
            }
        }

        public Action closePincher() {
            return new ClosePincher();
        }
    }

    public static class EmergencyArm {
        private EmergencyArmSubsystem emergencyArmSubsystem;

        public EmergencyArm(HardwareMap hardwareMap, Telemetry telemetry) {
            try {
                emergencyArmSubsystem = new EmergencyArmSubsystem(hardwareMap, telemetry);
            } catch (Exception e) {
                dbp.info("[EMER_ARM] ERROR IN ARM SYSTEM");
                dbp.error(e);
                dbp.send(true);
                throw new RuntimeException(e);
            }
        }

        public class OpenPincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                emergencyArmSubsystem.openPincher();

                return true;
            }
        }

        public Action openPincher() {
            return new OpenPincher();
        }

        public class ClosePincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                emergencyArmSubsystem.closePincher();

                return true;
            }
        }

        public Action closePincher() {
            return new ClosePincher();
        }
    }
}
