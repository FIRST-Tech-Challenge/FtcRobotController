package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.MovePincherCommand;
import org.firstinspires.ftc.teamcode.commands.SetUppiesCommand;
import org.firstinspires.ftc.teamcode.subsystems.PincherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UppiesSubsystem;

public class AutonomousActions {
    private static final FTCDashboardPackets dbp = new FTCDashboardPackets("AutoActions");

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
    }

    public static class Uppies {
        private UppiesSubsystem uppiesSubsystem;

        public Uppies(HardwareMap hardwareMap) {
            try {
                DcMotorEx uppiesMotor = RobotHardwareInitializer.MotorComponent.UPPIES.getEx(hardwareMap);
                uppiesSubsystem = new UppiesSubsystem(uppiesMotor);
            } catch (Exception e) {
                //e.printStackTrace();
                throw new RuntimeException(e);
            }
        }
    }
}
