package org.firstinspires.ftc.teamcode.subsystems.claw;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.lift.LiftActions;

public class ClawActions {
    private Servo claw;
    private Servo rotator;
//    private HardwareMap hardwareMap;
    private LiftActions lift;


    public ClawActions(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "clawServo");
        rotator = hardwareMap.get(Servo.class, "rotatorServo");
        lift = new LiftActions(hardwareMap);
    }

    public class GrabCube implements Action {
        private double clawPosition = 0.2;
        private double rotatorPosition = 0;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(clawPosition);
            rotator.setPosition(rotatorPosition);
            return rotator.getPosition() == rotatorPosition && claw.getPosition() == clawPosition;
        }
    }

    public Action grabCube() {
        return new GrabCube();
    }

    public class ReleaseCube implements Action {
        private double clawPosition = 1;
        private double rotatorPosition = 1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rotator.setPosition(rotatorPosition);
            if (rotator.getPosition() == rotatorPosition) {
                claw.setPosition(clawPosition);
            }
            return rotator.getPosition() == rotatorPosition && claw.getPosition() == clawPosition;
        }

    }

    public Action releaseCube() {
        return new ReleaseCube();
    }

    public class ReturnToStart implements Action {
      private double clawPosition = 0;
      private double rotatorPosition = 0;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rotator.setPosition(rotatorPosition);
            if (rotator.getPosition() == rotatorPosition) {
                claw.setPosition(clawPosition);
            }
            return rotator.getPosition() == rotatorPosition && claw.getPosition() == clawPosition;
        }
        public void setParams(double clawPosition, double rotatorPosition) {
            this.clawPosition = clawPosition;
            this.rotatorPosition = rotatorPosition;
        }
    }

    public Action returnToStart() {
        return new ReleaseCube();
    }


}
