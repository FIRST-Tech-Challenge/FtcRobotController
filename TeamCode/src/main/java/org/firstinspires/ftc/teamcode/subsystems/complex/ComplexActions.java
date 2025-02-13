package org.firstinspires.ftc.teamcode.subsystems.complex;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawActions;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftActions;

public class ComplexActions {
    private LiftActions liftActions;
    private ClawActions clawActions;

    private Servo claw;
    private Servo rotator;

    public ComplexActions(HardwareMap hardwareMap) {
        liftActions = new LiftActions(hardwareMap);
        clawActions = new ClawActions(hardwareMap);

        claw = hardwareMap.get(Servo.class, "claw");
        rotator = hardwareMap.get(Servo.class, "rotator");
    }

    public class grabCubeFromTray implements Action {
        private double clawPosition = 0;
        private double rotatorPosition = 1;
        boolean liftStatus = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            claw.setPosition(clawPosition);
            if (claw.getPosition() == clawPosition){
                if (liftStatus) {
                    liftStatus = liftActions.liftUp().run(packet);
                }
                else rotator.setPosition(rotatorPosition);
            }
            return rotator.getPosition() == rotatorPosition && claw.getPosition() == clawPosition && !liftStatus;
        }

    }

    public Action grabCubeFromTray() {
        return new grabCubeFromTray();
    }

    public class returnToTray implements Action {
        private double clawPosition = 0;
        private double rotatorPosition = 0;
        boolean clawStatus = true;
        boolean liftStatus = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (clawStatus) {
                clawStatus = clawActions.releaseCube().run(packet);
            }
            if (liftStatus && !clawStatus) {
                liftStatus = liftActions.liftDown().run(packet);
            }
            return rotator.getPosition() == rotatorPosition && claw.getPosition() == clawPosition && !liftStatus;
        }

    }

    public Action returnToTray() {
        return new returnToTray();
    }



}
