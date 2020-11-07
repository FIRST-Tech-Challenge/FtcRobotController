package org.firstinspires.ftc.teamcode.TeleOps.TeleBase;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FSM.ControllerSM;
import org.firstinspires.ftc.teamcode.FSM.State;
import org.firstinspires.ftc.teamcode.HardwareMap.HMap;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class TeleStates extends ControllerSM {
    HMap robot;
    Gamepad gp1;

    public TeleStates(HMap rb, Gamepad gp1){
        this.robot = rb;
        this.gp1 = gp1;
    }

    public State IDLE = new State() {
        @Override
        public void state_action() {
            robot.TL.setPower(gp1.left_stick_y);
            robot.BL.setPower(gp1.left_stick_y);
            robot.TR.setPower(gp1.right_stick_y);
            robot.BR.setPower(gp1.right_stick_y);
        }
    };

    // Control Based States
    public State MONITOR_LS = new State() {
        @Override
        public void state_action() {
            robot.TL.setPower(gp1.left_stick_y);
            robot.BL.setPower(gp1.left_stick_y);
        }
    };

    public State MONITOR_RS = new State() {
        @Override
        public void state_action() {
            robot.TR.setPower(gp1.right_stick_y);
            robot.BR.setPower(gp1.right_stick_y);
        }
    };

    public State MONITOR_A = new State() {
        @Override
        public void state_action() {

        }
    };

    public State MONITOR_Y = new State() {
        @Override
        public void state_action() {

        }
    };
}
