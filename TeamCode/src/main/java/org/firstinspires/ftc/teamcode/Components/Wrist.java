package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class Wrist extends RFServo {
    double target = 0, lastTime = 0.0, FLIP_TIME = 0.2;

    public Wrist() {
        super("wristServo", 1.0);
        target = WristStates.FLAT.position;
        super.setPosition(WristStates.FLAT.position);
        LOGGER.log("initializing hardware");
    }

    public enum WristStates {
        FLAT(1.0, true),
        HOLD(0.7, false),
        FLIP(0.2, false),

        DROP(0.0, false);
        double position;
        boolean state;

        WristStates(double p_position, boolean p_state) {
            position = p_position;
            state = p_state;
        }

        public void setStateTrue() {
            if (!this.state) {
                for (int i = 0; i < WristStates.values().length; i++) {
                    if (WristStates.values()[i].state) {
                        LOGGER.log(WristStates.values()[i].name() + " position set to: false");
                    }
                    WristStates.values()[i].state = false;
                }
                this.state = true;
                LOGGER.log(this.name() + " position set to : true");
            }
        }
    }

    public enum WristTargetStates {
        FLAT(1.0, true),
        FLIP(0.2, false),

        HOLD(0.7, false),
        DROP(0.0, false);
        double position;
        boolean state;

        WristTargetStates(double p_position, boolean p_state) {
            position = p_position;
            state = p_state;
        }

        public void setStateTrue() {
            if (!this.state) {
                for (int i = 0; i < WristTargetStates.values().length; i++) {
                    if (WristTargetStates.values()[i].state) {
                        LOGGER.log(WristTargetStates.values()[i].name() + " target set to: false");
                    }
                    WristTargetStates.values()[i].state = false;
                }
                this.state = true;
                LOGGER.log(this.name() + " target set to : true");
            }
        }
    }

    public void flipTo(WristTargetStates p_state) {
        if (target != p_state.position) {
            if (p_state == WristTargetStates.FLAT) {
                if (Arm.ArmStates.UNFLIPPED.state && Arm.ArmTargetStates.UNFLIPPED.getState() && Lift.LiftPositionStates.AT_ZERO.state) {
                    LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
                    super.setPosition(p_state.position);
                    target = super.getTarget();
                } else {
                    LOGGER.log("slides not in right position, can't flip to flat");
                }
            } else if (p_state == WristTargetStates.HOLD) {
                if (Arm.ArmStates.UNFLIPPED.state) {
                    LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
                    super.setPosition(p_state.position);
                    target = super.getTarget();
                } else
                    LOGGER.log("arm not in right position, can't flip to hold");
            } else if (p_state == WristTargetStates.DROP) {
                if (!Lift.LiftPositionStates.AT_ZERO.state) {
                    LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
                    super.setPosition(p_state.position);
                target = super.getTarget();
                } else
                    LOGGER.log("lift not in right position, can't flip to drop");
            }
            else if (p_state == WristTargetStates.FLIP) {
                if (!Lift.LiftPositionStates.AT_ZERO.state) {
                    LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
                    super.setPosition(p_state.position);
                    target = super.getTarget();
                } else
                    LOGGER.log("lift not in right position, can't flip to flip");
            }
            p_state.setStateTrue();
        }
    }

    public void update() {
        for (var i : WristStates.values()) {
            if (i.position == target && time - lastTime > FLIP_TIME) {
                LOGGER.log("goon");
                i.setStateTrue();
            }
        }
        for (var i : WristTargetStates.values()) {
            if (i.state && target != i.position) {
                flipTo(i);
            }
        }
    }
}
