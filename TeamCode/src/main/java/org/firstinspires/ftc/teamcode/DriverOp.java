package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DriverOperationMode")
public class DriverOp extends RobotOpMode {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void robotLoop() {
        //gamePadMoveRobot();

        movement(gamepad1, AnalogInput.LEFT_STICK_Y, AnalogInput.LEFT_STICK_X, AnalogInput.RIGHT_STICK_X);

        /*int direction = 0;
        if(armMotor!= null) {
            if (gamepad1.dpad_up) {
                int position = positionFromAngle(-110, AngleUnit.DEGREES);
                armMotor.setTargetPosition(position);
                armMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                int position = positionFromAngle(0, AngleUnit.DEGREES);
                armMotor.setTargetPosition(position);
                armMotor.setPower(1);
            }
            if (!armMotor.isBusy()) {
                armMotor.setPower(0);
            }
        }*/
        if(gamepad1.dpad_up) {

        } else if(gamepad1.dpad_down) {

        } else {

        }

        //armExtensionMotor.setPower(gamepad1.right_stick_y);
    }

    public void movement(Gamepad gamepad, AnalogInput forwardBackward, AnalogInput strafeLeftRight, AnalogInput rotateLeftRight) {
        if(gamepad1 == null) {
            return;
        }

        double axial   = -forwardBackward.getValue(gamepad);
        double lateral =  strafeLeftRight.getValue(gamepad);
        double yaw     =  rotateLeftRight.getValue(gamepad);
        moveRobot(axial, lateral, yaw, Long.MAX_VALUE);
    }

    public void extension(Gamepad gamepad, DigitalInput forward, DigitalInput backward) {

    }
    public void extension(Gamepad gamepad, AnalogInput forward, AnalogInput backward) {

    }
    public void wrist(Gamepad gamepad, DigitalInput up, DigitalInput down) {

    }
    public void finger(Gamepad gamepad, DigitalInput clamp, DigitalInput release) {

    }
    public void arm(Gamepad gamepad, DigitalInput frontward, DigitalInput backward) {

    }
    public void arm(Gamepad gamepad, AnalogInput frontward, AnalogInput backward) {

    }

    public int positionFromAngle(double angle, AngleUnit angleUnit) {
        double ticksPerRevolution = armMotor.getMotorType().getTicksPerRev();
        double scale = angleUnit.toDegrees(angle)/360;
        return (int) (ticksPerRevolution*scale);
    }

    public enum AnalogInput {
        LEFT_TRIGGER {
            @Override
            public float getValue(Gamepad gamepad) {
                return gamepad.left_trigger;
            }
        },
        RIGHT_TRIGGER {
            @Override
            public float getValue(Gamepad gamepad) {
                return gamepad.right_trigger;
            }
        },
        LEFT_STICK_X {
            @Override
            public float getValue(Gamepad gamepad) {
                return gamepad.left_stick_x;
            }
        },
        LEFT_STICK_Y {
            @Override
            public float getValue(Gamepad gamepad) {
                return gamepad.left_stick_y;
            }
        },
        RIGHT_STICK_X {
            @Override
            public float getValue(Gamepad gamepad) {
                return gamepad.right_stick_x;
            }
        },
        RIGHT_STICK_Y {
            @Override
            public float getValue(Gamepad gamepad) {
                return gamepad.right_stick_y;
            }
        },
        DPAD_X {
            @Override
            public float getValue(Gamepad gamepad) {
                float val = 0;
                val += gamepad.dpad_right ? 1 : 0;
                val -= gamepad.dpad_left ? 1 : 0;
                return 0;
            }
        },
        DPAD_Y {
            @Override
            public float getValue(Gamepad gamepad) {
                float val = 0;
                val += gamepad.dpad_up ? 1 : 0;
                val -= gamepad.dpad_down ? 1 : 0;
                return 0;
            }
        };
        public abstract float getValue(Gamepad gamepad);

    }
    public enum DigitalInput {
        LEFT_BUMPER {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.left_bumper;
            }
        },
        RIGHT_BUMPER {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.right_bumper;
            }
        },
        A {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.a;
            }
        },
        B {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.b;
            }
        },
        X {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.x;
            }
        },
        Y {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.y;
            }
        },
        DPAD_UP {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.dpad_up;
            }
        },
        DPAD_DOWN {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.dpad_down;
            }
        },
        DPAD_LEFT {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.dpad_left;
            }
        },
        DPAD_Right {
            @Override
            public boolean getValue(Gamepad gamepad) {
                return gamepad.dpad_right;
            }
        };

        public abstract boolean getValue(Gamepad gamepad);

    }

    public enum InputType {

    }

}
