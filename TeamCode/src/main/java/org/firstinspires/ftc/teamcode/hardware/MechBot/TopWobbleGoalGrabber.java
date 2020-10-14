package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * FoundationHook spec:
 */
public class TopWobbleGoalGrabber extends Logger<TopWobbleGoalGrabber> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo arm;
    private DcMotorEx armMotor;
    private AdjustableServo grabber;


    private final double ARM_UP = 0.9;
    private final double ARM_INIT = ARM_UP;
    private final double ARM_DOWN = 0.25;

    private final double ARM_POWER = 0.5;
    private final double ARM_SPEED = 1000;
    private final int ARM_POS_UP = -350;
    private final int ARM__POS_INIT = 0;
    private final int ARM_POS_DOWN = -936;

    private final double GRABBER_OPEN = 0.8;
    private final double GRABBER_CLOSE = 0.47;
    private final double GRABBER_INIT = GRABBER_CLOSE;

    private boolean armIsDown = false;
    private boolean grabberIsClosed = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "topWobbleGoalGrabber";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public TopWobbleGoalGrabber(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (grabber != null)
            servoInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        grabber = new AdjustableServo(0, 1).configureLogging(
                logTag + ":topWobbleGoalGrabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "t_grabber");
        configuration.register(grabber);

        armMotor = configuration.getHardwareMap().get(DcMotorEx.class, "armMotor");
        if (armMotor != null) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
//        arm = new AdjustableServo(0, 1).configureLogging(
//                logTag + ":topWobbleGoalGrabber", logLevel
//        );
//        arm.configure(configuration.getHardwareMap(), "arm");
//        configuration.register(arm);
        configuration.register(this);
        // servoInit();
    }

    public void servoInit() {
        grabber.setPosition(GRABBER_INIT);
        if (arm!=null)
            arm.setPosition(ARM_INIT);
        armIsDown = false;
        grabberIsClosed = true;
        // configuration.register(this);
    }

    public void armMotorStop() {
        armMotor.setPower(0);
    }

    public Progress armToPosition(int pos) {
        if (armMotor == null) return null;
        armMotor.setPower(0);
        armMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(ARM_SPEED);
        return new Progress() {
            public boolean isDone() {
                if (Math.abs(armMotor.getCurrentPosition() - armMotor.getTargetPositionTolerance()) < 50) {
                    return true;
                }
                return !armMotor.isBusy();
            }
        };
    }

    public void armToPos(int pos) {
        armMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(ARM_SPEED);
    }

    public void armPosUp() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armToPos(ARM_POS_UP);
        armIsDown = false;
    }

    public void armPosDown() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armToPos(ARM_POS_DOWN);
        armIsDown = true;
    }

    public void armPosInit() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armToPos(ARM__POS_INIT);
        armIsDown = false;
    }

    public void armPosAuto() {
        if (armIsDown)
            armPosUp();
        else
            armPosDown();
    }

    public void armMotorUp() {
        armMotor.setVelocity(ARM_SPEED);
        // armMotor.setPower(ARM_POWER);
        // armIsDown = false;
    }

    public void armMotorDown() {
        armMotor.setVelocity(-ARM_SPEED);
        // armMotor.setPower(-ARM_POWER);
        // armIsDown = true;
    }

    public void armUp() {
        if (arm==null) return;
        arm.setPosition(ARM_UP);
        armIsDown = false;
    }

    public void armDown() {
        if (arm==null) return;
        arm.setPosition(ARM_DOWN);
        armIsDown = true;
    }

    public void armAuto() {
        if (armIsDown) {
            armUp();
        } else {
            armDown();
        }
    }

    public void grabberOpen(){
        grabber.setPosition(GRABBER_OPEN);
        grabberIsClosed = false;
    }

    public void grabberClose(){
        grabber.setPosition(GRABBER_CLOSE);
        grabberIsClosed = true;
    }

    public void grabberAuto(){
        if (grabberIsClosed) {
            grabberOpen();
        } else {
            grabberClose();
        }
    }
    public void releaseWobbleGoalCombo() {

        final String taskName = "release Top Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armToPosition(ARM_POS_DOWN);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armToPosition(ARM_POS_UP);
            }
        }, taskName);
    }

    public void grabWobbleGoalCombo() {
        final String taskName = "grab Top Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                armToPosition(ARM_POS_UP);
                return moveGrabber(GRABBER_OPEN);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armToPosition(ARM_POS_DOWN);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_CLOSE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armToPosition(ARM_POS_UP);
            }
        }, taskName);
    }
    private Progress moveGrabber(double position) {
        double adjustment = Math.abs(position - grabber.getPosition());
        grabber.setPosition(position);
        if (position<= GRABBER_CLOSE + 0.1)
            grabberIsClosed=true;
        else {
            grabberIsClosed = false;
        }
        // 600 ms per 180 degree
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 900);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }
    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (grabber != null) {
            line.addData("T-Grabber", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return grabber.getPosition();
                }
            });
        }

        if (arm != null) {
            line.addData("Arm", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return arm.getPosition();
                }
            });
        }

        if (armMotor != null) {
            line.addData("Arm-Motor", "pos/pwd/speed =%s,", new Func<String>() {
                @Override
                public String value() {
                    String s = armMotor.getCurrentPosition() + "/" + armMotor.getPower() + "/" +
                            armMotor.getVelocity();
                    return s;
                }
            });
        }
    }

}



