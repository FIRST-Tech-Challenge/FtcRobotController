package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.magic.Logger;
import org.firstinspires.ftc.teamcode.magic.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.LinearActuator;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;

import java.util.HashMap;
import java.util.Map;

public class BaseRobot {
    public final Map<String, DcMotor> motors = new HashMap<>();
    public final Map<String, Servo> servos = new HashMap<>();
    public final Map<String, Object> sensors = new HashMap<>();
    public final ElapsedTime runtime = new ElapsedTime();
    public final boolean USE_WEBCAM = true;
    public final DcMotor frontLeftMotor;
    public final DcMotor frontRightMotor;
    public final DcMotor rearLeftMotor;
    public final DcMotor rearRightMotor;
    public final Gamepad primaryGamepad;
    public final Gamepad auxGamepad;
    public final HardwareMap hardwareMap;
    public final LinearActuator linearActuator;
    public final Telemetry telemetry;
    public final Logger logger;
    private final double wristPower = 0.0;
    private final boolean extenderActive = false;
    private final String previousArmMode = "none";
    public DcMotor armMotor;
    public ColorSensor colorSensor;
    public Servo droneServo;
    public Arm arm;
    public Settings settings;
    public Odometry odometry;
    private boolean extenderReleased = true;
    private boolean clawReleasedR = true;
    private boolean actuatorReleased = true;
    private boolean clawReleasedL = true;

    public BaseRobot(HardwareMap hardwareMap, Gamepad primaryGamepad, Gamepad auxGamepad, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.primaryGamepad = primaryGamepad;
        this.auxGamepad = auxGamepad;
        this.telemetry = telemetry;
        this.logger = new Logger(this);
        // Initialize and configure the motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rearRight");

        // IF A WHEEL IS GOING THE WRONG DIRECTION CHECK WIRING red/black
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors.put("frontLeft", frontLeftMotor);
        motors.put("frontRight", frontRightMotor);
        motors.put("rearLeft", rearLeftMotor);
        motors.put("rearRight", rearRightMotor);


        if (Settings.Deploy.ARM) {
            arm = new Arm(this);
        }


        if (Settings.Deploy.DRONE) {
            droneServo = hardwareMap.get(Servo.class, "drone");
            servos.put("drone", droneServo);
            droneServo.scaleRange(0, 1);
        }


        if (Settings.Deploy.ODOMETRY) {
            odometry = new Odometry(this);
        }

        linearActuator = new LinearActuator(this);
    }

    public void shutDown() {
        logger.stop();
    }


    public void driveGamepads() {
        gamepadPrimary();
        gamepadAuxiliary();
    }

    public void mecanumDrive(double drivePower, double strafePower, double rotation) {
        // Adjust the values for strafing and rotation
        strafePower *= Settings.strafe_power_coefficient;

        double frontLeft = drivePower + strafePower + rotation;
        double frontRight = drivePower - strafePower - rotation;
        double rearLeft = drivePower - strafePower + rotation;
        double rearRight = drivePower + strafePower - rotation;


        // Normalize the power values to stay within the range [-1, 1]
        double max = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(rearLeft), Math.abs(rearRight))
        );
        if (max > 1.0) {
            frontLeft /= max;
            frontRight /= max;
            rearLeft /= max;
            rearRight /= max;
        }

        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        rearLeftMotor.setPower(rearLeft);
        rearRightMotor.setPower(rearRight);
    }


    public void gamepadPrimary() {
        /*
            Defaults
         */
        double dpadPower = Settings.dpad_sensitivity;
        double bumperPower = Settings.bumper_sensitivity;
        double rotation = 0.0;

        /*
            Left joystick sets the power of the wheels
         */
        double strafePower = primaryGamepad.left_stick_x;
        double drivePower = -primaryGamepad.left_stick_y;

        /*
            Bumpers rotate the robot
         */
        if (primaryGamepad.right_bumper) {
            rotation += bumperPower;
        }
        if (primaryGamepad.left_bumper) {
            rotation -= bumperPower;
        }
        
       
        /*
            D-pad does fine-tuned movement at a constant rate in a single direction
         */
        if (primaryGamepad.dpad_up) {
            drivePower += dpadPower;
        }
        if (primaryGamepad.dpad_down) {
            drivePower -= dpadPower;
        }
        if (primaryGamepad.dpad_left) {
            strafePower -= dpadPower;
        }
        if (primaryGamepad.dpad_right) {
            strafePower += dpadPower;
        }

        /*
            Drives the motors based on the given power/rotation
         */
        mecanumDrive(drivePower, strafePower, rotation);
    }

    public void gamepadAuxiliary() {
        // BACK: Launch drone
        if (Settings.Deploy.DRONE) {
            if (auxGamepad.back) {
                droneServo.setPosition(droneServo.getPosition() + 0.2);
            } else {
                droneServo.setPosition(droneServo.getPosition() - 0.4);
            }
        }

        // Arm manager, the main auxiliary function
        // Y: Extend arm upwards | X: Retract arm
        // RT: Open right claw | LT: Open left claw
        if (Settings.Deploy.ARM) {
            if (extenderReleased) {
                if (auxGamepad.x) {
                    extenderReleased = false;
                    arm.extender.retract();
                } else if (auxGamepad.y) {
                    extenderReleased = false;
                    arm.extender.extend();
                } else if (auxGamepad.a) {
                    extenderReleased = false;
                    arm.extender.ground();
                }
            } else if (!auxGamepad.a && !auxGamepad.x && !auxGamepad.y) {
                extenderReleased = true;
            }

            if (auxGamepad.right_trigger > 0.1) {
                if (clawReleasedR) {
                    clawReleasedR = false;
                    arm.claw.setRightServo(!arm.claw.openedR);
                }
            } else {
                clawReleasedR = true;
            }
            if (auxGamepad.left_trigger > 0.1) {
                if (clawReleasedL) {
                    clawReleasedL = false;
                    arm.claw.setLeftServo(!arm.claw.openedL);
                }
            } else {
                clawReleasedL = true;
            }

            // UP: Set the wrist to up
            // DOWN: Set the wrist to down
            if (auxGamepad.right_bumper) {
                arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
            } else if (auxGamepad.left_bumper) {
                arm.wrist.setPosition(Wrist.Position.BOARD);
            }
        }

        if (auxGamepad.dpad_up) {
            linearActuator.extend();
        } else if (auxGamepad.dpad_down) {
            linearActuator.retract();
        } else {
            linearActuator.stop();
        }
        if (auxGamepad.dpad_right && actuatorReleased) {
            linearActuator.changePosition();
            actuatorReleased = false;
        } else if (!auxGamepad.dpad_right) {
            actuatorReleased = true;
        }
    }

    public void driveAuto(double strafePower, double drivePower, double rotation, double duration) {
        double start = runtime.seconds();
        while (runtime.seconds() - start < duration) {
            mecanumDrive(drivePower, strafePower, rotation);
        }
    }


    private void setMode(DcMotor.RunMode mode) {
        // Set motor mode for all motors
        for (DcMotor motor : motors.values()) {
            motor.setMode(mode);
        }
    }

    private boolean areMotorsBusy() {
        // Check if any motor is busy
        for (DcMotor motor : motors.values()) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    private void stopMotors() {
        // Stop all motors
        for (DcMotor motor : motors.values()) {
            motor.setPower(0);
        }
    }
}

