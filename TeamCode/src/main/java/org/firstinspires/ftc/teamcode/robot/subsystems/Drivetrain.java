package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.broombots.sweep.Classes.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drivetrain subsystem for a mecanum drive robot.
 * This class handles the control of the robot's drivetrain, including field-centric driving and telemetry.
 */
public class Drivetrain implements Subsystem {
    // Motor declarations for the four mecanum wheels
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    // Array to hold the motors for easier iteration
    private final DcMotor[] motors;
    // Flag to enable or disable field-centric driving
    private boolean fieldCentricDrivingEnabled = false;
    // Variables to hold the desired movement vector and rotation, allows previous values to be stored and used in the update loop
    private double X = 0,Y = 0,Yaw = 0;
    // Variable to hold the robot's current yaw angle, used for field-centric calculations
    private double robotYaw = 0; // From localization values, used to transform values to field centric
    // Telemetry object for sending data to the driver station
    private Telemetry telemetry;
    // Flag to enable or disable telemetry output
    private boolean telemetryEnabled = false;
    // Flag to indicate if the subsystem is disabled
    private boolean disabled = false;

    /**
     * Constructor for the Drivetrain subsystem.
     * @param hardwareMap The hardware map to initialize motors
     * @param telemetry The telemetry object for sending data to the driver station
     * @param telemetryEnabled Flag to enable or disable telemetry output
     */
    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, boolean telemetryEnabled){
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        motors = new DcMotor[]{FLMotor,FRMotor,BLMotor,BRMotor};
        for (DcMotor motor: motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        this.telemetry = telemetry;
        this.telemetryEnabled = telemetryEnabled;
    }

    /**
     * Update method for the Drivetrain subsystem.
     * This method is called every iteration of the main loop and updates the drivetrain's power based on the current movement vector and rotation.
     * If field-centric driving is enabled, it transforms the movement vector based on the robot's current yaw angle.
     * It also posts telemetry data if telemetry is enabled.
     */
    @Override
    public void update() {
        // To the drive power every iteration
        if (fieldCentricDrivingEnabled){
            double[] fieldVector = calculateFieldCentric(robotYaw);
            X = fieldVector[0];
            Y = fieldVector[1];
        }
        setDrivePower(X,Y,Yaw);
        if (telemetryEnabled){
            postDrivetrainTelemetry();
        }
    }
    /**
     * Update the movement vector for the drivetrain.
     * @param X The desired X movement
     * @param Y The desired Y movement
     * @param Yaw The desired rotation
     */
    public void updateMovementVector(double X, double Y, double Yaw){
        this.X = X;
        this.Y = Y;
        this.Yaw = Yaw;
    }
    /**
     * Update the movement vector and robot yaw for the drivetrain.
     * @param X The desired X movement
     * @param Y The desired Y movement
     * @param Yaw The desired rotation
     * @param robotYaw The current robot yaw
     */
    public void updateMovementVector(double X, double Y, double Yaw, double robotYaw){
        updateMovementVector(X,Y,Yaw);
        this.robotYaw = robotYaw;
    }
    /**
     * Enable or disable field-centric driving.
     * @param enabled True to enable field-centric driving, false to disable
     */
    public void enableFieldCentricDriving(boolean enabled){
        this.fieldCentricDrivingEnabled = enabled;
    }

    /**
     * Set the disabled state of the drivetrain subsystem. Required function for all subsystems.
     * @param disable True to disable the subsystem, false to enable
     */
    @Override
    public void setDisabled(boolean disable){
        if (disable){
            for (DcMotor motor: motors){
                motor.setPower(0);
            }
        }
        this.disabled = disable;
    }
    /**
     * Calculate the field-centric movement vector based on the robot's current yaw angle.
     * @param angle The current robot yaw angle in degrees
     * @return An array containing the transformed X and Y movement values
     */
    private double[] calculateFieldCentric(double angle){
        angle = AngleUnit.DEGREES.toRadians(angle); // convert to radians
        double fieldY = Y * Math.cos(angle) + X * Math.sin(angle);
        double fieldX = -Y * Math.sin(angle) + X * Math.cos(angle);
        return new double[]{fieldX,fieldY};
    }
    /**
     * Set the drive power for the drivetrain.
     * @param forward The forward/backward movement power
     * @param strafe The left/right movement power
     * @param rotate The rotational movement power
     */
    private void setDrivePower(double forward, double strafe, double rotate) {
        if (disabled) return;
        // Mecanum drive: distribute robot forces to individual motors
        double[] powers = {
                forward + strafe + rotate, //FL
                forward - strafe - rotate, //FR
                forward - strafe + rotate, //BL
                forward + strafe - rotate  //BR
        };
        // Calculate the max power value out of all possible solutions
        double maxPower = 0;
        for (double power : powers){
            maxPower = Math.max(maxPower, power);
        }
        // Apply motor power normalized to the greatest possible value
        for (int i = 0; i < 4; i++){
            motors[i].setPower(powers[i]/maxPower);
        }
    }
    /**
     * Post telemetry data for the drivetrain subsystem.
     */
    private void postDrivetrainTelemetry(){
        telemetry.addLine("---Drivetrain---");
        telemetry.addData("X Power", X);
        telemetry.addData("Y Power", Y);
        telemetry.addData("Yaw Power", Yaw);
        telemetry.addData("FCD Enabled", fieldCentricDrivingEnabled);
        telemetry.addData("FCD Angle", robotYaw);
    }
}
