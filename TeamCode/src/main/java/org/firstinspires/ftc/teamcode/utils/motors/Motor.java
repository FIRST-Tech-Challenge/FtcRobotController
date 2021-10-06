package org.firstinspires.ftc.teamcode.utils.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Motor {

    private final Telemetry TELEMETRY;
    private final String NAME;
    private final HardwareMap HARDWARE;
    private final DcMotor MOTOR;
    private final int OFFSET;
    private final double COUNTS_PER_REV;
    private final double GEAR_REDUCTION;
    private final double RADIUS;
    private final double COUNTS_PER_INCH;
    private final ElapsedTime TIME = new ElapsedTime();

    public Motor(Telemetry telemetry, HardwareMap hardware, String name, int offset, double countsPerRev, double gearReduction, double radius) {
        TELEMETRY = telemetry;
        NAME = name;
        HARDWARE = hardware;
        MOTOR = HARDWARE.get(DcMotor.class, name);
        OFFSET = offset;
        COUNTS_PER_REV = countsPerRev;
        GEAR_REDUCTION = gearReduction;
        RADIUS = radius;
        COUNTS_PER_INCH = (COUNTS_PER_REV * GEAR_REDUCTION) / (RADIUS * 2 * Math.PI);
        TELEMETRY.addData("Status", "Resetting Encoders");    //
        TELEMETRY.update();
        MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TELEMETRY.addData("Encoder reset and location found",  "Position is ", MOTOR.getCurrentPosition());
        TELEMETRY.update();
    }

    public void move(double distance, double speed, double timeout) {
        MOTOR.setTargetPosition(MOTOR.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH * OFFSET));
        // Turn On RUN_TO_POSITION
        MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        TIME.reset();
        MOTOR.setPower(Math.abs(speed));
        while(TIME.seconds() < timeout && MOTOR.isBusy()) {
            // Display it for the driver.
            TELEMETRY.addData("Target",  "Running to ", MOTOR.getTargetPosition());
            TELEMETRY.addData("Location",  "Running at ", MOTOR.getCurrentPosition());
            TELEMETRY.update();
        }
        MOTOR.setPower(0);
        MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TELEMETRY.addData("Run complete and motor ready for next run",  "Position is ", MOTOR.getCurrentPosition());
        TELEMETRY.update();
    }

    public Telemetry getTelemetry() {
        return TELEMETRY;
    }

    public String getName() {
        return NAME;
    }

    public String toString() {
        return NAME;
    }

    public HardwareMap getHardware() {
        return HARDWARE;
    }

    public DcMotor getMotor() {
        return MOTOR;
    }

    public int getOffset() {
        return OFFSET;
    }

    public double getCountsPerRev() {
        return COUNTS_PER_REV;
    }

    public double getGearReduction() {
        return GEAR_REDUCTION;
    }

    public double getRadius() {
        return RADIUS;
    }

    public double getCountsPerInch() {
        return COUNTS_PER_INCH;
    }

    public ElapsedTime getRuntime() {
        return TIME;
    }

}
