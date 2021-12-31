package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;

/**
 * Controller-less intake.
 */
public class AutoIntake {
    private final DcMotor motor;
    private final DistanceSensor distanceSensor;

    public AutoIntake(@NotNull HardwareMap map) {
        this.motor = map.get(DcMotor.class, "intake");
        this.distanceSensor = map.get(DistanceSensor.class, "intakeSensor");
    }

    public void forward() {
        this.motor.setPower(1);
    }

    public void stop() {
        this.motor.setPower(0);
    }

    public void backward() {
        this.motor.setPower(-1);
    }

    public boolean containsObject() {
        return distanceSensor.getDistance(DistanceUnit.MM) <= 210;
    }
}
