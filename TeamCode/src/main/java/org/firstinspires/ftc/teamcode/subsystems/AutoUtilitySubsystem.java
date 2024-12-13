package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.DistanceSensorComponent;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoUtilitySubsystem extends SubsystemBase {
    final DistanceSensor LEFT, CENTER, RIGHT;
    final double TARGET = 1;
    final DistanceUnit UNIT = DistanceUnit.CM;


    double error = 0;
    double distance;

    private double DL = 0;
    private double DR = 0;
    private double DC = 0;


    public AutoUtilitySubsystem(HardwareMap hardwareMap) {
        LEFT = DistanceSensorComponent.LEFT_SENSOR.get(hardwareMap);
        CENTER = DistanceSensorComponent.CENTER_SENSOR.get(hardwareMap);
        RIGHT = DistanceSensorComponent.RIGHT_SENSOR.get(hardwareMap);
    }

    @Override
    public void periodic() {
        DR = RIGHT.getDistance(UNIT);
        DL = LEFT.getDistance(UNIT);
        DC = CENTER.getDistance(UNIT);

        if (Math.abs(error) > TARGET) {
            error = DL - DR;

            if (error > 0) {
                // turn right
            }
            else if (error < 0) {
                // turn left
            }
        }
        else if ((((DL + DR) / 2) > (DC + 1)) || ((DL + DR) / 2) < (DC - 1)) {
            if (((DL + DR) / 2) > DC) {
                // drive forward
            }
            else if (((DL + DR) / 2) < DC) {
                // drive backward
            }
        }

    }
}
