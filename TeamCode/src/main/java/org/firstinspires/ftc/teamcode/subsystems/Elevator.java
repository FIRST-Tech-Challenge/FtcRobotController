package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator extends SubsystemBase {
    private static double TICKS_PER_MM = 0.335;
    private Motor elevatorLeft;
    private Motor elevatorRight;
    private Telemetry telemetry;

    public Elevator(HardwareMap hMap, Telemetry telemetry){
        this.elevatorLeft = new Motor(hMap, "ElevatorLeft");
        this.elevatorRight = new Motor(hMap, "ElevatorRight");

        this.telemetry = telemetry;
        this.elevatorLeft.setInverted(true);
    }

    @Override
    public void periodic() {
        telemetry.addData("Pos", this.elevatorRight.getCurrentPosition());
    }

    public void goToPos() {
        int currentPos = this.elevatorRight.getCurrentPosition();
        double currentPosMM = currentPos * TICKS_PER_MM;
    }

    public void manualControl(double pow) {
        elevatorRight.set(pow);
        elevatorLeft.set(pow);
    }

}
