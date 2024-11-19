package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

public class HangSubsystem extends SubsystemBase {

    final DcMotor extensionMotor;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("Hang Subsystem");

    public HangSubsystem(DcMotor extensionMotor) {
        this.extensionMotor = extensionMotor;
        if (extensionMotor == null) {
            dbp.info("WARNING: Hang motor is not set up in the config!", true);
        }
    }

    public void setHangDirection(HangDirection direction) {
        if (extensionMotor == null) {
            return;
        }

        extensionMotor.setPower(direction == HangDirection.IDLE ? 0 : 1);
        // TODO: Verify if the Direction.FORWARD and REVERSE is correct (may need to reverse it)
        DcMotorSimple.Direction motorDirection = direction == HangDirection.DOWN ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        extensionMotor.setDirection(motorDirection);

        dbp.createNewTelePacket();
        dbp.info(String.format("Hang Direction: %s", direction.toString()));
        dbp.send(false);
    }

    // Used to determine whether the motor will go up, down, or remain idle
    public enum HangDirection {
        UP,
        DOWN,
        IDLE;
    }

}
