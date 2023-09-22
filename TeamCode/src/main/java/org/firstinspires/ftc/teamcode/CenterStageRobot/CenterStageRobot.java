package org.firstinspires.ftc.teamcode.CenterStageRobot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.DriveConstants;
import org.inventors.ftc.robotbase.GamepadExEx;
import org.inventors.ftc.robotbase.RobotEx;

public class CenterStageRobot extends RobotEx {
    //----------------------------------- Initialize Subsystems -----------------------------------//

    //----------------------------------- Initialize Commands ------------------------------------//

    public CenterStageRobot(HardwareMap hm, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                            GamepadExEx toolOp) {
        super(hm, RobotConstants, telemetry, driverOp, toolOp, OpModeType.TELEOP, false,
                false);
    }

    public CenterStageRobot(HardwareMap hm, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                            GamepadExEx toolOp, OpModeType opModeType, boolean camera,
                            boolean cameraFollower) {
        super(hm, RobotConstants, telemetry, driverOp, toolOp, opModeType, camera,
                cameraFollower);
    }

    @Override
    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        super.initMechanismsAutonomous(hardwareMap);
    }

    @Override
    public void initMechanismsTeleOp(HardwareMap hardwareMap) {
        super.initMechanismsTeleOp(hardwareMap);
    }
}