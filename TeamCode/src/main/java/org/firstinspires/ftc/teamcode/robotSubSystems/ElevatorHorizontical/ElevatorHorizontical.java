package org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.PID;

public class ElevatorHorizontical {
    private static DcMotor elevatorMotor;
    private static PID pid = new PID(ElevatorHorizonticalConstants.Kp,ElevatorHorizonticalConstants.Ki,ElevatorHorizonticalConstants.Kd,ElevatorHorizonticalConstants.Kf,ElevatorHorizonticalConstants.iZone,ElevatorHorizonticalConstants.maxSpeed,ElevatorHorizonticalConstants.minSpeed);

    private static float wantedPos;
    private static int resetVal = 0;

    public static void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class,"horElevator");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void opreate(ElevatorHorizonticalState elevatorHorizonticalState, float rightJoyStick, Telemetry telemetry){
                switch(elevatorHorizonticalState){
                    case OPEN:
                        wantedPos = ElevatorHorizonticalConstants.openPos;
                        break;
                    case CLOSE:
                        wantedPos = ElevatorHorizonticalConstants.closedPos;
                        break;
                    case ALMOST:
                        wantedPos = ElevatorHorizonticalConstants.almostPos;
                        break;
                    case HALF:
                        wantedPos = ElevatorHorizonticalConstants.halfpos;
                        break;
                    case OVERRIDE:
                        wantedPos += ElevatorHorizonticalConstants.overrideFactor * -rightJoyStick;
                        break;
                }
                pid.setWanted(wantedPos);
                elevatorMotor.setPower(pid.update(getPos()));
    }
    public static double getPos() {return elevatorMotor.getCurrentPosition() - resetVal;}
    public static void resetEncoder() {resetVal = elevatorMotor.getCurrentPosition();}
}
