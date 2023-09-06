package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.motorRampProfile;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive2023;
import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.Tweakable;
import org.firstinspires.ftc.teamcode.util.TweakableBoolean;
import org.firstinspires.ftc.teamcode.util.TweakableDouble;
import org.firstinspires.ftc.teamcode.util.TweakableLong;

import java.io.FileReader;
import java.io.FileWriter;
import java.util.Properties;


public class TweakableMovementThread extends RobotThread {

    private Gamepad _gamepad;


    TweakableDouble _zone_lateral = new TweakableDouble("LateralZone",0.02, 0.2);
    TweakableDouble _zone_forward =new TweakableDouble("ForwardZone", 0.02, 0.2 );
    TweakableDouble _zone_rotation =new TweakableDouble("RotateZone", 0.02, 0.1);
    TweakableDouble _ramp_rate =  new TweakableDouble("RampRate", 0.02, 1.5);
    TweakableBoolean _robot_centric = new TweakableBoolean("RobotCentricDrive", true);
    TweakableDouble _fine_control = new TweakableDouble("FineControl", 0.05,0.55);
    TweakableLong _debounce_delay_ms = new TweakableLong("Button Debounce Delay", 20, 150);
    TweakableDouble speed_factor = new TweakableDouble("Speed Factor",0.04,1.4);
    Tweakable _load_save = new Tweakable("<--LOAD / SAVE --") {
            @Override
            public void adjustUp() {
            // load from config
            try {
                Properties props = new Properties();
                props.load(new FileReader(AppUtil.getInstance().getSettingsFile(config)));
                _zone_forward.value = Double.parseDouble(props.getProperty("_zone_forward"));
                _zone_lateral.value =Double.parseDouble(props.getProperty("_zone_lateral"));
                _zone_rotation.value = Double.parseDouble(props.getProperty("_zone_rotation"));
                _ramp_rate.value = Double.parseDouble(props.getProperty("_ramp_rate"));
                _robot_centric.value = Boolean.parseBoolean(props.getProperty("_robot_centric"));
                _fine_control.value  = Double.parseDouble(props.getProperty("_fine_control"));
            } catch (Exception ex) {
                // something wrong here
                _telemetry.log().add("Error loading config!" + config+" "+ex.getMessage());
            }
        }
            @Override
            public void adjustDown() {
            try {
                Properties props = new Properties();
                props.setProperty("_zone_forward", _zone_forward.toString());
                props.setProperty("_zone_lateral", _zone_lateral.toString());
                props.setProperty("_zone_rotation", _zone_rotation.toString());
                props.setProperty("_ramp_rate", _ramp_rate.toString());
                props.setProperty("_robot_centric", _robot_centric.toString());
                props.setProperty("_fine_control ", _fine_control.toString());
                FileWriter writer = new FileWriter(AppUtil.getInstance().getSettingsFile(config));
                props.store(writer,"robot settings");
                writer.close();
            }
            catch (Exception ex) {
                _telemetry.log().add("Error Saving config! "+config);
            }
        }
    };



    private MecanumDrive2023 drive;


    Debounce dpad_up,dpad_down,dpad_left,dpad_right;


    motorRampProfile _Joy1X, _Joy1Y, _Joy2X;
    Telemetry.Item T_TWEAK;//,T_RF,T_RR,T_LF,T_LR;



    private final static String config = "tweakableconfig.props";

    private Tweakable[] tweakables;

    private Telemetry _telemetry;

    private boolean includeTweaks=true;

    public TweakableMovementThread(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry, BNO055IMU imu, long debounceDelayms) {
        this.init(gamepad, motors, telemetry, imu,debounceDelayms);
    }


    public TweakableMovementThread(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry, BNO055IMU imu, long debounceDelayms, boolean includeTweaks) {
        this.includeTweaks= includeTweaks;
        this.init(gamepad, motors, telemetry, imu,debounceDelayms);
    }

    protected void init(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry, BNO055IMU imu, long debounceDelayms) {
        this._gamepad = gamepad;
        this._telemetry = telemetry;

        MecanumDrive2023.Parameters driveParameters = new MecanumDrive2023.Parameters();
        driveParameters.imu = imu;
        driveParameters.telemetry = telemetry;
        driveParameters.motors = motors;
        driveParameters._ENCODER_WHEELS = new int[]{};
        driveParameters._FREE_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters._REVERSED_WHEELS = new int[]{2, 3};
        driveParameters.robotCentric = true;
        driveParameters._SPEED_FACTOR = 1.4;
        driveParameters._ROTATION_RATE = 0.75;
        drive = new MecanumDrive2023(driveParameters);

        _Joy1Y = new motorRampProfile(_ramp_rate.value);
        _Joy1X = new motorRampProfile(_ramp_rate.value);
        _Joy2X = new motorRampProfile(_ramp_rate.value);

        if (includeTweaks) init_tweaks(debounceDelayms);
    }

    protected void init_tweaks(long debounceDelayms) {
            dpad_left = new Debounce(debounceDelayms);
            dpad_right = new Debounce(debounceDelayms);
            dpad_up = new Debounce(debounceDelayms);
            dpad_down = new Debounce(debounceDelayms);


            tweakables = new Tweakable[]{
                    _zone_lateral,
                    _zone_forward,
                    _zone_rotation,
                    _ramp_rate,
                    _robot_centric,
                    _fine_control,
                    _debounce_delay_ms,
                    _load_save
                    };
            T_TWEAK = _telemetry.addData("Tweak:", "%s", tweakables[current_tweakable].name + " - " + tweakables[current_tweakable].toString());
    }

    int current_tweakable = 0;
    private double deadzone(double power, double zone)  {
        return Math.abs(power) > zone ? power : 0.0 ;
    }

    protected void checkTweaks() {
        if (dpad_left.checkPress(_gamepad.dpad_left)) {
            tweakables[current_tweakable].adjustDown();
        }
        if (dpad_right.checkPress(_gamepad.dpad_right)) {
            tweakables[current_tweakable].adjustUp();
        }
        if (dpad_up.checkPress(_gamepad.dpad_up)) {
            // move up the list
            current_tweakable = current_tweakable == 0 ? tweakables.length - 1 : current_tweakable - 1;
        }
        if (dpad_down.checkPress(_gamepad.dpad_down)) {
            //move down the list
            current_tweakable = current_tweakable == tweakables.length - 1 ? 0 : current_tweakable + 1;
        }

        // print out the current value and it's name
        T_TWEAK.setValue("%s", tweakables[current_tweakable].name + " - " + tweakables[current_tweakable].toString());

        drive.setRobotCentric(_robot_centric.value);
        dpad_up.set_debounceDelay(_debounce_delay_ms.value);
        dpad_left.set_debounceDelay(_debounce_delay_ms.value);
        dpad_right.set_debounceDelay(_debounce_delay_ms.value);
        dpad_down.set_debounceDelay(_debounce_delay_ms.value);
        drive.setSpeedFactor(speed_factor.value);
    }

    public void run() {
        while (!isCancelled()) {
            if (includeTweaks) checkTweaks();
            if (_gamepad.right_trigger > 0.2) {
                drive.moveRect(
                        _Joy1Y.ramp(deadzone(_gamepad.left_stick_y*_fine_control.value,_zone_forward.value)),
                        _Joy1X.ramp(deadzone(_gamepad.left_stick_x*_fine_control.value,_zone_lateral.value)),
                        _Joy2X.ramp(deadzone(_gamepad.right_stick_x* _fine_control.value,_zone_rotation.value))
                );
            }
            else {
                drive.moveRect(
                        _Joy1Y.ramp(deadzone(_gamepad.left_stick_y, _zone_forward.value)),
                        _Joy1X.ramp(deadzone(_gamepad.left_stick_x, _zone_lateral.value)),
                        _Joy2X.ramp(deadzone(_gamepad.right_stick_x, _zone_rotation.value))
                );
            }
        }
    }


}
