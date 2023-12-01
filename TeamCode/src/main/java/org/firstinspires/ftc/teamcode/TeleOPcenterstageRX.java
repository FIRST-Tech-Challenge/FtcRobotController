/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TeleOPcenterstageRX extends OpMode {
    private double sm = 1;
    private boolean lastx = false, lasty = false;
    private double bval = 0;
    private boolean bl;
    private double y, x, rx, max;
    private double lastTime;
    private double pmotorBL, pmotorBR, pmotorFL, pmotorFR;
    private boolean stop = false, aLansat = false, notEntered, aRetras = false;
    private final ChestiiDeAutonom c = new ChestiiDeAutonom();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        c.init(hardwareMap, telemetry, true);
    }

    public void start() {
        Chassis.start();
        Systems.start();
        AvionSiAGC.start();
    }

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                Log.wtf("Chassis", "check");
                y = gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;

                pmotorFL = -y + x + rx;
                pmotorBL = -y - x + rx;
                pmotorBR = -y + x - rx;
                pmotorFR = -y - x - rx;

                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }

                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                if (gamepad1.x != lastx) {
                    if (gamepad1.x) {
                        if (sm == 1) {
                            sm = 4;
                        }
                        else {
                            sm = 1;
                        }
                    }
                    lastx = gamepad1.x;
                }
                if (gamepad1.y != lasty) {
                    if (gamepad1.y) {
                        if (sm == 1) {
                            sm = 2;
                        }
                        else {
                            sm = 1;
                        }
                    }
                    lasty = gamepad1.y;
                }

                c.POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            }
        }
    });

    private final Thread AvionSiAGC = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                Log.wtf("AvionSiAGC","check");
                if (gamepad2.left_trigger > 0) {
                    c.setPlauncherPosition(0.5);
                    aLansat = true;
                }
                else {
                    c.setPlauncherPosition(0.35);
                }
                Log.wtf("AvionSiAGC","check1");
                if (gamepad2.x) {
                    c.setExtensorPower(-1, 1300);
                }
                else if (!aLansat) {
                    if (!notEntered && c.isRobotFolded()) {
                        c.setExtensorPower(-1, 1300);
                        notEntered = true;
                    }
                    else if (c.isRobotFolded()) {
                        c.setExtensorPower(gamepad2.left_stick_x, 1);
                    }
                    else if (notEntered) {
                        c.setExtensorPower(1, 1300);
                        notEntered = false;
                    }
                }
                else {
                    if (!aRetras) {
                        c.setExtensorPower(-1, 700);
                        aRetras = true;
                    }
                }
                Log.wtf("AvionSiAGC","check2");
            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                Log.wtf("systems","check");
                c.setSliderPower(gamepad2.right_stick_y);
                c.setMelcPower(gamepad2.left_stick_y);
                if (gamepad2.left_bumper) {
                    c.setMacetaPower(1.0);
                }
                if (gamepad2.right_bumper) {
                    c.setMacetaPower(-1.0);
                }

                Log.wtf("systems","check1");
                if (c.isRobotFolded()) {
                    c.inchidere();
                }
                Log.wtf("systems","check2");
                if (gamepad2.b != bl) {
                    if (gamepad2.b) {
                        new Thread(() -> c.letPixelDrop(150)).start();
                    }
                    bl = false;
                }
                else if (gamepad2.a) {
                    c.deschidere();
                }
                else if (gamepad2.y) {
                    c.inchidere();
                }
            }
        }
    });

    public void stop() {
        stop = true;
        c.setIsStopRequested(true);
    }

    @Override
    public void loop() {
        telemetry.addData("motorBL", c.getMotorBLPower());
        telemetry.addData("motorFL", c.getMotorFLPower());
        telemetry.addData("motorBR", c.getMotorBRPower());
        telemetry.addData("motorFR", c.getMotorFRPower());
        telemetry.addData("slider:", c.getSliderPower());
        telemetry.addData("melcjos:", c.getMelcjosPower());
        telemetry.addData("melcsus:", c.getMelcsusPower());
        telemetry.addData("macetaPow:", c.getMacetaPower());
        telemetry.addData("bval:", bval);
        telemetry.addData("bl:", bl);
        telemetry.addData("sm", sm);
        telemetry.addData("gamepad2.b:", gamepad2.b);
        telemetry.addData("potentiometru:", c.getPotentiometruVoltage());
        telemetry.addData("distanceL:", c.getDistanceL(DistanceUnit.CM));
        telemetry.addData("distanceR:", c.getDistanceR(DistanceUnit.CM));
        telemetry.update();
    }
}