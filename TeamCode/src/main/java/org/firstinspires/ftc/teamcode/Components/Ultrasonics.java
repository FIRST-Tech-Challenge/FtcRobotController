package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robot.imu;
import static org.firstinspires.ftc.teamcode.Robot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

public class Ultrasonics {
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    private double ultraRange = 35, lastUltraUpdate = -10, lastSetPos = -10, time = 0.0, robotWidth = 13, robotLength = 17.3;
    private double[] pos = {0, 0, 0};
    private boolean high = false, updated = false;

    public Ultrasonics() {
        ultrasonicFront = op.hardwareMap.get(AnalogInput.class, "ultrasonicFront");
        ultrasonicBack = op.hardwareMap.get(AnalogInput.class, "ultrasonicBack");
        ultrasonicLeft = op.hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicRight = op.hardwareMap.get(AnalogInput.class, "ultrasonicRight");
        ultraFront = op.hardwareMap.get(LED.class, "ultraFront");
        ultraBack = op.hardwareMap.get(LED.class, "ultraBack");
        ultraRight = op.hardwareMap.get(LED.class, "ultraRight");
        ultraLeft = op.hardwareMap.get(LED.class, "ultraLeft");
        ultraFront.enable(true);
        ultraBack.enable(true);
        ultraRight.enable(true);
        ultraLeft.enable(true);
    }

    public boolean updateUltra(double xpos, double ypos, double angle) {
        pos[0] = xpos;
        pos[1] = ypos;
        pos[2] = angle;
        updated = false;
        angle*=180/PI;
        time = op.getRuntime();
        if (time - lastUltraUpdate > 0.06 && !high) {
            ultraBack.enable(true);
            ultraRight.enable(true);
            ultraFront.enable(true);
            ultraLeft.enable(true);
            high = true;
        }
        if (time - lastUltraUpdate > 0.1&high) {
            double distance = getDistance(ultrasonicRight)+robotWidth/2;
            if (distance < 20 && distance > 0) {
                if (abs(angle) < 5) {
                    if(abs(-70.5+distance-pos[1])<1) {
                        pos[1] = -70.5 + distance;
                        updated = true;
                    }
                } else if (abs(180-angle)<5) {
                    if(abs(70.5-distance-pos[1])<1) {
                        pos[1] = 70.5 - distance;
                        updated =true;
                    }
                } else if (abs(-90 - angle) < 5) {
                    if(abs(-70.5+distance-pos[0])<1) {
                        pos[0] = -70.5 + distance;
                        updated =true;
                    }
                }
                if (abs(90 - angle) < 5) {
                    if(abs(70.5-distance-pos[0])<1) {
                        pos[0] = 70.5 - distance;
                        updated =true;
                    }
                }
            }
            distance = getDistance(ultrasonicLeft)-robotWidth/2;
            if (distance < 20 && distance > 0) {
                if (abs(180-angle) < 5) {
                    if(abs(-70.5+distance-pos[1])<1) {
                        pos[1] = -70.5 + distance;
                        updated = true;
                    }
                } else if (abs(angle)<5) {
                    if(abs(70.5-distance-pos[1])<1) {
                        pos[1] = 70.5 - distance;
                        updated =true;
                    }
                } else if (abs(90 - angle) < 5) {
                    if(abs(-70.5+distance-pos[0])<1) {
                        pos[0] = -70.5 + distance;
                        updated =true;
                    }
                }
                else if (abs(-90 - angle) < 5) {
                    if(abs(70.5-distance-pos[0])<1) {
                        pos[0] = 70.5 - distance;
                        updated =true;
                    }
                }
            }
            distance = getDistance(ultrasonicFront)-robotLength/2;
            if (distance < 20 && distance > 0) {
                if (abs(180-angle) < 5) {
                    if(abs(-70.5+distance-pos[1])<1) {
                        pos[0] = -70.5 + distance;
                        updated = true;
                    }
                } else if (abs(angle)<5) {
                    if(abs(70.5-distance-pos[1])<1) {
                        pos[0] = 70.5 - distance;
                        updated =true;
                    }
                } else if (abs(90 - angle) < 5) {
                    if(abs(-70.5+distance-pos[0])<1) {
                        pos[1] = -70.5 + distance;
                        updated =true;
                    }
                }
                else if (abs(-90 - angle) < 5) {
                    if(abs(70.5-distance-pos[0])<1) {
                        pos[1] = 70.5 - distance;
                        updated =true;
                    }
                }
            }
            distance = getDistance(ultrasonicBack)-robotLength/2;
            if (distance < 20 && distance > 0) {
                if (abs(180-angle) < 5) {
                    if(abs(-70.5+distance-pos[1])<1) {
                        pos[1] = -70.5 + distance;
                        updated = true;
                    }
                } else if (abs(angle)<5) {
                    if(abs(70.5-distance-pos[1])<1) {
                        pos[1] = 70.5 - distance;
                        updated =true;
                    }
                } else if (abs(90 - angle) < 5) {
                    if(abs(-70.5+distance-pos[0])<1) {
                        pos[0] = -70.5 + distance;
                        updated =true;
                    }
                }
                else if (abs(-90 - angle) < 5) {
                    if(abs(70.5-distance-pos[0])<2) {
                        pos[0] = 70.5 - distance;
                        updated =true;
                    }
                }
            }
        }
        if(updated&&lastSetPos-op.getRuntime()>1){
            lastSetPos=op.getRuntime();
            return true;
        }
        else {
            return false;
        }
    }

    public double getDistance(AnalogInput sensor) {
        ultraBack.enable(false);
        ultraRight.enable(false);
        ultraFront.enable(false);
        ultraLeft.enable(false);
        lastUltraUpdate=op.getRuntime();
        high = false;
        return 90.48337 * sensor.getVoltage() - 13.12465;
    }

    public Pose2d getPose2d() {
        return new Pose2d(pos[0], pos[1], imu.updateAngle());
    }
}
