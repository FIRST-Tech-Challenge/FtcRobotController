package org.firstinspires.ftc.teamcode.main.utils.resources;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.main.utils.autonomous.location.pipeline.PositionSystem;
import org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.NavigationSensorCollection;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardDistanceSensor;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardIMU;

public class Resources {
    public static final class Navigation {
        public static final class Sensors {

            public static final class Distance {
                public static final String North = "distN";
                public static final String East = "distE";
                public static final String West = "distW";
            }
            public static NavigationSensorCollection getSensorCollection(HardwareMap hardwareMap) {
                return new NavigationSensorCollection(
                        new StandardDistanceSensor(hardwareMap, Distance.North),
                        new StandardDistanceSensor(hardwareMap, Distance.East),
                        new StandardDistanceSensor(hardwareMap, Distance.West),
                        new StandardIMU(hardwareMap),
                        90);
            }

            public static PositionSystem getPositionSystem(HardwareMap hardwareMap) {
                return new PositionSystem(getSensorCollection(hardwareMap));
            }
        }
    }

    public static final class Drivetrain {
        public static final class Motors {
            public static final class Driving {
                public static final String LeftTop = "ltM";
                public static final String LeftBottom = "lbM";
                public static final String RightTop = "rtM";
                public static final String RightBottom = "rbM";
            }
        }
    }

    public static final class Intake {
        public static final class Motors {
            public static final String Spinner = "iM";
        }
        public static final class Servos {
            public static final String Lifter = "iuS";
        }
        public static final class Sensors {
            public static final String LiftLimitSwitch = "iT";
            public static final String LiftDistanceSensor = "iS";
        }
    }

    public static final class Elevator {
        public static final class Motors {
            public static final String RightLift = "l1M";
            public static final String LeftLift = "l2M";
        }
        public static final class Sensors {
            public static final String BottomLimitSwitch = "lT";
        }
    }

    public static final class Hand {
        public static final class Servos {
            public static final String Spinning = "hfS";
        }
        public static final class Sensors {
            public static final String HandDistance = "diD";
        }
    }

    public static final class DuckSpinner {
        public static final class Motors {
            public static final String Spinner = "duM";
        }
    }

    public static final class Misc {
        public static final String VuforiaKey = "AcQbfNb/////AAABmUoZxvy9bUCeksf5rYATLidV6rQS+xwgakOfD4C+LPj4FmsvqtRDFihtnTBZUUxxFbyM7CJMfiYTUEwcDMJERl938oY8iVD43E/SxeO64bOSBfLC0prrE1H4E5SS/IzsVcQCa9GsNaWrTEushMhdoXA3VSaW6R9KrrwvKYdNN/SbaN4TPslQkTqSUr63K60pkE5GqpeadAQuIm8V6LK63JD1TlF665EgpfsDZeVUBeAiJE86iGlT1/vNJ9kisAqKpBHsRyokaVClRnjlp28lmodjVRqeSk8cjCuYryn74tClfxfHQpkDDIsJO+7IYwJQCZQZZ+U9KJaMUeben4HOj0JTnQaEE6MZLaLQzY+C/6MS";
        public static final String Webcam = "webcam";
    }
}
