package org.firstinspires.ftc.teamcode.baseClasses.navigation

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.teamcode.baseClasses.MecanumAutoOpMode

enum class AutonomousTarget {
    TARGET_ZONE_A {
        override val bluePositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
        override val redPositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
    },
    TARGET_ZONE_B {
        override val bluePositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
        override val redPositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
    },
    TARGET_ZONE_C {
        override val bluePositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
        override val redPositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
    },
    STARTER_STACK_PICKUP {
        override val bluePositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
        override val redPositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
    },
    OPTIMAL_LAUNCH_POSITION {
        override val bluePositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
        override val redPositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
    },
    LAUNCH_LINE_PARKING_POSITION {
        override val bluePositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
        override val redPositionMatrix: OpenGLMatrix = OpenGLMatrix
                .translation(0f, 0f, 0f);
    };

    abstract val bluePositionMatrix: OpenGLMatrix
    abstract val redPositionMatrix: OpenGLMatrix

    fun positionMatrix(side: MecanumAutoOpMode.AutonomousSide): OpenGLMatrix {
        return when (side) {
            MecanumAutoOpMode.AutonomousSide.RED -> redPositionMatrix
            MecanumAutoOpMode.AutonomousSide.BLUE -> bluePositionMatrix
        }
    }
}