plugins {
    id("ftc.convention")
}

android {
    namespace = "org.firstinspires.ftc.teamcode.intothedeep"
}

dependencies {
    implementation(project(":FtcRobotController"))
    implementation(libs.androidx.appcompat)
}
