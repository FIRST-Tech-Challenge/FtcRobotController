import java.net.URI

plugins {
    id("java-library")
    id("org.jetbrains.kotlin.jvm")
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

repositories {
    maven { url = URI.create("https://jitpack.io") }
    maven { url = URI.create("https://maven.brott.dev/") }
}

dependencies {
    implementation("com.github.rh-robotics:MeepMeep:v1.0.0")
}