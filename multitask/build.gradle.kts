import org.jetbrains.kotlin.gradle.dsl.JvmTarget as DSLJvmTarget
import org.jetbrains.kotlin.gradle.tasks.KotlinJvmCompile

plugins {
    id("java-library")
    id("org.jetbrains.kotlin.jvm")
}

java {
    sourceCompatibility = JavaVersion.VERSION_1_8
    targetCompatibility = JavaVersion.VERSION_1_8
}

kotlin {
    compilerOptions {
        jvmTarget.set(DSLJvmTarget.JVM_1_8)
        target {
            version
        }
    }
}
tasks.test {
    useJUnitPlatform()
}

tasks.withType<Jar>().configureEach {
    duplicatesStrategy = DuplicatesStrategy.EXCLUDE
}

dependencies {
    testImplementation(kotlin("test"))
}
