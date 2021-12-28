package org.firstinspires.ftc.teamcode.main.utils.tests;

import java.lang.annotation.Documented;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

/**
 * The Skippable annotation tells a Tester that a test shouldn't be ran. This is to make removing Tests easier, as you don't have to delete the Test if you want to keep it.
 */
@Retention(RetentionPolicy.RUNTIME)
public @interface Skippable {}
