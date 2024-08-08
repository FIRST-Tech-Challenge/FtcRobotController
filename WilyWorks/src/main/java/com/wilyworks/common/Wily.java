package com.wilyworks.common;

import java.lang.annotation.*;

/**
 * Mark a class to be used by Wily Works.
 */
@Documented
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface Wily
{
}