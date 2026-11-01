/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.config;

@FunctionalInterface
public interface Configuration<T> {
    void configure(T config);
}
