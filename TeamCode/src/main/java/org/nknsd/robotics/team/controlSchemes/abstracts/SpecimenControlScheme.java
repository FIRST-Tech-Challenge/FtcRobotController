package org.nknsd.robotics.team.controlSchemes.abstracts;

import org.nknsd.robotics.framework.NKNControlScheme;

import java.util.concurrent.Callable;

public abstract class SpecimenControlScheme extends NKNControlScheme {
    public EACControlScheme eacControlScheme;

    public abstract Callable<Boolean> specimenGrab();

    public abstract Callable<Boolean> specimenRelease();

    public abstract Callable<Boolean> specimenForward();

    public abstract Callable<Boolean> specimenBackwards();

    public abstract Callable<Boolean> specimenRaise();

    public abstract Callable<Boolean> specimenLower();

    public void linkSchemes(EACControlScheme eacControlScheme) {
        this.eacControlScheme = eacControlScheme;
    }
}

