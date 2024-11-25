package org.nknsd.robotics.team.controlSchemes.abstracts;

import org.nknsd.robotics.framework.NKNControlScheme;
import org.nknsd.robotics.team.components.RotationHandler;

import java.util.concurrent.Callable;
public abstract class SpecimenControlScheme extends NKNControlScheme {
    public EACControlScheme eacControlScheme;
    public abstract Callable specimenGrab();
    public abstract Callable specimenRelease();
    public abstract Callable specimenForward();
    public abstract Callable specimenBackwards();
    public abstract Callable specimenRaise();
    public abstract Callable specimenLower();

    public void link(EACControlScheme eacControlScheme) {this.eacControlScheme = eacControlScheme ;}
}

