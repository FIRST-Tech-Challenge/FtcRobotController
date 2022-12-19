import React from "react";

import BackgroundDesign from "@site/static/img/sentinel-website-background-thingy-v2.svg";
import styles from "./styles.module.css";
import clsx from "clsx";

export function GradientBackground({ className, ...props }: any) {
  return (
    <>
      <div className={clsx(styles.gradientBackground, className)}/>
      {props.children}
    </>
  );
}

export function GradientBackgroundWithLineDesign({ className, designClassName, fixed, ...props }: any) {
  return (
    <GradientBackground>
      <div className={clsx(styles.designWrapper, className)} style={{ position: fixed ? 'fixed' : 'absolute' }}>
        <BackgroundDesign className={clsx(styles.backgroundSVG, designClassName)}/>
      </div>
      {props.children}
    </GradientBackground>
  );
}
