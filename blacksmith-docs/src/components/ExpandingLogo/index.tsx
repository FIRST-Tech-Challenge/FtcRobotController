import LogoBit from "@site/static/img/logo/blacksmith-logo-bit.svg";
import Logo from "@site/static/img/logo/blacksmith-logo-body.svg";
import React from "react";
import clsx, { ClassValue } from "clsx";

import styles from "./styles.module.css";

export interface ExpandingLogoProps {
  className?: ClassValue;
}

export default function ExpandingLogo({ className }: ExpandingLogoProps) {
  return (
    <div className={clsx(styles.logoWrapper, className, styles.logoWrapperAnimation)}>
      <LogoBit className={clsx(styles.logoBit, styles.logoBit1, styles.logoBit1Animation)}/>
      <Logo className={styles.logo}/>
      <LogoBit className={clsx(styles.logoBit, styles.logoBit2, styles.logoBit2Animation)}/>
    </div>
  );
}
