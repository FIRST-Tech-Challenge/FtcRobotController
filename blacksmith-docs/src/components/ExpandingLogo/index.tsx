import LogoBit from "@site/static/img/logo/partial-sentinel-temp-logo-thingy-bit.svg";
import Logo from "@site/static/img/logo/partial-sentinel-temp-logo-thingy.svg";
import React from "react";
import clsx from "clsx";

import styles from "./styles.module.css";

export default function ExpandingLogo({ className }) {
  return (
    <div className={clsx(styles.logoWrapper, className, styles.logoWrapperAnimation)}>
      <LogoBit className={clsx(styles.logoBit, styles.logoBit1, styles.logoBit1Animation)}/>
      <Logo className={styles.logo}/>
      <LogoBit className={clsx(styles.logoBit, styles.logoBit2, styles.logoBit2Animation)}/>
    </div>
  );
}
