import React from 'react';

import BackgroundDesign from '@site/static/img/landing-page-background-pattern.svg';
import styles from './styles.module.css';
import clsx from 'clsx';

export function GradientBackgroundWithLineDesign({ children }) {
  return (<>
    <div className={clsx(styles.gradientBackground)}/>
    <div className={clsx(styles.designWrapper)}>
      <BackgroundDesign className={clsx(styles.backgroundSVG, styles.pattern)}/>
    </div>
    {children}
  </>);
}
