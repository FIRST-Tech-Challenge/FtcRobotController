import React from 'react';
import { GradientBackgroundWithLineDesign } from "@site/src/components/CoolLineBackground";

import styles from "./styles.module.css";
import ExpandingLogo from "@site/src/components/ExpandingLogo";
import Layout from '@theme/Layout';

export default function Home(): JSX.Element {
  const handleClick = () => {
    window.location.href = '/overview';
  }

  return (
    <Layout noFooter={true}>
      <GradientBackgroundWithLineDesign>
        <div className={styles.container}>
          <ExpandingLogo className={styles.logoWrapperOuter}/>
          <button className={styles.button} onClick={handleClick}>GO TO DOCS</button>
        </div>
      </GradientBackgroundWithLineDesign>
    </Layout>
  );
}
