import React from 'react';
import Head from '@docusaurus/Head';
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
      <Head>
        <title>Blacksmith</title>
        <meta content='Embed Title' property='og:title'/>
        <meta content='An intuitive framework for FTC software development' property='og:description'/>
        <meta content='https://blacksmithftc.vercel.app/' property='og:url'/>
        <meta content='/img/logo/blacksmith-logo.png' property='og:image'/>
        <meta content='#c1eab6' data-react-helmet='true' name='theme-color'/>
        <meta name='twitter:card' content='summary_large_image'/>
      </Head>
      <GradientBackgroundWithLineDesign>
        <div className={styles.container}>
          <ExpandingLogo className={styles.logoWrapperOuter}/>
          <button className={styles.button} onClick={handleClick}>GO TO DOCS</button>
        </div>
      </GradientBackgroundWithLineDesign>
    </Layout>
  );
}
