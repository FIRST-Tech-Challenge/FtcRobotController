import clsx from "clsx";
import React from "react";

import styles from "./styles.module.css";

type ReturnItem = {
  type: string;
  desc: string;
  typeLink?: string;
};

function ReturnItem({ type, desc, typeLink }: ReturnItem) {
  const typeStyle = clsx(styles.paramType, typeLink && styles.link);

  const handleClick = () => {
    typeLink && (window.location.href = typeLink);
  };

  return (
    <li className={styles.paramList}>
      <text className={typeStyle} onClick={handleClick}>{type}</text>
      <text className={styles.paramDesc}>{' - ' + desc}</text>
    </li>
  );
}

export default function Returns({ returns }: { returns: ReturnItem }): JSX.Element {
  return (
    <div className={'margin-bottom--md'}>
      <div className={clsx('tabs__item', 'tabs__item--active', styles.paramHeader)}>
        Returns
      </div>
      <ReturnItem {...returns}/>
    </div>
  );
}
