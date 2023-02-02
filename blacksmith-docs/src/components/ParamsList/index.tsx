import clsx from "clsx";
import React from "react";

import styles from "./styles.module.css";

type ParamItem = {
  name: string;
  type: string;
  desc: string;
  typeLink?: string;
  defaultsTo?: string;
  isGeneric?: boolean;
};

function Parameter({ name, type, desc, typeLink, defaultsTo, isGeneric }: ParamItem) {
  const typeStyle = clsx(styles.paramType, typeLink && styles.link);

  const handleClick = () => {
    typeLink && (window.location.href = typeLink);
  };

  const DefaultsToComponent = (
    <div>
      <text className={styles.paramDefaultsToHeader}>- Defaults to: </text>
      <code className={styles.paramDefaults}>{defaultsTo}</code>
    </div>
  );

  const formattedType = (isGeneric) ? `<${type}>` : type

  return (
    <li key={name} className={styles.paramList}>
      {!isGeneric && <text className={styles.paramName}>{name + ': '}</text>}
      <text className={typeStyle} onClick={handleClick}>{formattedType}</text>
      <text className={styles.paramDesc}>{' - ' + desc}</text>
      {defaultsTo && DefaultsToComponent}
    </li>
  );
}

export default function ParamsList({ params }: { params: ParamItem[] }): JSX.Element {
  return (
    <div className={'margin-bottom--md'}>
      <div className={clsx('tabs__item', 'tabs__item--active', styles.paramHeader)}>
        Params
      </div>
      {params.map((param) => (
        <Parameter {...param}/>
      ))}
    </div>
  );
}
