import { ReactElement, cloneElement, Children, PropsWithChildren } from 'react';

type RadialFabProps = PropsWithChildren<{
  isOpen: boolean;
  isShowing: boolean;

  icon: string;

  className: string;

  width?: string;
  height?: string;

  bottom?: string;
  right?: string;

  onClick: () => void;
}>;

const FixedContainer = ({
  bottom,
  height,
  isShowing,
  right,
  children,
}: RadialFabProps) => (
  <div
    style={{
      position: 'fixed',
      bottom: isShowing ? bottom : `calc(${bottom} - (${height} * 2))`,
      right,
      transition: 'bottom 300ms ease',
    }}
  >
    {children}
  </div>
);

const FloatingButton = ({
  children,
  width,
  height,
  className,
  ...props
}: Pick<RadialFabProps, 'children' | 'width' | 'height'> &
  JSX.IntrinsicElements['button']) => (
  <button
    className={`flex-center rounded-full border-none p-0 !outline-none transition duration-300 focus:outline-none focus:ring-2 focus:ring-opacity-50 ${className}`}
    style={{ width, height }}
    {...props}
  >
    {children}
  </button>
);

const RadialFab = (props: RadialFabProps) => {
  return (
    <FixedContainer {...props}>
      <FloatingButton
        width={props.width}
        height={props.height}
        onClick={props.onClick}
        className={props.className}
      >
        <img src={props.icon} className="w-[1.95em] text-white" />
      </FloatingButton>
      {Children.map(props.children, (e) =>
        cloneElement(e as ReactElement, { isOpen: props.isOpen }),
      )}
    </FixedContainer>
  );
};

RadialFab.defaultProps = {
  width: '4em',
  height: '4em',
  bottom: '2em',
  right: '2em',
};

export default RadialFab;
