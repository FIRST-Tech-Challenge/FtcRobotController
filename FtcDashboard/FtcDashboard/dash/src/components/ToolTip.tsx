import { useRef, RefObject, forwardRef, PropsWithChildren } from 'react';
import { createPortal } from 'react-dom';
import clsx from 'clsx';

export const ToolTipEl = forwardRef<
  HTMLSpanElement,
  PropsWithChildren<{
    isShowing: boolean;
    top: string;
    left: string;
  }>
>((props, ref) => (
  <span
    ref={ref}
    className={clsx(
      'pointer-events-none fixed w-max transform rounded-md px-3 py-1 transition',
      'bg-gray-800 bg-opacity-80 text-sm text-white',
      props.isShowing
        ? '-translate-y-11 opacity-100'
        : '-translate-y-9 opacity-0',
    )}
    style={{
      top: props.top,
      left: props.left,
    }}
  >
    {props.children}
  </span>
));
ToolTipEl.displayName = 'ToolTipEl';

type ToolTipProps = PropsWithChildren<{
  hoverRef: RefObject<HTMLElement | null>;
  isShowing: boolean;
}>;

const ToolTip = ({ children, hoverRef, isShowing }: ToolTipProps) => {
  const tooltipRef = useRef<HTMLSpanElement | null>(null);

  return createPortal(
    <ToolTipEl
      ref={tooltipRef}
      isShowing={isShowing}
      top={`${hoverRef.current?.getBoundingClientRect().top}px`}
      left={`${
        (hoverRef.current?.getBoundingClientRect().left ?? 0) +
        (hoverRef.current?.clientWidth ?? 0) / 2 -
        (tooltipRef.current?.clientWidth ?? 0) / 2
      }px`}
    >
      {children}
    </ToolTipEl>,
    document.body,
  );
};

export default ToolTip;
