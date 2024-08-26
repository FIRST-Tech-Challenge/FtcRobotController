import clsx from 'clsx';
import { forwardRef, PropsWithChildren } from 'react';

type BaseViewProps = PropsWithChildren<{
  id?: string;
  isUnlocked?: boolean;
}>;

const BaseView = forwardRef<
  HTMLDivElement,
  BaseViewProps & JSX.IntrinsicElements['div']
>(({ className, isUnlocked, children, ...props }, ref) => (
  <div
    ref={ref}
    className={clsx(
      'flex h-full flex-col overflow-hidden bg-white bg-opacity-75 transition-shadow',
      isUnlocked
        ? 'select-none rounded-md bg-opacity-75 shadow-md dark:bg-slate-800'
        : 'dark:bg-slate-900',
      className,
    )}
    {...props}
  >
    {children}
  </div>
));
BaseView.displayName = 'BaseView';

type BaseViewHeadingProps = {
  isDraggable?: boolean;
};

const BaseViewHeading = ({
  className,
  children,
  isDraggable,
  ...props
}: BaseViewHeadingProps & JSX.IntrinsicElements['h2']) => (
  <h2
    className={clsx(
      'w-full px-4 py-2 text-xl font-medium',
      isDraggable && 'grab-handle',
      className,
    )}
    {...props}
  >
    {children}
  </h2>
);

const BaseViewBody = ({
  children,
  className,
  ...props
}: JSX.IntrinsicElements['div']) => (
  <div className={`flex-1 overflow-auto px-4 ${className}`} {...props}>
    {children}
  </div>
);

const BaseViewIcons = ({
  className,
  children,
  ...props
}: JSX.IntrinsicElements['div']) => (
  <div className={`mr-3 flex items-center space-x-1 ${className}`} {...props}>
    {children}
  </div>
);

const BaseViewIcon = ({
  className,
  children,
  ...props
}: JSX.IntrinsicElements['div']) => (
  <div className={`flex-center h-8 w-8 ${className}`} {...props}>
    {children}
  </div>
);

const BaseViewIconButton = ({
  className,
  children,
  ...props
}: JSX.IntrinsicElements['button']) => (
  <button className={`icon-btn h-8 w-8 ${className}`} {...props}>
    {children}
  </button>
);

export {
  BaseView as default,
  BaseViewHeading,
  BaseViewBody,
  BaseViewIcons,
  BaseViewIcon,
  BaseViewIconButton,
};
export type { BaseViewProps, BaseViewHeadingProps };
