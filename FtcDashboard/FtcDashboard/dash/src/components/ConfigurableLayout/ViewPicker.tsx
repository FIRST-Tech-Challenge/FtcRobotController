import { cloneElement, PropsWithChildren } from 'react';
import clsx from 'clsx';

import { ConfigurableView } from '@/enums/ConfigurableView';

import { ReactComponent as CameraIcon } from '@/assets/icons/camera.svg';
import { ReactComponent as SettingsIcon } from '@/assets/icons/settings.svg';
import { ReactComponent as ChartIcon } from '@/assets/icons/chart.svg';
import { ReactComponent as ApiIcon } from '@/assets/icons/api.svg';
import { ReactComponent as SubjectIcon } from '@/assets/icons/subject.svg';
import { ReactComponent as WidgetIcon } from '@/assets/icons/widgets.svg';
import { ReactComponent as ListIcon } from '@/assets/icons/list.svg';

type ViewPickerProps = {
  isOpen: boolean;

  bottom: string;
  right: string;

  onClick: (item: ConfigurableView) => void;
};

const Container = (props: PropsWithChildren<ViewPickerProps>) => (
  <div
    className="pointer-events-none fixed grid grid-cols-2 gap-x-6 gap-y-5"
    style={{ bottom: props.bottom, right: props.right }}
  >
    {props.children}
  </div>
);

const CardButton = ({
  isOpen,
  customStyles,
  index,
  children,
  ...props
}: PropsWithChildren<{
  isOpen: boolean;
  customStyles: string;
  index: number;
}> &
  JSX.IntrinsicElements['button']) => (
  <button
    className={clsx(
      'flex transform items-center justify-start',
      'rounded bg-white px-4 py-4 shadow-md',
      'border border-gray-300 transition hover:border-gray-500 hover:shadow-lg',
      'ring-2 ring-transparent hover:-translate-y-0.5',
      'focus:-translate-y-0.5 focus:border-0 focus:outline-none',
      'dark:border-slate-500/70 dark:bg-slate-700 dark:hover:border-slate-300',
      'dark:shadow-slate-400/10 dark:hover:shadow-slate-400/20',
      isOpen
        ? 'pointer-events-auto scale-100 opacity-100'
        : 'pointer-events-none scale-75 opacity-0',
      customStyles,
    )}
    style={{
      transitionDelay: `${8 * Math.pow(index, 1.5)}ms`,
    }}
    {...props}
  >
    {children}
  </button>
);

const listContent = [
  {
    title: 'OpMode View',
    view: ConfigurableView.OPMODE_VIEW,
    icon: <WidgetIcon className="h-6 w-6" />,
    customStyles: 'focus:ring-red-600',
    iconBg: 'bg-red-500',
  },
  {
    title: 'Field View',
    view: ConfigurableView.FIELD_VIEW,
    icon: <ApiIcon className="h-7 w-7 rotate-45 transform" />,
    customStyles: 'focus:ring-blue-600',
    iconBg: 'bg-blue-500',
  },
  {
    title: 'Graph View',
    view: ConfigurableView.GRAPH_VIEW,
    icon: <ChartIcon className="h-6 w-6 text-white" />,
    customStyles: 'focus:ring-green-600',
    iconBg: 'bg-green-500',
  },
  {
    title: 'Config View',
    view: ConfigurableView.CONFIG_VIEW,
    icon: <SettingsIcon className="h-6 w-6" />,
    customStyles: 'focus:ring-orange-600',
    iconBg: 'bg-orange-500',
  },
  {
    title: 'Telemetry View',
    view: ConfigurableView.TELEMETRY_VIEW,
    icon: <ListIcon className="h-7 w-7" />,
    customStyles: 'focus:ring-yellow-600',
    iconBg: 'bg-yellow-500',
  },
  {
    title: 'Logging View',
    view: ConfigurableView.LOGGING_VIEW,
    icon: <SubjectIcon className="h-6 w-6" />,
    customStyles: 'focus:ring-ping-600',
    iconBg: 'bg-pink-500',
  },
  {
    title: 'Camera View',
    view: ConfigurableView.CAMERA_VIEW,
    icon: <CameraIcon className="h-5 w-5" />,
    customStyles: 'focus:ring-purple-600',
    iconBg: 'bg-purple-500',
  },
];

const ViewPicker = (props: ViewPickerProps) => {
  return (
    <Container {...props}>
      {listContent.map((item, index) => (
        <CardButton
          key={item.title}
          {...props}
          index={index}
          customStyles={item.customStyles}
          onClick={() => props.onClick(item.view)}
          disabled={!props.isOpen}
        >
          <>
            <div className={`flex-center mr-3  h-8 w-8 rounded ${item.iconBg}`}>
              {cloneElement(item.icon)}
            </div>
            <div className="flex flex-col items-start">
              <h3 className="mt-0 text-lg font-medium leading-4">
                {item.title}
              </h3>
            </div>
          </>
        </CardButton>
      ))}
    </Container>
  );
};

export default ViewPicker;
