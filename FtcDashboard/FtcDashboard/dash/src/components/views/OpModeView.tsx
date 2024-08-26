import { Component, ChangeEvent, createRef, MutableRefObject } from 'react';
import { connect, ConnectedProps } from 'react-redux';

import { RootState } from '@/store/reducers';
import { initOpMode, startOpMode, stopOpMode } from '@/store/actions/opmode';
import OpModeStatus from '@/enums/OpModeStatus';
import BaseView, {
  BaseViewHeading,
  BaseViewBody,
  BaseViewIcon,
  BaseViewIcons,
  BaseViewProps,
  BaseViewHeadingProps,
} from './BaseView';

import { ReactComponent as GamepadIcon } from '@/assets/icons/gamepad.svg';
import { ReactComponent as GamepadNotSupportedIcon } from '@/assets/icons/gamepad_not_supported.svg';
import { STOP_OP_MODE_TAG } from '@/store/types/opmode';
import ToolTip from '@/components/ToolTip';

type OpModeViewState = {
  selectedOpMode: string;
  shouldShowGamepadUnsupportedTooltip: boolean;
};

const mapStateToProps = ({ status, gamepad }: RootState) => ({
  ...status,
  ...gamepad,
});

const mapDispatchToProps = {
  initOpMode: (opModeName: string) => initOpMode(opModeName),
  startOpMode: () => startOpMode(),
  stopOpMode: () => stopOpMode(),
};

const connector = connect(mapStateToProps, mapDispatchToProps);

type OpModeViewProps = ConnectedProps<typeof connector> &
  BaseViewProps &
  BaseViewHeadingProps;

const ActionButton = ({
  children,
  className,
  ...props
}: JSX.IntrinsicElements['button']) => (
  <button
    className={`ml-2 rounded-md border py-1 px-3 shadow-md ${className}`}
    {...props}
  >
    {children}
  </button>
);

class OpModeView extends Component<OpModeViewProps, OpModeViewState> {
  gamepadUnsupportedTooltipRef: MutableRefObject<HTMLDivElement | null>;
  gamepadUnsupportedTooltipTimeout: ReturnType<typeof setTimeout> | null = null;

  constructor(props: OpModeViewProps) {
    super(props);

    this.state = {
      selectedOpMode: '',
      shouldShowGamepadUnsupportedTooltip: false,
    };
    this.gamepadUnsupportedTooltipRef = createRef();

    this.onChange = this.onChange.bind(this);
  }

  gamepadIconsHover() {
    let myTimeout: ReturnType<typeof setTimeout> | null = null;

    this.gamepadUnsupportedTooltipTimeout = setTimeout(() => {
      if (this.gamepadUnsupportedTooltipTimeout === myTimeout)
        this.setState((prev) => ({
          ...prev,
          shouldShowGamepadUnsupportedTooltip: true,
        }));
    }, 500);
    myTimeout = this.gamepadUnsupportedTooltipTimeout;
  }
  gamepadIconsUnhover() {
    this.gamepadUnsupportedTooltipTimeout = null;
    this.setState((prev) => ({
      ...prev,
      shouldShowGamepadUnsupportedTooltip: false,
    }));
  }

  static getDerivedStateFromProps(
    props: OpModeViewProps,
    state: OpModeViewState,
  ) {
    if (props.activeOpMode !== STOP_OP_MODE_TAG) {
      return {
        selectedOpMode: props.activeOpMode,
      };
    } else if (
      state.selectedOpMode === '' ||
      props.opModeList.indexOf(state.selectedOpMode) === -1
    ) {
      return {
        selectedOpMode: props.opModeList[0] || '',
      };
    } else {
      return {};
    }
  }

  onChange(evt: ChangeEvent<HTMLSelectElement>) {
    this.setState({
      selectedOpMode: evt.target.value,
    });
  }

  renderInitButton() {
    return (
      <ActionButton
        className={`
          border-blue-300 bg-blue-200 transition-colors
          dark:border-transparent dark:bg-blue-600 dark:text-blue-50 dark:highlight-white/30
          dark:hover:border-blue-400/80 dark:focus:bg-blue-700
        `}
        onClick={() => this.props.initOpMode(this.state.selectedOpMode)}
      >
        Init
      </ActionButton>
    );
  }

  renderStartButton() {
    return (
      <ActionButton
        className={`
          border-green-200 bg-green-100 transition-colors
          dark:border-transparent dark:bg-green-500 dark:text-green-50 dark:highlight-white/30
          dark:hover:border-green-300/80 dark:focus:bg-green-600
        `}
        onClick={() => this.props.startOpMode()}
      >
        Start
      </ActionButton>
    );
  }

  renderStopButton() {
    return (
      <ActionButton
        className={`
          border-red-200 bg-red-100 transition-colors
          dark:border-transparent dark:bg-red-500 dark:text-red-50 dark:highlight-white/30
          dark:hover:border-red-300/80 dark:focus:bg-red-600
        `}
        onClick={() => this.props.stopOpMode()}
      >
        Stop
      </ActionButton>
    );
  }

  renderButtons() {
    const { activeOpMode, activeOpModeStatus, opModeList } = this.props;

    if (opModeList.length === 0) {
      return null;
    } else if (activeOpMode === STOP_OP_MODE_TAG) {
      return this.renderInitButton();
    } else if (activeOpModeStatus === OpModeStatus.INIT) {
      return (
        <span>
          {this.renderStartButton()}
          {this.renderStopButton()}
        </span>
      );
    } else if (activeOpModeStatus === OpModeStatus.RUNNING) {
      return this.renderStopButton();
    } else if (activeOpModeStatus === OpModeStatus.STOPPED) {
      return null;
    } else {
      return <p>Unknown op mode status: {activeOpModeStatus}</p>;
    }
  }

  render() {
    const {
      available,
      activeOpMode,
      opModeList,
      warningMessage,
      errorMessage,
      gamepadsSupported,
    } = this.props;

    const { gamepad1Connected, gamepad2Connected } = this.props;

    if (!available) {
      return (
        <BaseView isUnlocked={this.props.isUnlocked}>
          <BaseViewHeading isDraggable={this.props.isDraggable}>
            Op Mode
          </BaseViewHeading>
          <BaseViewBody className="flex-center">
            <h3 className="text-md text-center">
              Op mode controls have not initialized
            </h3>
          </BaseViewBody>
        </BaseView>
      );
    }

    const isShowingGamepadUnsupportedTooltip =
      !gamepadsSupported && this.state.shouldShowGamepadUnsupportedTooltip;

    return (
      <BaseView isUnlocked={this.props.isUnlocked}>
        <div className="flex">
          <BaseViewHeading isDraggable={this.props.isDraggable}>
            Op Mode
          </BaseViewHeading>
          <div
            onPointerEnter={this.gamepadIconsHover.bind(this)}
            onPointerLeave={this.gamepadIconsUnhover.bind(this)}
            ref={this.gamepadUnsupportedTooltipRef}
          >
            <BaseViewIcons>
              <BaseViewIcon>
                {gamepadsSupported ? (
                  <GamepadIcon
                    className="h-6 w-6"
                    style={{
                      opacity: gamepad1Connected ? 1.0 : 0.3,
                    }}
                  />
                ) : (
                  <GamepadNotSupportedIcon className="h-6 w-6" />
                )}
              </BaseViewIcon>
              <BaseViewIcon>
                {gamepadsSupported ? (
                  <GamepadIcon
                    className="h-6 w-6"
                    style={{
                      opacity: gamepad2Connected ? 1.0 : 0.3,
                    }}
                  />
                ) : (
                  <GamepadNotSupportedIcon className="h-6 w-6" />
                )}
              </BaseViewIcon>
            </BaseViewIcons>
            <ToolTip
              isShowing={isShowingGamepadUnsupportedTooltip}
              hoverRef={this.gamepadUnsupportedTooltipRef}
            >
              Due to changes to JavaScript, gamepads are no longer supported in
              your browser.
            </ToolTip>
          </div>
        </div>
        <BaseViewBody>
          <select
            className={`
              m-1 mr-2 rounded border border-gray-300 bg-gray-200 p-1 pr-6 
              shadow-md transition focus:border-primary-500
              focus:ring-primary-500 disabled:text-gray-600 disabled:shadow-none
              dark:border-slate-500/80 dark:bg-slate-700 dark:text-slate-200
            `}
            value={this.state.selectedOpMode}
            disabled={
              activeOpMode !== STOP_OP_MODE_TAG || opModeList.length === 0
            }
            onChange={this.onChange}
          >
            {opModeList.length === 0 ? (
              <option>Loading...</option>
            ) : (
              opModeList
                .sort()
                .map((opMode: string) => <option key={opMode}>{opMode}</option>)
            )}
          </select>
          {this.renderButtons()}
          {errorMessage !== '' && (
            <p className="error mt-5 ml-1">Error: {errorMessage}</p>
          )}
          {warningMessage !== '' && (
            <p className="warning mt-5 ml-1">Warning: {warningMessage}</p>
          )}
        </BaseViewBody>
      </BaseView>
    );
  }
}

export default connector(OpModeView);
