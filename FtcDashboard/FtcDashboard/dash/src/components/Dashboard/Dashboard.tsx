import { useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';

import LayoutPreset, { LayoutPresetType } from '@/enums/LayoutPreset';
import { saveLayoutPreset, getLayoutPreset } from '@/store/actions/settings';
import { RootState } from '@/store/reducers';

import { BaseViewIconButton } from '@/components/views/BaseView';
import { ReactComponent as ConnectedIcon } from '@/assets/icons/connected.svg';
import { ReactComponent as DisconnectedIcon } from '@/assets/icons/disconnected.svg';
import { ReactComponent as SettingsIcon } from '@/assets/icons/settings.svg';
import SettingsModal from './SettingsModal';
import { startSocketWatcher } from '@/store/middleware/socketMiddleware';

export default function Dashboard() {
  const socket = useSelector((state: RootState) => state.socket);
  const layoutPreset = useSelector(
    (state: RootState) => state.settings.layoutPreset,
  );
  const enabled = useSelector((state: RootState) => state.status.enabled);
  const batteryVoltage = useSelector(
    (state: RootState) => state.status.batteryVoltage,
  );
  const dispatch = useDispatch();

  const [isSettingsModalOpen, setIsSettingsModalOpen] = useState(false);

  useEffect(() => {
    dispatch(getLayoutPreset());

    startSocketWatcher(dispatch);
  }, [dispatch]);

  return (
    <div
      className="flex flex-col text-black dark:text-white"
      style={{ width: '100vw', height: '100vh' }}
    >
      <header className="flex items-center justify-between bg-primary-600 px-3 py-1 text-white">
        <h1 className="text-2xl font-medium">FTC Dashboard</h1>
        <div className="flex-center">
          <select
            className="mx-2 rounded border-primary-300 bg-primary-100 py-1 text-sm text-black focus:border-primary-100 focus:ring-2 focus:ring-white focus:ring-opacity-40"
            value={layoutPreset as LayoutPresetType}
            onChange={(evt) =>
              dispatch(saveLayoutPreset(evt.target.value as LayoutPresetType))
            }
          >
            {Object.keys(LayoutPreset)
              .filter(
                (key) =>
                  typeof LayoutPreset[key as LayoutPresetType] === 'string',
              )
              .map((key) => (
                <option key={key} value={key}>
                  {LayoutPreset.getName(key as LayoutPresetType)}
                </option>
              ))}
          </select>
          {socket.isConnected && (
            <p
              className="mx-2"
              style={{
                width: batteryVoltage > 0 ? '120px' : '60px',
                textAlign: 'right',
              }}
            >
              {socket.pingTime}ms
              {batteryVoltage > 0 ? ` / ${batteryVoltage.toFixed(2)}V` : ''}
            </p>
          )}
          {socket.isConnected ? (
            <ConnectedIcon className="ml-4 h-10 w-10 py-1" />
          ) : (
            <DisconnectedIcon className="ml-4 h-10 w-10 py-1" />
          )}
          <BaseViewIconButton
            title="Settings"
            className="icon-btn group ml-3 h-8 w-8 hover:border-white/50"
            onClick={() => setIsSettingsModalOpen(true)}
          >
            <SettingsIcon className="h-7 w-7 transition group-hover:rotate-[15deg] group-focus:rotate-[15deg]" />
          </BaseViewIconButton>
        </div>
      </header>
      {socket.isConnected && !enabled ? (
        <div
          style={{
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
            height: '100%',
          }}
        >
          <div
            className="justify-self-center text-center"
            style={{ maxWidth: '600px' }}
          >
            <h1 className="text-xl font-medium">FTC Dashboard is Disabled</h1>
            <p>
              To re-enable, run the &quot;Enable/Disable Dashboard&quot; op mode
              or select &quot;Enable Dashboard&quot; from the RC menu
            </p>
          </div>
        </div>
      ) : (
        LayoutPreset.getContent(layoutPreset as LayoutPresetType)
      )}
      <SettingsModal
        isOpen={isSettingsModalOpen}
        onClose={() => setIsSettingsModalOpen(false)}
      />
      {/* Insert a headless-ui portal so the .set-theme-x styles apply to the headless ui dialogs. */}
      {/* They are rendered as siblings to the root by default, outside of our scope */}
      <div id="headlessui-portal-root">
        {/* Leave an empty div here. Otherwise, headless-ui will remove this container on dialog close */}
        <div />
      </div>
    </div>
  );
}
