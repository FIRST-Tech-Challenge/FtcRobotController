import { Middleware } from 'redux';

import LayoutPreset, { LayoutPresetType } from '@/enums/LayoutPreset';
import { GET_LAYOUT_PRESET, SAVE_LAYOUT_PRESET } from '@/store/types';
import { receiveLayoutPreset } from '@/store/actions/settings';
import { RootState } from '@/store/reducers';

const LAYOUT_PRESET_KEY = 'layoutPreset';

const storageMiddleware: Middleware<Record<string, unknown>, RootState> =
  (store) => (next) => (action) => {
    switch (action.type) {
      case GET_LAYOUT_PRESET: {
        const preset =
          localStorage.getItem(LAYOUT_PRESET_KEY) || LayoutPreset.DEFAULT;

        store.dispatch(receiveLayoutPreset(preset as LayoutPresetType));

        break;
      }
      case SAVE_LAYOUT_PRESET: {
        localStorage.setItem(LAYOUT_PRESET_KEY, action.preset);

        store.dispatch(receiveLayoutPreset(action.preset));

        break;
      }
      default:
        next(action);

        break;
    }
  };

export default storageMiddleware;
