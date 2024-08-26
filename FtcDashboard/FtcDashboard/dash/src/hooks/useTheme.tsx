import {
  createContext,
  PropsWithChildren,
  useReducer,
  useContext,
  Dispatch,
  useEffect,
} from 'react';

import twColors from 'tailwindcss/colors';
import { KeysWithType } from '@/typeHelpers';

const BLACK_LIST_COLORS = [
  'lightBlue',
  'warmGray',
  'trueGray',
  'coolGray',
  'blueGray',
] as const;
export const colors = Object.fromEntries(
  Object.entries(twColors).filter(
    ([name, c]) =>
      typeof c !== 'string' &&
      !(BLACK_LIST_COLORS as unknown as string[]).includes(name),
  ),
);

export type Colors = KeysWithType<
  Omit<typeof twColors, typeof BLACK_LIST_COLORS[number]>,
  Record<string, unknown>
>;

type ThemeState = {
  theme: Colors;
  isDarkMode: boolean;
};

type ThemeAction =
  | { type: 'setTheme'; payload: Colors }
  | { type: 'setDarkMode'; payload: boolean };

enum LocalStorageKeys {
  isDarkMode = 'isDarkMode',
  themeColor = 'themeColor',
}

function isDarkModeOnLoad(): boolean {
  return (
    localStorage[LocalStorageKeys.isDarkMode] === 'dark' ||
    (!(LocalStorageKeys.isDarkMode in localStorage) &&
      window.matchMedia('(prefers-color-scheme: dark)').matches)
  );
}

function themeOnLoad(): Colors {
  return (localStorage[LocalStorageKeys.themeColor] as Colors) || 'blue';
}

const initialState: ThemeState = {
  theme: themeOnLoad(),
  isDarkMode: isDarkModeOnLoad(),
};

function themeReducer(state: ThemeState, action: ThemeAction): ThemeState {
  switch (action.type) {
    case 'setTheme':
      return { ...state, theme: action.payload };
    case 'setDarkMode':
      return { ...state, isDarkMode: action.payload };
    default:
      return state;
  }
}

const ThemeContext = createContext(initialState);
const ThemeDispatchContext = createContext<Dispatch<ThemeAction>>(
  () => initialState,
);

export const ThemeConsumer = ThemeContext.Consumer;

export const ThemeProvider = (props: PropsWithChildren) => {
  const [theme, dispatch] = useReducer(themeReducer, initialState);

  useEffect(() => {
    document.documentElement.classList.toggle('dark', theme.isDarkMode);
    document.body.classList.toggle('bg-slate-900', theme.isDarkMode);
    localStorage.setItem('isDarkMode', theme.isDarkMode ? 'dark' : 'light');
  }, [theme.isDarkMode]);

  useEffect(() => {
    const target = document.body;
    const currentColor = [...target.classList].find((e) =>
      e.startsWith('set-theme-'),
    );
    if (currentColor) target.classList.remove(currentColor);
    target.classList.add(`set-theme-${theme.theme}`);

    // Sync theme color with theme-color meta tag (for Safari tab color)
    document
      .querySelector('meta[name="theme-color"]')
      ?.setAttribute('content', colors[theme.theme][600]);

    // Sync to local storage
    localStorage.setItem('themeColor', theme.theme);
  }, [theme.theme]);

  return (
    <ThemeContext.Provider value={theme}>
      <ThemeDispatchContext.Provider value={dispatch}>
        {props.children}
      </ThemeDispatchContext.Provider>
    </ThemeContext.Provider>
  );
};

export function useTheme() {
  return useContext(ThemeContext);
}

export function useThemeDispatch() {
  return useContext(ThemeDispatchContext);
}
