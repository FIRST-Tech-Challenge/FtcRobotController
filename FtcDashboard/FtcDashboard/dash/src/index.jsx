import { createRoot } from 'react-dom/client';
import { Provider } from 'react-redux';

import Dashboard from './components/Dashboard/Dashboard';
import configureStore from './store/configureStore';
import { ThemeProvider } from '@/hooks/useTheme';

import './index.css';

const store = configureStore();
const root = createRoot(document.getElementById('root'));
root.render(
  <ThemeProvider>
    <Provider store={store}>
      <Dashboard />
    </Provider>
  </ThemeProvider>,
);
