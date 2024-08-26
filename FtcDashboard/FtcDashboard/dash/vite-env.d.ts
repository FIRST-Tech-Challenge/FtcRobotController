import { Config } from 'tailwindcss';

declare module 'tailwind.config.js' {
  const config: Config;
  export default config;
}
