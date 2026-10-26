/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,ts,jsx,tsx}'],
  theme: {
    extend: {
      colors: {
        deep: {
          700: '#172538',
          800: '#101b2b',
          900: '#080f1c',
        },
      },
      opacity: {
        15: '0.15',
        25: '0.25',
      },
    },
  },
  plugins: [],
};
