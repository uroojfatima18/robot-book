import type { Config } from "tailwindcss";

const config: Config = {
  content: [
    "./src/pages/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/components/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/app/**/*.{js,ts,jsx,tsx,mdx}",
  ],
  theme: {
    extend: {
      colors: {
        background: "var(--background)",
        foreground: "var(--foreground)",
        primary: {
          50: "#f0f4f8",
          100: "#e1e9f2",
          200: "#c2d3e6",
          300: "#94b5d4",
          400: "#5a86a2",
          500: "#2d4a6a",
          600: "#1a365d",
          700: "#132844",
          800: "#0e1e33",
          900: "#091220",
        },
        secondary: {
          50: "#f0fafb",
          100: "#e0f5f6",
          200: "#c0ebee",
          300: "#90dfe0",
          400: "#40c5c7",
          500: "#008080",
          600: "#006666",
          700: "#005252",
          800: "#003d3d",
          900: "#002929",
        },
        accent: {
          50: "#fdf8ee",
          100: "#fbf1dd",
          200: "#f7e3bb",
          300: "#f0d08c",
          400: "#e4b85c",
          500: "#d4af37",
          600: "#c09c25",
          700: "#9c7d1d",
          800: "#7d6318",
          900: "#604b13",
        },
      },
      fontFamily: {
        sans: ['Inter', 'system-ui', 'sans-serif'],
      },
    },
  },
  plugins: [],
};
export default config;
