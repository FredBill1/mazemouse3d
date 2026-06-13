import { defineConfig } from "vite";

export default defineConfig({
  server: {
    port: 5173,
    strictPort: false,
  },
  test: {
    environment: "node",
    exclude: ["coverage/**", "dist/**", "node_modules/**", "src/generated/**", "target/**"],
  },
});
