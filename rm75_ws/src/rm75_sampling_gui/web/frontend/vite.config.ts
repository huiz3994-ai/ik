import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    host: '127.0.0.1',
    port: 5173,
    proxy: {
      '/api': { target: 'http://127.0.0.1:8088', ws: true }
    }
  },
  build: {
    target: 'es2020',
    sourcemap: false,
    rollupOptions: { output: { manualChunks: {
      react: ['react', 'react-dom', 'zustand'],
      three: ['three', '@react-three/fiber', '@react-three/drei'],
      charts: ['recharts']
    } } }
  }
})
