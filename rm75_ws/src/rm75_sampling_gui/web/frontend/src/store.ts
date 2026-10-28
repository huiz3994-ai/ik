import { create } from 'zustand'
import type { RobotDescriptor, StationState } from './types'

type View = 'dashboard'|'robot'|'cartesian'|'force'|'qr'|'tools'|'robotctl'|'sampling'|'diagnostics'|'parameters'
export type Planner = 'ompl'|'lin'|'ptp'|'direct'

interface Store {
  view: View
  state: StationState | null
  model: RobotDescriptor | null
  target: number[]
  targetEdited: boolean
  sessionId: string | null
  sessionReady: boolean
  connected: boolean
  lastError: string
  planner: Planner
  speed: number
  setPlanner: (v: Planner) => void
  setSpeed: (v: number) => void
  loadSettings: () => Promise<void>
  setView: (v: View) => void
  setState: (s: StationState) => void
  setModel: (m: RobotDescriptor) => void
  setTarget: (t: number[], edited?: boolean) => void
  setSession: (id: string | null) => void
  setSessionReady: (ready: boolean) => void
  setConnected: (v: boolean) => void
  setError: (v: string) => void
}

export const useStation = create<Store>((set, get) => ({
  view: 'dashboard', state: null, model: null, target: Array(7).fill(0),
  targetEdited: false, sessionId: null, sessionReady: false,
  connected: false, lastError: '',
  planner: 'ompl', speed: 5,
  setView: view => set({view}),
  setState: state => {
    const actual = Array.from({length: 7}, (_, i) => state.joints?.[`joint${i+1}`])
    const canSync = !get().targetEdited && actual.every(Number.isFinite)
    set(canSync ? {state, target: actual as number[]} : {state})
  },
  setModel: model => set({model}),
  setTarget: (target, edited = true) => set({target, targetEdited: edited}),
  setSession: sessionId => set({sessionId, sessionReady: false}),
  setSessionReady: sessionReady => set({sessionReady}),
  setConnected: connected => set({connected}),
  setError: lastError => set({lastError}),
  setPlanner: planner => { set({planner}); void saveSettings(planner, get().speed) },
  setSpeed: speed => { set({speed}); void saveSettings(get().planner, speed) },
  loadSettings: async () => {
    try {
      const value = await api<{planner:Planner;speed:number}>('/api/v1/settings')
      set({planner:value.planner,speed:value.speed})
    } catch { /* retain safe defaults while backend is unavailable */ }
  },
}))

async function saveSettings(planner: Planner, speed: number) {
  await fetch('/api/v1/settings', {method:'PUT', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({planner,speed})})
}

const CONTROL_PATH = /^\/api\/v1\/(cartesian\/(execute|jog\/)|joints\/execute|commands\/|poses\/|control\/path|limit-recovery)/

export async function api<T>(path: string, init: RequestInit = {}): Promise<T> {
  if (CONTROL_PATH.test(path) && !(path === '/api/v1/limit-recovery' && (!init.method || init.method === 'GET'))) {
    const {sessionId,sessionReady}=useStation.getState()
    if (!sessionId) throw new Error('只读模式：请先取得控制权')
    if (!sessionReady) throw new Error('控制权心跳尚未就绪')
  }
  const session = useStation.getState().sessionId
  const headers = new Headers(init.headers)
  if (init.body) headers.set('Content-Type', 'application/json')
  if (session) headers.set('X-RM75-Control-Session', session)
  const response = await fetch(path, {...init, headers})
  const body = await response.json().catch(() => ({}))
  if (!response.ok) throw new Error(body.detail || `HTTP ${response.status}`)
  return body as T
}

export async function command(name: string, payload: Record<string, unknown> = {}) {
  return api(`/api/v1/commands/${name}`, {method: 'POST', body: JSON.stringify({payload})})
}
