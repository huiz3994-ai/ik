import { Suspense, useEffect, useMemo, useRef, useState } from 'react'
import { Canvas, useLoader, useThree } from '@react-three/fiber'
import { Grid, OrbitControls, PerspectiveCamera } from '@react-three/drei'
import { Matrix4, MeshStandardMaterial, Quaternion, Vector3 } from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { AlertTriangle, CheckCircle2, Circle, LoaderCircle, ShieldAlert, WifiOff } from 'lucide-react'
import type { JointDescriptor, RobotDescriptor, StationState, VisualDescriptor } from './types'

export function StatusPill({label, ok, pending=false, detail}: {label:string; ok:boolean; pending?:boolean; detail?:string}) {
  const cls = pending ? 'pending' : ok ? 'ok' : 'bad'
  const Icon = pending ? LoaderCircle : ok ? CheckCircle2 : WifiOff
  return <div className={`status-pill ${cls}`} title={detail}><Icon size={14}/><span>{label}</span><b>{pending?'启动中':ok?'运行中':'未连接'}</b></div>
}

export function StateBadge({state}: {state?: string}) {
  const danger = state === 'SAFE_STOP'
  return <div className={`state-badge ${danger ? 'danger':''}`}>
    {danger ? <ShieldAlert size={17}/> : <Circle size={10} fill="currentColor"/>}
    {state || 'UNKNOWN'}
  </div>
}

export function Panel({title, eyebrow, children, className=''}: {title:string; eyebrow?:string; children:React.ReactNode; className?:string}) {
  return <section className={`panel ${className}`}><header>{eyebrow&&<span>{eyebrow}</span>}<h2>{title}</h2></header>{children}</section>
}

function matrix(values: number[]) { return new Matrix4().fromArray(values).transpose() }

function Visual({visual, material}: {visual: VisualDescriptor; material: MeshStandardMaterial}) {
  const geometry = useLoader(STLLoader, visual.mesh)
  const transform = useMemo(() => matrix(visual.matrix), [visual.matrix])
  return <group matrixAutoUpdate={false} matrix={transform}>
    <mesh geometry={geometry} scale={visual.scale} material={material} castShadow receiveShadow />
  </group>
}

function RobotLink({descriptor, link, jointIndex, values, material, frames}: {
  descriptor: RobotDescriptor; link:string; jointIndex:number; values:number[];
  material:MeshStandardMaterial; frames:boolean
}) {
  const visuals = descriptor.visuals.filter(v => v.link === link)
  const joint = descriptor.joints[jointIndex]
  const origin = useMemo(() => joint ? matrix(joint.matrix) : new Matrix4(), [joint])
  const rotation = useMemo(() => joint ? new Quaternion().setFromAxisAngle(
    new Vector3(...joint.axis).normalize(), values[jointIndex] || 0) : new Quaternion(), [joint, jointIndex, values])
  return <>
    {visuals.map(v => <Visual key={v.id} visual={v} material={material}/>) }
    {joint && joint.parent === link && <group matrixAutoUpdate={false} matrix={origin}>
      {frames && <axesHelper args={[0.09]}/>} 
      <group quaternion={rotation}>
        <RobotLink descriptor={descriptor} link={joint.child} jointIndex={jointIndex+1}
          values={values} material={material} frames={frames}/>
      </group>
    </group>}
  </>
}

function CameraRig({view}: {view:string}) {
  const {camera} = useThree()
  useMemo(() => {
    const positions: Record<string,[number,number,number]> = {
      iso:[1.35,-1.45,1.15], front:[1.8,0,0.75], side:[0,-1.8,0.75], top:[0,0,2.25]
    }
    camera.position.set(...(positions[view] || positions.iso)); camera.up.set(0,0,1); camera.lookAt(0,0,0.45)
  }, [camera, view])
  return null
}

export function RobotTwin({descriptor, actual, target, actualFresh}: {
  descriptor:RobotDescriptor; actual:number[]; target:number[]; actualFresh:boolean
}) {
  const [view,setView]=useState('iso'); const [frames,setFrames]=useState(false)
  const [actualOn,setActualOn]=useState(true); const [targetOn,setTargetOn]=useState(true)
  const materials = useMemo(() => ({
    actual: new MeshStandardMaterial({color: actualFresh ? '#d7e3e8':'#e25353', roughness:.42, metalness:.28, transparent:!actualFresh, opacity:actualFresh?1:.45}),
    target: new MeshStandardMaterial({color:'#20d6cb', roughness:.25, metalness:.1, transparent:true, opacity:.30, depthWrite:false})
  }), [actualFresh])
  return <div className="twin-shell">
    <div className="twin-toolbar">
      {['iso','front','side','top'].map(v=><button key={v} className={view===v?'active':''} onClick={()=>setView(v)}>{({iso:'等轴',front:'正视',side:'侧视',top:'俯视'} as Record<string,string>)[v]}</button>)}
      <i/>
      <label><input type="checkbox" checked={actualOn} onChange={e=>setActualOn(e.target.checked)}/> 实际</label>
      <label><input type="checkbox" checked={targetOn} onChange={e=>setTargetOn(e.target.checked)}/> 目标</label>
      <label><input type="checkbox" checked={frames} onChange={e=>setFrames(e.target.checked)}/> 坐标系</label>
    </div>
    <Canvas shadows gl={{antialias:true}}>
      <color attach="background" args={['#071018']}/><fog attach="fog" args={['#071018',2.1,4.5]}/>
      <PerspectiveCamera makeDefault position={[1.35,-1.45,1.15]} fov={42}/><CameraRig view={view}/>
      <ambientLight intensity={1.1}/><directionalLight position={[2,-2,3]} intensity={2.4} castShadow/>
      <Suspense fallback={null}>
        {actualOn&&<RobotLink descriptor={descriptor} link="base_link" jointIndex={0} values={actual} material={materials.actual} frames={frames}/>} 
        {targetOn&&<RobotLink descriptor={descriptor} link="base_link" jointIndex={0} values={target} material={materials.target} frames={false}/>} 
      </Suspense>
      <Grid args={[5,5]} cellSize={.1} cellThickness={.5} cellColor="#254050" sectionSize={.5} sectionColor="#396170" fadeDistance={4} infiniteGrid rotation={[Math.PI/2,0,0]}/>
      <axesHelper args={[.25]}/><OrbitControls makeDefault target={[0,0,.45]} enableDamping/>
    </Canvas>
    <div className={`live-mark ${actualFresh?'':'stale'}`}>{actualFresh?'LIVE / 实时反馈':'STALE / 反馈过期'}</div>
  </div>
}

export function JointPanel({state, model, target, onTarget, onExecute, onCancel}: {
  state:StationState; model:RobotDescriptor; target:number[]; onTarget:(v:number[])=>void;
  onExecute:()=>void; onCancel:()=>void
}) {
  const actual = Array.from({length:7},(_,i)=>state.joints?.[`joint${i+1}`])
  const sync=()=>actual.every(Number.isFinite)&&onTarget(actual as number[])
  const targetRef=useRef(target); targetRef.current=target
  const holdTimer=useRef<number|undefined>(undefined)
  const set=(i:number,v:number)=>{const n=[...targetRef.current];n[i]=v;targetRef.current=n;onTarget(n)}
  const stopHold=()=>{if(holdTimer.current!==undefined){window.clearInterval(holdTimer.current);holdTimer.current=undefined}}
  const startHold=(i:number,direction:number,joint:JointDescriptor)=>{stopHold();const step=()=>set(i,Math.max(joint.lower,Math.min(joint.upper,targetRef.current[i]+direction*Math.PI/180)));step();holdTimer.current=window.setInterval(step,180)}
  useEffect(()=>stopHold,[])
  return <div className="joint-panel">
    <div className="joint-head"><span>轴</span><span>实际</span><span>速度</span><span>目标 / 误差</span><span>限位余量</span></div>
    {model.joints.map((j,i)=>{
      const a=actual[i], t=target[i], margin=Number.isFinite(a)?Math.min(a-j.lower,j.upper-a):NaN
      return <div className="joint-row" key={j.name}>
        <b>J{i+1}</b><span>{Number.isFinite(a)?`${(a*180/Math.PI).toFixed(2)}°`:'—'}</span>
        <span>{Number.isFinite(state.joint_velocities?.[j.name])?`${(state.joint_velocities[j.name]*180/Math.PI).toFixed(2)}°/s`:'不可用'}</span>
        <div className="joint-target"><input type="number" step="0.1" value={(t*180/Math.PI).toFixed(2)} onChange={e=>set(i,Number(e.target.value)*Math.PI/180)}/>
          <small>{Number.isFinite(a)?`Δ ${((t-a)*180/Math.PI).toFixed(2)}°`:'Δ —'}</small>
          <input type="range" min={j.lower} max={j.upper} step="0.001" value={t} onChange={e=>set(i,Number(e.target.value))}/>
          <button onPointerDown={()=>startHold(i,-1,j)} onPointerUp={stopHold} onPointerLeave={stopHold} onPointerCancel={stopHold}>−</button><button onPointerDown={()=>startHold(i,1,j)} onPointerUp={stopHold} onPointerLeave={stopHold} onPointerCancel={stopHold}>+</button>
        </div><span className={margin<.08?'warn':''}>{Number.isFinite(margin)?`${(margin*180/Math.PI).toFixed(1)}°`:'—'}<small>{(j.lower*180/Math.PI).toFixed(0)}°…{(j.upper*180/Math.PI).toFixed(0)}°</small></span>
      </div>})}
    <div className="joint-actions"><button onClick={sync}>同步实际姿态</button><button className="secondary" onClick={onCancel}>取消执行</button><button className="primary" onClick={onExecute}>执行七轴目标</button></div>
    <p className="safety-note"><AlertTriangle size={14}/>滑块、输入框和 ± 仅修改离线预览；执行会自动经过既有 URDF 限位、反馈时效、控制源互斥和 MoveIt/FCL 检查。</p>
  </div>
}

export function Metric({label,value,unit,accent}: {label:string;value:string|number;unit?:string;accent?:boolean}) {
  return <div className={`metric ${accent?'accent':''}`}><span>{label}</span><strong>{value}</strong>{unit&&<em>{unit}</em>}</div>
}
