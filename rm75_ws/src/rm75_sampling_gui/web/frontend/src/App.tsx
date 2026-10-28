import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { Activity, ArrowDown, ArrowUp, Bot, Camera, ChevronRight, CircleGauge, FileSliders, FlaskConical, FolderOpen, Gamepad2, Menu, Move3D, OctagonX, PackageCheck, Play, Plus, Power, QrCode, Radio, RefreshCw, RotateCw, Save, Server, ShieldCheck, Siren, Square, Trash2, Unplug, Waves, Waypoints } from 'lucide-react'
import { CartesianGrid, Legend, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts'
import { JointPanel, Metric, Panel, RobotTwin, StateBadge, StatusPill } from './components'
import { api, command, useStation } from './store'
import type { Dict, RM75Trajectory, RobotDescriptor, StationState, ToolPickConfig, TrajectoryExecution, TrajectorySummary } from './types'

const nav = [
  ['dashboard','操作台',CircleGauge],['robot','机器人状态',Bot],['cartesian','末端位置控制',Move3D],['trajectory','路径示教',Waypoints],['force','实时力监控',Waves],
  ['qr','二维码取物',QrCode],['tools','自主写点取物',PackageCheck],['robotctl','机器人控制',Gamepad2],['sampling','采样设备',FlaskConical],['diagnostics','故障诊断',Siren],['parameters','系统参数',FileSliders]
] as const

function useConnection() {
  const {sessionId,setSession,setSessionReady,setState,setConnected,setModel,setError}=useStation()
  const retry=useRef(0)
  useEffect(()=>{ api<RobotDescriptor>('/api/v1/model').then(setModel).catch(e=>setError(String(e))) },[setModel,setError])
  useEffect(()=>{
    let socket:WebSocket|undefined, stopped=false, heartbeat:number|undefined, reconnect:number|undefined
    const connect=()=>{
      const scheme=location.protocol==='https:'?'wss':'ws'
      socket=new WebSocket(`${scheme}://${location.host}/api/v1/ws`)
      socket.onopen=()=>{retry.current=0;setConnected(true); if(sessionId) socket?.send(JSON.stringify({type:'heartbeat',session_id:sessionId}))}
      socket.onmessage=e=>{const m=JSON.parse(e.data);if(m.type==='state')setState(m.data);if(m.type==='heartbeat_ack')setSessionReady(true);if(m.type==='error')setError(m.message)}
      socket.onclose=()=>{setConnected(false);setSessionReady(false);if(sessionId)setSession(null);if(!stopped){const delay=Math.min(5000,500*2**retry.current++);reconnect=window.setTimeout(connect,delay)}}
      socket.onerror=()=>socket?.close()
      heartbeat=window.setInterval(()=>{if(socket?.readyState===WebSocket.OPEN){socket.send(JSON.stringify(sessionId?{type:'heartbeat',session_id:sessionId}:{type:'ping'}))}},200)
    }
    connect(); return()=>{stopped=true;if(heartbeat)clearInterval(heartbeat);if(reconnect)clearTimeout(reconnect);socket?.close()}
  },[sessionId,setConnected,setError,setSession,setSessionReady,setState])
}

function Dashboard({state}:{state:StationState}) {
  const {sessionReady}=useStation()
  const run=async(name:string,action:string)=>api(`/api/v1/system/${name}/${action}`,{method:'POST'})
  const systems=[['base','RM75 基础栈','驱动 · MoveIt · TF'],['camera','D435i RGB','424×240 · 6 Hz'],['qr','Mark / QR','定位与任务服务']]
  const pathLabels:Record<string,string>={observe:'安全观察',joint_control:'七轴关节控制',cartesian_jog:'末端笛卡尔点动',z_force:'Z 力位 / Touch',pose_pickup:'写点取物 / 轨迹桥',qr_pickup:'QR / 轨迹桥',tool5_drill:'Tool5 钻取'}
  return <div className="page-grid dashboard-grid">
    <Panel title="系统态势" eyebrow="SYSTEM OVERVIEW" className="span-2"><div className="status-grid">
      <StatusPill label="ROS Master" ok={!!state.health.master}/><StatusPill label="RM75 控制器" ok={!!state.health.driver&&!!state.fresh.joints} detail="驱动节点在线且收到新鲜控制器反馈才视为已连接"/>
      <StatusPill label="七轴反馈" ok={!!state.fresh.joints}/><StatusPill label="MoveIt" ok={!!state.health.joint_control||!!state.subsystems.base?.running}/>
      <StatusPill label="D435i" ok={!!state.health.camera||!!state.fresh.image}/><StatusPill label="Geomagic Touch" ok={!!state.health.touch}/>
      <StatusPill label="六维力" ok={!!state.fresh.wrench}/><StatusPill label="Mark识别" ok={!!state.health.marker}/>
    </div></Panel>
    <Panel title="安全与控制" eyebrow="SAFETY"><div className="safety-hero"><StateBadge state={state.operator_state}/><Metric label="功能权限" value={state.execution.unified_control?'全部控制已启用':'受限'}/><Metric label="当前控制链路" value={pathLabels[state.active_control_path||'observe']}/><Metric label="运行控制源" value={state.control_launch_mode||state.control_mode||'IDLE'}/><Metric label="当前工具" value={state.selected_tool||'未选择'}/><Metric label="工具电压" value={state.tool_voltage??'—'} unit="V"/></div><p className="lock-note"><ShieldCheck size={16}/>无需选择授权路径；点击功能时自动切换对应的唯一控制链路，并继续执行命令源互斥、控制租约、心跳和故障停止检查。</p><div className="button-row power-buttons"><button disabled={!sessionReady} onClick={()=>command('set_voltage',{voltage:24})}>工具 24V</button><button disabled={!sessionReady} onClick={()=>command('set_voltage',{voltage:0})}>工具 0V</button></div></Panel>
    <Panel title="生命周期" eyebrow="SUPERVISED LAUNCH" className="span-2"><div className="lifecycle-list">{systems.map(([key,name,detail])=>{const s=state.subsystems[key];return <div className="lifecycle" key={key}><div className={`node-icon ${s?.running?'on':''}`}><Server size={19}/></div><div><b>{name}</b><span>{detail}</span></div><div className="lifecycle-state">{s?.running?'运行中':'已停止'}{s?.owned&&<small>本平台管理</small>}</div><button disabled={!sessionReady} onClick={()=>run(key,s?.running?'stop':'start')}>{s?.running?'停止':'启动'}</button></div>})}</div></Panel>
    <Panel title="最近系统事件" eyebrow="EVENT STREAM"><EventList state={state}/></Panel>
  </div>
}

function RobotPage({state,model}:{state:StationState;model:RobotDescriptor}) {
  const {target,setTarget,sessionReady,planner,speed,setPlanner,setSpeed}=useStation()
  const [mode,setMode]=useState<'joint'|'cart'>('joint')
  const actual=Array.from({length:7},(_,i)=>state.joints?.[`joint${i+1}`]||0)
  const exec=()=>api('/api/v1/joints/execute',{method:'POST',body:JSON.stringify({target})})
  return <div className="robot-layout"><div className="robot-stage"><RobotTwin descriptor={model} actual={actual} target={target} actualFresh={!!state.fresh.joints}/><div className="model-proof"><b>{model.model}</b><span>{model.link7_variant} · J7 Z 0.1612 m</span><small>厂商 xacro / STL 实时装载</small></div></div><aside><div className="aside-title"><div><span>{mode==='joint'?'JOINT CONTROL':'CARTESIAN CONTROL'}</span><h2>{mode==='joint'?'七轴离线目标':'末端位姿目标'}</h2></div><StateBadge state={state.operator_state}/></div><div className="control-tabs"><button className={mode==='joint'?'active':''} onClick={()=>setMode('joint')}>关节角</button><button className={mode==='cart'?'active':''} onClick={()=>setMode('cart')}>笛卡尔位姿</button></div><div className="planner-row"><select value={planner} onChange={e=>setPlanner(e.target.value as any)}><option value="ompl">自由规划 OMPL</option><option value="lin">直线规划 LIN</option><option value="ptp">关节规划 PTP</option><option value="direct">直接关节插补</option></select><select value={speed} onChange={e=>setSpeed(Number(e.target.value))}>{[5,20,30,50,100].map(v=><option key={v} value={v}>速度 {v}%</option>)}</select></div>{mode==='joint'?<JointPanel state={state} model={model} target={target} controlsReady={sessionReady} onTarget={setTarget} onExecute={exec} onCancel={()=>command('cancel_joints')}/>:<CartesianControl state={state}/>}</aside></div>
}

type CartesianPose={name?:string;x:number;y:number;z:number;rx:number;ry:number;rz:number}
function CartesianControl({state}:{state:StationState}) {
  const {setTarget,sessionReady,planner,speed}=useStation()
  const current=state.tcp||{}
  const [pose,setPose]=useState<CartesianPose>({x:Number(current.x)||0,y:Number(current.y)||0,z:Number(current.z)||0,rx:Number(current.rx)||0,ry:Number(current.ry)||0,rz:Number(current.rz)||0})
  const [presets,setPresets]=useState<CartesianPose[]>([]),[busy,setBusy]=useState(false),[notice,setNotice]=useState('')
  useEffect(()=>{api<{poses:CartesianPose[]}>('/api/v1/poses').then(r=>setPresets(r.poses)).catch(()=>{})},[])
  const update=(key:keyof CartesianPose,value:number)=>setPose({...pose,[key]:value})
  const preview=async(next=pose)=>{setBusy(true);try{const r=await api<{joints:number[]}>('/api/v1/cartesian/preview',{method:'POST',body:JSON.stringify(next)});setTarget(r.joints);setNotice('IK 预览完成：青色模型为目标，机械臂未运动')}finally{setBusy(false)}}
  const execute=async()=>{if(!sessionReady)throw new Error('请先取得控制权');setBusy(true);try{await api('/api/v1/cartesian/execute',{method:'POST',body:JSON.stringify({...pose,planner,speed})});setNotice('笛卡尔目标已通过现有七轴安全执行链')}finally{setBusy(false)}}
  const sync=()=>setPose({x:Number(current.x)||0,y:Number(current.y)||0,z:Number(current.z)||0,rx:Number(current.rx)||0,ry:Number(current.ry)||0,rz:Number(current.rz)||0})
  return <div className="cartesian-control"><p className="muted">Link7 在 base_link 下的位姿；位置为 m，姿态为 rad。预览只计算 IK，不运动。</p><div className="preset-list">{presets.map((item,i)=><button key={i} onClick={()=>{setPose(item);void preview(item)}}>{item.name||`姿态 ${i+1}`}</button>)}</div><div className="cartesian-grid">{(['x','y','z','rx','ry','rz'] as const).map(key=><label key={key}><span>{key.toUpperCase()}</span><input type="number" step={key.startsWith('r')?0.01:0.001} value={pose[key]} onChange={e=>update(key,Number(e.target.value))}/></label>)}</div><div className="joint-actions"><button onClick={sync}>同步实际位姿</button><button disabled={busy} onClick={()=>preview()}>IK 预览</button><button className="primary" disabled={busy||!sessionReady} onClick={execute}>执行末端目标</button></div>{notice&&<p className="tool-notice">{notice}</p>}</div>
}

type JogDirection='x_plus'|'x_minus'|'y_plus'|'y_minus'|'z_plus'|'z_minus'|'rx_plus'|'rx_minus'|'ry_plus'|'ry_minus'|'rz_plus'|'rz_minus'
const translationButtons:{direction:JogDirection;label:string;cell:string}[]=[
  {direction:'z_plus',label:'Z+',cell:'tl'},{direction:'x_plus',label:'X+',cell:'tc'},
  {direction:'z_minus',label:'Z−',cell:'tr'},{direction:'y_minus',label:'Y−',cell:'ml'},
  {direction:'y_plus',label:'Y+',cell:'mr'},{direction:'x_minus',label:'X−',cell:'bc'},
]
const rotationButtons:{direction:JogDirection;label:string;cell:string}[]=[
  {direction:'rx_minus',label:'X−',cell:'tl'},{direction:'ry_plus',label:'Y+',cell:'tc'},
  {direction:'rx_plus',label:'X+',cell:'tr'},{direction:'rz_minus',label:'Z−',cell:'ml'},
  {direction:'rz_plus',label:'Z+',cell:'mr'},{direction:'ry_minus',label:'Y−',cell:'bc'},
]

function CartesianJogPage({state}:{state:StationState}) {
  const {sessionReady,sessionId,connected,setError}=useStation()
  const jog=state.cartesian_jog
  const held=useRef<JogDirection|null>(null), request=useRef(0), stopping=useRef(false)
  const [localDirection,setLocalDirection]=useState<JogDirection|null>(null)
  const motionReady=sessionReady&&connected&&state.operator_state!=='SAFE_STOP'
  const sendStop=useCallback(async()=>{
    held.current=null;setLocalDirection(null)
    if(stopping.current)return
    stopping.current=true
    try{await api('/api/v1/cartesian/jog/stop',{method:'POST'})}
    catch(error){if(sessionId)setError(error instanceof Error?error.message:String(error))}
    finally{stopping.current=false}
  },[sessionId,setError])
  const start=useCallback(async(direction:JogDirection)=>{
    if(!motionReady||held.current)return
    held.current=direction;setLocalDirection(direction)
    const generation=++request.current
    try{
      await api('/api/v1/cartesian/jog/start',{method:'POST',body:JSON.stringify({direction})})
      if(held.current!==direction||generation!==request.current)await sendStop()
    }catch(error){held.current=null;setLocalDirection(null);setError(error instanceof Error?error.message:String(error))}
  },[motionReady,sendStop,setError])
  useEffect(()=>{
    const stop=()=>{if(held.current)void sendStop()}
    const hidden=()=>{if(document.visibilityState!=='visible')stop()}
    const unload=()=>{if(!held.current||!sessionId)return;fetch('/api/v1/cartesian/jog/stop',{method:'POST',keepalive:true,headers:{'X-RM75-Control-Session':sessionId}}).catch(()=>{})}
    window.addEventListener('blur',stop);window.addEventListener('pagehide',unload)
    document.addEventListener('visibilitychange',hidden)
    return()=>{window.removeEventListener('blur',stop);window.removeEventListener('pagehide',unload);document.removeEventListener('visibilitychange',hidden);unload()}
  },[sendStop,sessionId])
  useEffect(()=>{if((!sessionReady||!connected)&&held.current){held.current=null;setLocalDirection(null)}},[sessionReady,connected])
  const handlers=(direction:JogDirection)=>({
    onPointerDown:(event:React.PointerEvent<HTMLButtonElement>)=>{event.preventDefault();event.currentTarget.setPointerCapture(event.pointerId);void start(direction)},
    onPointerUp:(event:React.PointerEvent<HTMLButtonElement>)=>{event.preventDefault();void sendStop()},
    onPointerCancel:()=>void sendStop(),onLostPointerCapture:()=>{if(held.current)void sendStop()},
    onKeyDown:(event:React.KeyboardEvent<HTMLButtonElement>)=>{if((event.key===' '||event.key==='Enter')&&!event.repeat){event.preventDefault();void start(direction)}},
    onKeyUp:(event:React.KeyboardEvent<HTMLButtonElement>)=>{if(event.key===' '||event.key==='Enter'){event.preventDefault();void sendStop()}},
  })
  const renderPad=(items:typeof translationButtons,rotation=false)=><div className="jog-pad">
    <div className="jog-center"><b>{rotation?'Link7':'base_link'}</b><span>{rotation?'局部旋转':'直线平移'}</span></div>
    {items.map(item=><button key={item.direction} className={`jog-button ${item.cell} ${rotation?'rotation':''} ${item.direction.endsWith('minus')?'negative':''} ${(localDirection||jog?.direction)===item.direction?'active':''}`} disabled={!motionReady||(!!localDirection&&localDirection!==item.direction)||jog?.state==='STOPPING'} aria-label={`${rotation?'绕局部轴旋转':'沿基坐标轴平移'} ${item.label}`} {...handlers(item.direction)}>{rotation&&<RotateCw size={20}/>}<strong>{item.label}</strong></button>)}
  </div>
  const pose=state.tcp||{},target=jog?.target_pose||{}
  const mm=(value:unknown)=>Number.isFinite(Number(value))?(Number(value)*1000).toFixed(2):'—'
  const deg=(value:unknown)=>Number.isFinite(Number(value))?(Number(value)*180/Math.PI).toFixed(2):'—'
  return <div className="page-grid cartesian-jog-page">
    <Panel title="位置控制" eyebrow="BASE_LINK · HOLD TO JOG">{renderPad(translationButtons)}<p className="lock-note"><Move3D size={16}/>X/Y/Z 均沿 <b>base_link</b> 坐标轴直线平移。</p></Panel>
    <Panel title="姿态控制" eyebrow="LINK7 LOCAL AXES · HOLD TO JOG">{renderPad(rotationButtons,true)}<p className="lock-note"><RotateCw size={16}/>X/Y/Z 表示绕 <b>Link7 局部轴</b>旋转，不是平移。</p></Panel>
    <Panel title="Link7 实际位姿" eyebrow="NOT A CALIBRATED TOOL TCP" className="span-2"><div className="jog-pose-grid">{(['x','y','z'] as const).map(key=><Metric key={key} label={key.toUpperCase()} value={mm(pose[key])} unit="mm"/>)}{(['rx','ry','rz'] as const).map(key=><Metric key={key} label={key.toUpperCase()} value={deg(pose[key])} unit="°"/>)}</div><p className="safety-note"><ShieldCheck size={16}/>末端参考是厂商模型的 <b>Link7</b>；当前没有经过确认的工具 TCP 标定。</p></Panel>
    <Panel title="点动执行状态" eyebrow="GUARDED CARTESIAN EXECUTOR"><div className="jog-status"><Metric label="控制模式" value={jog?.state||'IDLE'} accent={jog?.state==='JOGGING'}/><Metric label="当前方向" value={jog?.direction||'—'}/><Metric label="进度" value={Math.round((jog?.progress||0)*100)} unit="%"/><Metric label="停止原因" value={jog?.stop_reason||'—'}/></div><div className="jog-target"><b>本次目标位姿</b><span>XYZ {mm(target.x)} / {mm(target.y)} / {mm(target.z)} mm</span><span>RPY {deg(target.rx)} / {deg(target.ry)} / {deg(target.rz)} °</span></div><p className="lock-note">后端硬限制：单次平移 ≤ {(jog?.limits?.translation_m||.01)*1000} mm，单次旋转 ≤ {((jog?.limits?.rotation_rad||Math.PI/36)*180/Math.PI).toFixed(1)}°。达到上限后必须先松开。</p></Panel>
  </div>
}

function RobotControlPage({state}:{state:StationState}) {
  const {sessionReady,planner,speed,setPlanner,setSpeed}=useStation()
  const [presets,setPresets]=useState<CartesianPose[]>([]),[history,setHistory]=useState<{t:number;fz:number}[]>([]),[busy,setBusy]=useState('')
  useEffect(()=>{api<{poses:CartesianPose[]}>('/api/v1/poses').then(r=>setPresets(r.poses)).catch(()=>{})},[])
  useEffect(()=>{const fz=Number(state.filtered_force_z??state.force_z??state.wrench?.Fz);if(Number.isFinite(fz))setHistory(v=>[...v.slice(-239),{t:state.timestamp,fz}])},[state.timestamp,state.filtered_force_z,state.force_z,state.wrench])
  const run=async(label:string,fn:()=>Promise<unknown>)=>{setBusy(label);try{await fn()}finally{setBusy('')}}
  const goto=(item:CartesianPose)=>run(item.name||'预设',()=>api('/api/v1/cartesian/execute',{method:'POST',body:JSON.stringify({...item,planner,speed})}))
  return <div className="page-grid robot-control-page"><Panel title="Touch 遥操作" eyebrow="EXISTING RM75 CONTROL"><div className="button-grid"><button className="primary" disabled={!sessionReady||!!busy||state.control_launch_mode==='teleop'} onClick={()=>run('启动遥操作',()=>command('start_teleop'))}>启动 Touch 遥操作</button><button disabled={!sessionReady||!!busy||state.control_launch_mode!=='teleop'} onClick={()=>run('停止遥操作',()=>command('stop_teleop'))}>停止遥操作</button></div><div className="recognition"><Metric label="控制模式" value={state.control_launch_mode||'IDLE'}/><Metric label="Touch" value={state.health.touch?'在线':'离线'}/><Metric label="状态机" value={state.operator_state}/></div></Panel><Panel title="规划器与速度" eyebrow="GLOBAL SETTINGS"><div className="settings-grid"><label>规划器<select value={planner} onChange={e=>setPlanner(e.target.value as any)}><option value="ompl">自由规划 OMPL</option><option value="lin">直线规划 LIN</option><option value="ptp">关节规划 PTP</option><option value="direct">直接插补</option></select></label><label>速度<select value={speed} onChange={e=>setSpeed(Number(e.target.value))}>{[5,20,30,50,100].map(v=><option key={v} value={v}>{v}%</option>)}</select></label></div><p className="lock-note"><Save size={15}/>设置写入 ui_settings.yaml，刷新后保留。</p></Panel><Panel title="一键姿态" eyebrow="CARTESIAN PRESETS" className="span-2"><div className="macro-grid">{presets.map((item,i)=><button key={i} disabled={!sessionReady||!!busy} onClick={()=>goto(item)}><b>{item.name||`姿态 ${i+1}`}</b><small>{item.x.toFixed(3)}, {item.y.toFixed(3)}, {item.z.toFixed(3)} m</small></button>)}</div><p className="safety-note"><ShieldCheck size={15}/>预设先求 IK，再进入 RealMan 已有的限位、反馈时效、控制源互斥和 MoveIt/FCL 检查。</p></Panel><Panel title="快捷安全控制" eyebrow="STOP · TOOL POWER"><div className="button-grid"><button disabled={!sessionReady} onClick={()=>command('cancel_joints')}>停止关节运动</button><button disabled={!sessionReady} onClick={()=>command('cancel_tool')}>终止工具任务</button><button disabled={!sessionReady} onClick={()=>command('set_voltage',{voltage:24})}>工具 24V</button><button disabled={!sessionReady} onClick={()=>command('set_voltage',{voltage:0})}>工具 0V</button></div></Panel><Panel title="实时 Fz" eyebrow="240 SAMPLES" className="span-2 chart-panel"><ResponsiveContainer width="100%" height={260}><LineChart data={history}><CartesianGrid stroke="#17303d"/><XAxis dataKey="t" tickFormatter={v=>new Date(v*1000).toLocaleTimeString()} stroke="#6c8694"/><YAxis unit=" N" stroke="#6c8694"/><Tooltip/><Line dataKey="fz" stroke="#26d8cc" dot={false} isAnimationActive={false}/></LineChart></ResponsiveContainer></Panel>{busy&&<div className="tool-notice span-3">{busy}执行中…</div>}</div>
}

function ForcePage({state}:{state:StationState}) {
  const {sessionReady}=useStation()
  const [paused,setPaused]=useState(false), [samples,setSamples]=useState<Record<string,number>[]>([])
  useEffect(()=>{if(paused)return;const w=state.wrench||{};setSamples(v=>[...v.slice(-299),{t:Date.now(),Fx:w.Fx||0,Fy:w.Fy||0,Fz:w.Fz||0,setpoint:state.force_setpoint||0}])},[state.sequence,paused,state.force_setpoint,state.wrench])
  const w=state.wrench||{}, actual=state.filtered_force_z??state.force_z??w.Fz
  const teleopActive=state.control_launch_mode==='teleop'
  const forceControlActive=state.control_launch_mode==='force_control'
  const anotherControlActive=!!state.control_launch_mode&&!teleopActive
  return <div className="page-grid force-grid"><Panel title="六维力实时值" eyebrow="WRENCH · SENSOR FRAME" className="span-2"><div className="force-metrics">{['Fx','Fy','Fz','Tx','Ty','Tz'].map(k=><Metric key={k} label={k.replace('T','M')} value={Number.isFinite(w[k])?w[k].toFixed(3):'—'} unit={k.startsWith('T')?'N·m':'N'} accent={k==='Fz'}/>)}</div></Panel><Panel title="Touch 普通遥操作" eyebrow="EXISTING RM75 TELEOP"><div className="force-loop"><Metric label="遥操作状态" value={teleopActive?'运行中':'已停止'} accent={teleopActive}/><ChevronRight/><Metric label="Touch" value={state.health.touch?'在线':'离线'}/><ChevronRight/><Metric label="实际 Fz" value={Number.isFinite(actual)?Number(actual).toFixed(2):'—'} unit="N"/></div><dl><dt>运动自由度</dt><dd>X/Y/Z 平移 + 姿态</dd><dt>控制方式</dt><dd>普通位置遥操作</dd><dt>控制模式</dt><dd>{state.control_mode||'—'}</dd><dt>停止原因</dt><dd>{state.stop_reason||'无'}</dd><dt>力反馈新鲜度</dt><dd>{state.fresh.wrench?`${((state.ages.wrench||0)*1000).toFixed(0)} ms`:'过期 / 不可用'}</dd></dl><div className="button-grid"><button disabled={!sessionReady||anotherControlActive} className={teleopActive?'secondary':'primary'} onClick={()=>command(teleopActive?'stop_teleop':'start_teleop')}>{teleopActive?'停止普通遥操作':'启动普通遥操作'}</button><button disabled={!sessionReady||(!!state.control_launch_mode&&!forceControlActive)} onClick={()=>command(forceControlActive?'stop_sampling':'enter_force_control')}>{forceControlActive?'停止独立力控':'独立垂直下压力控'}</button></div><p className="safety-note"><ShieldCheck size={16}/>灰键控制平移，白键控制姿态；普通遥操作不启用 Z 力位闭环或 Touch 力反馈，六维力仅用于安全保护。紧急情况使用顶部软件急停。</p></Panel><Panel title="Fz 滚动曲线" eyebrow="300 SAMPLES" className="span-2 chart-panel"><button className="chart-pause" onClick={()=>setPaused(!paused)}>{paused?'继续显示':'暂停显示'}</button><ResponsiveContainer width="100%" height={360}><LineChart data={samples}><CartesianGrid stroke="#17303d"/><XAxis dataKey="t" tickFormatter={v=>new Date(v).toLocaleTimeString()} stroke="#6c8694"/><YAxis unit=" N" stroke="#6c8694"/><Tooltip labelFormatter={v=>new Date(Number(v)).toLocaleTimeString()}/><Legend/><Line dataKey="Fz" stroke="#26d8cc" dot={false} isAnimationActive={false}/><Line dataKey="setpoint" stroke="#f3a451" dot={false} strokeDasharray="5 5" isAnimationActive={false}/></LineChart></ResponsiveContainer></Panel></div>
}

function VisionPage({state}:{state:StationState}) {
  const {sessionReady}=useStation()
  const [tool,setTool]=useState(state.selected_tool||'Tool1'),[busy,setBusy]=useState(false),[notice,setNotice]=useState('')
  const [confirmation,setConfirmation]=useState<{kind:'execute'|'return';detail:string}|null>(null)
  const seq=(state as any).sequences?.image||0
  const slot=Number(tool.replace('Tool',''))
  const selectedTarget=(((state.five_targets as any)?.targets||[]) as any[]).find(item=>Number(item.slot_id)===slot)
  const run=async(name:string)=>{if(!sessionReady)throw new Error('请先取得控制权并等待心跳就绪');setBusy(true);try{await command('select_tool',{tool});const result=await command(name);setNotice(`${tool}：${name==='detect_marker'?'二维码识别完成':name==='compute_targets'?'五目标计算完成':name==='preview_tool'?'目标预览完成':name==='plan_tool'?'MoveIt Dry Run规划通过':name==='return_tool'?'归还流程完成':'命令完成'}`);return result}finally{setBusy(false)}}
  const execute=async()=>{await run('plan_tool');const pose=selectedTarget?.link7||selectedTarget?.tcp;setConfirmation({kind:'execute',detail:`${tool} 二维码取物规划已通过。\n\n目标来自公共二维码 ID 0 与已标定槽位变换。\n${pose?JSON.stringify(pose,null,2):'目标详见上位机实时目标状态'}\n\n确认执行现有二维码自动取工具流程？`})}
  const returnHeld=()=>setConfirmation({kind:'return',detail:`确认执行 ${tool} 的二维码引导归还流程？`})
  const confirmAction=async()=>{const pending=confirmation;if(!pending)return;setConfirmation(null);setBusy(true);try{await command('select_tool',{tool});await command(pending.kind==='execute'?'execute_tool':'return_tool');setNotice(pending.kind==='execute'?`${tool} 二维码取物运动完成；是否真实持物以现有反馈能力为准`:`${tool}：归还流程完成`)}finally{setBusy(false)}}
  const qrStatus=state.five_target_status||{}
  return <div className="vision-layout qr-layout"><Panel title="二维码视觉与定位" eyebrow={`${state.image_topic||'/simulation/camera/color/image_raw'} · APRILTAG ID 0`} className="video-panel"><div className="video-frame">{state.fresh.image?<img src={`/api/v1/vision/frame.jpg?s=${seq}`} alt="二维码定位实时画面"/>:<div className="video-empty"><Camera size={52}/><b>等待 RGB 帧</b><span>二维码功能使用现有相机、标定和稳定性检查</span></div>}<div className="video-overlay"><span>{state.health.camera?'CAMERA ONLINE':'CAMERA OFFLINE'}</span><span>{state.fresh.image?'LIVE':'NO SIGNAL'}</span></div></div><div className="qr-mode-note"><QrCode/><div><b>二维码定位取物</b><span>目标来源：公共二维码 ID 0 + 已标定 Tool1–Tool5 槽位变换；不读取“自主写点”配置。</span></div></div></Panel><aside><Panel title="二维码取物控制" eyebrow="INDEPENDENT QR WORKFLOW"><div className="tool-picker">{[1,2,3,4,5].map(i=><button key={i} className={tool===`Tool${i}`?'active':''} onClick={()=>setTool(`Tool${i}`)}><b>Tool{i}</b><small>QR 槽位 {i}</small></button>)}</div><button className="primary full" disabled={busy||!sessionReady} onClick={()=>command('select_tool',{tool})}>选择 {tool}</button><div className="recognition"><Metric label="二维码观测" value={(qrStatus as any).vision?.ok||state.vision_status?.ok?'有效':'等待'}/><Metric label="目标状态" value={(qrStatus as any).targets_valid?'已计算':'未计算'}/><Metric label="执行就绪" value={(qrStatus as any).execution_ready?'是':'否'}/><Metric label="持有工具" value={(qrStatus as any).holding_slot?`Tool${(qrStatus as any).holding_slot}`:'无'}/></div><div className="qr-command-grid"><button disabled={busy||!sessionReady} onClick={()=>run('detect_marker')}>1. 识别二维码</button><button disabled={busy||!sessionReady} onClick={()=>run('compute_targets')}>2. 计算五目标</button><button disabled={busy||!sessionReady} onClick={()=>run('preview_tool')}>3. 预览选中目标</button><button disabled={busy||!sessionReady} onClick={()=>run('plan_tool')}>4. 仅规划 Dry Run</button><button className="primary" disabled={busy||!sessionReady} onClick={execute}>确认并二维码取物</button><button disabled={busy||!sessionReady} onClick={returnHeld}>二维码引导归还</button><button className="danger-button" disabled={!sessionReady} onClick={()=>command('cancel_tool')}>终止二维码任务</button></div>{notice&&<div className="tool-notice">{notice}</div>}<p className="lock-note"><ShieldCheck size={16}/>二维码执行仍受标定、目标新鲜度、MoveIt碰撞检查、执行授权、命令源互斥、急停和ACK约束。</p></Panel><Panel title="二维码目标与任务状态" eyebrow="QR TASK STATUS"><PoseTable value={(selectedTarget||state.marker_0||{}) as Dict}/><pre className="qr-status-json">{JSON.stringify(qrStatus,null,2)}</pre></Panel></aside>{confirmation&&<ConfirmDialog title={confirmation.kind==='execute'?'确认二维码取物':'确认二维码归还'} detail={confirmation.detail} onCancel={()=>{setConfirmation(null);setNotice('用户取消：机械臂未运动')}} onConfirm={confirmAction}/>}</div>
}

const emptyTool=(slot:number):ToolPickConfig=>({slot_id:slot,name:`Tool${slot}`,x:0,y:0,z:0,rx:0,ry:0,rz:0,lift_height:.2,position_unit:'m',angle_unit:'rad',pose_reference:'Link7',frame:'base_link'})

function ToolsPage({state}:{state:StationState}) {
  const {sessionReady}=useStation()
  const [tools,setTools]=useState<ToolPickConfig[]>(Array.from({length:5},(_,i)=>emptyTool(i+1)))
  const [slot,setSlot]=useState(1),[loading,setLoading]=useState(true),[busy,setBusy]=useState(false),[notice,setNotice]=useState('')
  const [confirmation,setConfirmation]=useState<{slot:number;detail:string}|null>(null)
  const load=useCallback(()=>{setLoading(true);return api<{tools:ToolPickConfig[]}>('/api/v1/tools').then(r=>setTools(r.tools)).finally(()=>setLoading(false))},[])
  useEffect(()=>{load()},[load])
  const selected=tools.find(t=>t.slot_id===slot)||emptyTool(slot)
  const edit=(key:keyof ToolPickConfig,value:number)=>setTools(items=>items.map(item=>item.slot_id===slot?{...item,[key]:value}:item))
  const isLinear=(key:keyof ToolPickConfig)=>key==='x'||key==='y'||key==='z'
  const displayValue=(key:keyof ToolPickConfig)=>isLinear(key)?Number(selected[key])*1000:Number(selected[key])
  const editDisplay=(key:keyof ToolPickConfig,value:number)=>edit(key,isLinear(key)?value/1000:value)
  const payload=()=>({x:selected.x,y:selected.y,z:selected.z,rx:selected.rx,ry:selected.ry,rz:selected.rz,lift_height:selected.lift_height})
  const save=async()=>{if(!sessionReady)throw new Error('请先取得控制权并等待心跳就绪');setBusy(true);try{const saved=await api<ToolPickConfig>(`/api/v1/tools/${slot}`,{method:'PUT',body:JSON.stringify(payload())});setTools(items=>items.map(item=>item.slot_id===slot?saved:item));setNotice(`Tool${slot} 配置已持久化`);return saved}finally{setBusy(false)}}
  const dryRun=async()=>{if(!sessionReady)throw new Error('请先取得控制权并等待心跳就绪');setBusy(true);try{await api<ToolPickConfig>(`/api/v1/tools/${slot}`,{method:'PUT',body:JSON.stringify(payload())});const plan=await api<any>(`/api/v1/tools/${slot}/dry-run`,{method:'POST'});setNotice(`Tool${slot} Dry Run 通过：下降 ${(Number(plan.descent_cartesian_fraction??1)*100).toFixed(1)}%，抬升 ${(Number(plan.lift_cartesian_fraction??1)*100).toFixed(1)}%；机械臂未运动`)}finally{setBusy(false);load()}}
  const execute=async()=>{if(!sessionReady)throw new Error('请先取得控制权并等待心跳就绪');setBusy(true);try{await api<ToolPickConfig>(`/api/v1/tools/${slot}`,{method:'PUT',body:JSON.stringify(payload())});const plan=await api<any>(`/api/v1/tools/${slot}/dry-run`,{method:'POST'});const c=plan.configuration||selected;setConfirmation({slot,detail:`Tool${slot} Dry Run 已通过。\n\n目标 Link7（base_link）\nX ${Number(c.position_m?.[0]??c.x).toFixed(6)} m\nY ${Number(c.position_m?.[1]??c.y).toFixed(6)} m\nZ ${Number(c.position_m?.[2]??c.z).toFixed(6)} m\nLift ${Number(c.lift_height_m??c.lift_height).toFixed(4)} m\n\n将低速执行预抓取、严格Z下降、24V动作和严格Z抬升。是否确认执行？`})}finally{setBusy(false);load()}}
  const confirmExecute=async()=>{const pending=confirmation;if(!pending)return;setConfirmation(null);setBusy(true);try{const result=await api<any>(`/api/v1/tools/${pending.slot}/execute`,{method:'POST'});setNotice(result.grasp_confirmed?'执行完成并确认取物':'运动完成；24V ACK有效，但取物结果没有独立传感器确认')}finally{setBusy(false);load()}}
  const tcp=state.tcp||{}, status=state.configured_pick_status||{}
  const fields:[keyof ToolPickConfig,string][]=[['x','X'],['y','Y'],['z','Z'],['rx','Rx'],['ry','Ry'],['rz','Rz'],['lift_height','Lift Height']]
  const running=['PLANNING','RUNNING'].includes(status.status||'')
  return <div className="page-grid tool-config-page">
    <Panel title="五工具配置" eyebrow="ABSOLUTE LINK7 POSE · BASE_LINK" className="span-2">
      <div className="tool-tabs">{tools.map(tool=><button key={tool.slot_id} className={slot===tool.slot_id?'active':''} onClick={()=>setSlot(tool.slot_id)}>工具 {tool.slot_id}<small>{tool.updated_at?`已保存 ${new Date(tool.updated_at*1000).toLocaleString()}`:'项目默认值'}</small></button>)}</div>
      {loading?<div className="loading compact"><RefreshCw className="spin"/>读取配置…</div>:<div className="pose-form">{fields.map(([key,label])=><label key={key}><span>{label}</span><input type="number" step={isLinear(key)?.001:key==='lift_height'?.001:.000001} value={displayValue(key)} onChange={e=>editDisplay(key,Number(e.target.value))}/><b>{isLinear(key)?'mm':key==='lift_height'?'m':'rad'}</b></label>)}</div>}
      <div className="tool-actions"><button disabled={busy||running||!sessionReady} onClick={()=>save()}><Save/>保存配置</button><button disabled={busy||running||!sessionReady} onClick={()=>dryRun()}><ShieldCheck/>仅 Dry Run</button><button className="primary" disabled={busy||running||!sessionReady} onClick={()=>execute()}><PackageCheck/>{busy?'检查中…':'确定并执行'}</button><button className="danger-button" disabled={!sessionReady||!running} onClick={()=>command('cancel_tool')}>终止执行</button></div>
      <p className="safety-note"><ShieldCheck size={16}/>X/Y/Z 按毫米输入并在保存时转换为米；姿态单位为 rad，抬升高度单位为 m。执行前必须通过 MoveIt 可达性、限位、自碰撞和完整严格 Z 向路径 Dry Run。</p>
      {notice&&<div className="tool-notice">{notice}</div>}
    </Panel>
    <Panel title="实时位姿" eyebrow="BASE_LINK → LINK7"><div className="live-pose">{['x','y','z','rx','ry','rz'].map(key=><Metric key={key} label={key.toUpperCase()} value={Number.isFinite(tcp[key])?Number(tcp[key]).toFixed(5):'—'} unit={['x','y','z'].includes(key)?'m':'rad'}/>)}</div><p className="lock-note"><ShieldCheck size={16}/>{state.fresh.tcp?'位姿反馈新鲜':'位姿反馈不可用或已过期，执行将被拒绝'}</p></Panel>
    <Panel title="当前执行步骤" eyebrow="FAIL-CLOSED SEQUENCE"><div className={`pick-state ${status.status==='FAILED'?'failed':status.status==='COMPLETED_UNVERIFIED'?'warning':''}`}><b>{status.status||'IDLE'}</b><strong>步骤 {status.step??0} / 6</strong><span>{status.step_name||'等待执行'}</span>{status.error&&<small>{status.error}</small>}</div><ol className="step-list"><li>运动到 Z+H 预抓取位姿</li><li>保持姿态，沿基坐标 Z 下降</li><li>调用现有 24V 末端动作</li><li>核验动作 ACK / 取物确认能力</li><li>保持姿态，沿基坐标 Z 抬升 H</li><li>保持当前位置等待</li></ol>{status.status==='COMPLETED_UNVERIFIED'&&<p className="unverified">运动成功；没有独立物体在位传感器，不能宣称“抓取成功”。</p>}</Panel>
    <Panel title="当前工具配置" eyebrow={`TOOL ${slot}`} className="span-2"><pre className="config-preview">{JSON.stringify({frame:selected.frame,pose_reference:selected.pose_reference,position_m:[selected.x,selected.y,selected.z],rpy_rad:[selected.rx,selected.ry,selected.rz],lift_height_m:selected.lift_height},null,2)}</pre></Panel>
    {confirmation&&<ConfirmDialog title="确认自主写点取物" detail={confirmation.detail} onCancel={()=>{setConfirmation(null);setNotice('用户取消：只完成 Dry Run，机械臂未运动')}} onConfirm={confirmExecute}/>} 
  </div>
}

function ConfirmDialog({title,detail,onCancel,onConfirm,cancelLabel='取消，仅保留 Dry Run',confirmLabel='确认执行'}:{title:string;detail:string;onCancel:()=>void;onConfirm:()=>void;cancelLabel?:string;confirmLabel?:string}) {
  return <div className="confirm-backdrop" role="presentation"><section className="confirm-dialog" role="dialog" aria-modal="true" aria-labelledby="confirm-title"><span>SAFETY CONFIRMATION</span><h2 id="confirm-title">{title}</h2><pre>{detail}</pre><p><ShieldCheck size={16}/>该确认层不会阻塞控制心跳；确认前不会发送执行命令。</p><div><button onClick={onCancel}>{cancelLabel}</button><button className="primary" onClick={onConfirm}>{confirmLabel}</button></div></section></div>
}

function SamplingPage({state}:{state:StationState}) {
  const {sessionReady}=useStation()
  const hw=state.sampling_hardware
  const sensors=hw?.sensors
  const motor=hw?.motor
  const [speed,setSpeed]=useState(0.5)
  const [duration,setDuration]=useState(3)
  const motorRun=(action:string)=>command('motor',{action,speed,duration})
  const number=(value:number|null|undefined,digits=2)=>Number.isFinite(value)?Number(value).toFixed(digits):'—'
  const controlsReady=sessionReady&&(hw?.authorized||state.simulation)
  const motionReady=controlsReady&&state.operator_state!=='SAFE_STOP'
  const drill=state.tool5_drill
  const drillActive=['STARTING','ROTATING','DRILLING','DEPTH_REACHED','STOPPING_MOTOR'].includes(drill?.state||'')
  const deviceHealth=sensors?.health||{}
  const busHealthy=Object.keys(deviceHealth).length===0||Object.values(deviceHealth).every(value=>value==='ok')
  return <div className="page-grid sampling-grid">
    <Panel title="三路水质传感器" eyebrow="RS-485 · 9600 8N1" className="span-2">
      <div className="sensor-metrics">
        <Metric label="电导率 EC" value={number(sensors?.ec,0)} unit="µS/cm" accent/>
        <Metric label="EC 温度" value={number(sensors?.ec_temp,1)} unit="°C"/>
        <Metric label="ORP" value={number(sensors?.orp,0)} unit="mV"/>
        <Metric label="pH" value={number(sensors?.ph,2)} unit="pH" accent/>
      </div>
      <div className="sensor-health">{['EC','ORP','pH','继电器'].map(name=><span key={name} className={deviceHealth[name]==='ok'?'ok':'bad'} title={deviceHealth[name]||'等待探测'}>{name}<b>{deviceHealth[name]==='ok'?'在线':'异常'}</b></span>)}</div>
      <div className={`hardware-line ${sensors?.online?'online':'offline'}`}><Radio/><b>{!sensors?.online?'RS-485 离线':busHealthy?'RS-485 全部在线':'RS-485 部分在线'}</b><span>{sensors?.status||'等待状态'}</span><time>{sensors?.updated?new Date(sensors.updated*1000).toLocaleTimeString():'—'}</time></div>
    </Panel>
    <Panel title="水泵" eyebrow="RELAY CHANNEL 4">
      <div className={`pump-state ${sensors?.pump_on?'on':''}`}><Power/><span>继电器第 {hw?.pump_channel||4} 路</span><b>{sensors?.pump_on?'水泵运行中':'水泵已停止'}</b><small>当前位图 0x{Number(sensors?.relay_state||0).toString(16).padStart(2,'0').toUpperCase()}</small></div>
      <div className="button-row pump-buttons"><button className="primary" disabled={!motionReady||!sensors?.relay_online||!!sensors?.pump_on} onClick={()=>command('pump_on')}>开启水泵</button><button className="danger-button" disabled={!controlsReady||!sensors?.relay_online||!sensors?.pump_on} onClick={()=>command('pump_off')}>关闭水泵</button></div>
      <p className="lock-note"><ShieldCheck size={16}/>只改继电器第 4 路，保留其余七路状态；退出平台时会尝试关闭水泵。</p>
    </Panel>
    <Panel title="DM-J4310 电机" eyebrow="CAN ID 0x01 · VELOCITY MODE" className="span-2">
      <div className="motor-status"><StatusPill label="电机通信" ok={!!motor?.online}/><Metric label="位置" value={number(motor?.position,3)} unit="rad"/><Metric label="速度" value={number(motor?.velocity,3)} unit="rad/s"/><Metric label="力矩" value={number(motor?.torque,3)} unit="N·m"/></div>
      <div className="sensor-health"><span className="ok">直接控制 <b>无需预先配置</b></span><span className="ok">速度保护 <b>≤ {number(motor?.max_speed,1)} rad/s</b></span><span className="ok">电流保护 <b>≤ {number(motor?.max_current,1)} A</b></span><span className="ok">力矩保护 <b>≤ {number(motor?.max_torque,1)} N·m</b></span></div>
      <pre className="hardware-output">{motor?.status||'尚未读取电机状态'}</pre>
    </Panel>
    <Panel title="电机受控动作" eyebrow="BOUNDED MOTION">
      <label className="hardware-field"><span>速度上限</span><input type="number" min="0.05" max="1" step="0.05" value={speed} onChange={e=>setSpeed(Math.min(1,Math.max(.05,Number(e.target.value)||.05)))}/><b>rad/s</b></label>
      <label className="hardware-field"><span>运行时长</span><input type="number" min="0.1" max="60" step="0.1" value={duration} onChange={e=>setDuration(Math.min(60,Math.max(.1,Number(e.target.value)||.1)))}/><b>s</b></label>
      <div className="motor-buttons"><button disabled={!motionReady||motor?.busy} onClick={()=>motorRun('forward')}>正转</button><button disabled={!motionReady||motor?.busy} onClick={()=>motorRun('reverse')}>反转</button><button className="primary" disabled={!motionReady||motor?.busy} onClick={()=>motorRun('dig')}>采样钻进</button><button className="danger-button" disabled={!controlsReady} onClick={()=>motorRun('stop')}>停止并失能</button><button disabled={!controlsReady||motor?.busy} onClick={()=>motorRun('status')}>读取状态</button></div>
      <p className="lock-note"><ShieldCheck size={16}/>普通动作单次最长 60 s；Tool5 “开始取土”无固定时长，持续到点击“停止旋转”或安全保护触发。VEL 模式及硬上限由底层自动应用。</p>
    </Panel>
    <Panel title="Tool5 螺旋电机连续旋转" eyebrow="MOTOR ONLY · INDEPENDENT Z FORCE" className="span-3">
      <div className="drill-layout"><div>
        <div className="motor-buttons"><button className="primary" disabled={!motionReady||drillActive} onClick={()=>command('start_tool5_rotation')}>开始连续旋转</button><button className="danger-button" disabled={!controlsReady||!drillActive} onClick={()=>command('stop_tool5_rotation')}>停止旋转</button></div>
        <p className="lock-note"><ShieldCheck size={16}/>这里仅控制螺旋电机，不会命令机械臂下降。电机旋转期间可进入“实时力监控”，启动“独立垂直下压力控”；急停仍会同时停止电机和机械臂。</p>
      </div><div className="drill-status-grid">
        <Metric label="旋转状态" value={drill?.state||'IDLE'} accent={drill?.state==='ROTATING'}/><Metric label="电机" value={drill?.motor_running?'旋转中':'已停止'}/><Metric label="机械臂 Z" value="由实时力监控独立控制"/><Metric label="轴向力 Fz" value={number(state.filtered_force_z??state.force_z??state.wrench?.Fz,2)} unit="N"/>
      </div></div>
      <div className={`tool-notice ${drill?.state==='SAFE_STOP'?'critical':''}`}>{drill?.fault_reason||`正常连接时电机持续旋转，低速/堵转不会因土壤阻力自动停机；点击“停止旋转”才停止。${number(drill?.motor_watchdog_timeout_s,1)} s 通信看门狗、超速和硬限仍保留。`}</div>
    </Panel>
    {!controlsReady&&<div className="hardware-lock span-3"><ShieldCheck/><div><b>当前仅可监视</b><span>{hw?.authorized?'请先取得控制权并等待心跳就绪。':'请使用统一启动脚本启动完整控制平台。'}</span></div></div>}
  </div>
}

const idleTrajectoryExecution:TrajectoryExecution={state:'IDLE',trajectory_id:null,message:'待机',current_point:0,total_points:0,current_waypoint:0,total_waypoints:0,progress:0}

function TrajectoryPage({state}:{state:StationState}) {
  const {sessionReady,setError}=useStation()
  const [library,setLibrary]=useState<TrajectorySummary[]>([])
  const [document,setDocument]=useState<RM75Trajectory|null>(null)
  const [loadId,setLoadId]=useState('')
  const [selected,setSelected]=useState('')
  const [name,setName]=useState('采样撤离路径01')
  const [speed,setSpeed]=useState(.2)
  const [acceleration,setAcceleration]=useState(.2)
  const [busy,setBusy]=useState('')
  const [notice,setNotice]=useState('')
  const execution=state.trajectory_execution||idleTrajectoryExecution
  const refreshLibrary=useCallback(async()=>{const value=await api<{trajectories:TrajectorySummary[]}>('/api/v1/trajectories');setLibrary(value.trajectories);if(!loadId&&value.trajectories[0])setLoadId(value.trajectories[0].trajectory_id)},[loadId])
  const applyDocument=useCallback((value:RM75Trajectory)=>{setDocument(value);setLoadId(value.trajectory_id);setName(value.name);setSpeed(value.trajectory_config.speed_scale);setAcceleration(value.trajectory_config.acceleration_scale);setSelected(value.waypoints.some(item=>item.waypoint_id===selected)?selected:'')},[selected])
  useEffect(()=>{void refreshLibrary().catch(error=>setError(String(error)))},[refreshLibrary,setError])
  useEffect(()=>{if(document&&execution.trajectory_id===document.trajectory_id&&execution.state!=='EXECUTING'){api<RM75Trajectory>(`/api/v1/trajectories/${document.trajectory_id}`).then(applyDocument).catch(()=>{})}},[execution.state,execution.trajectory_id,document?.trajectory_id])
  const perform=async(label:string,operation:()=>Promise<void>)=>{setBusy(label);setNotice('');try{await operation()}catch(error){setError(error instanceof Error?error.message:String(error))}finally{setBusy('')}}
  const create=async()=>{const value=await api<RM75Trajectory>('/api/v1/trajectories',{method:'POST',body:JSON.stringify({name})});applyDocument(value);await refreshLibrary();return value}
  const ensure=async()=>document||await create()
  const savePath=()=>perform('save',async()=>{if(!document){await create();setNotice('路径已创建并保存');return}const value=await api<RM75Trajectory>(`/api/v1/trajectories/${document.trajectory_id}`,{method:'PUT',body:JSON.stringify({name,trajectory_config:{speed_scale:speed,acceleration_scale:acceleration}})});applyDocument(value);await refreshLibrary();setNotice('路径名称与轨迹参数已原子保存')})
  const loadPath=()=>perform('load',async()=>{if(!loadId)throw new Error('请选择要加载的路径');applyDocument(await api<RM75Trajectory>(`/api/v1/trajectories/${loadId}`));setNotice('路径已从 JSON 加载')})
  const deletePath=()=>perform('delete-path',async()=>{if(!document)return;if(!window.confirm(`删除路径“${document.name}”？该 JSON 文件将被删除。`))return;await api(`/api/v1/trajectories/${document.trajectory_id}`,{method:'DELETE'});setDocument(null);setSelected('');setLoadId('');await refreshLibrary();setNotice('路径已删除')})
  const record=()=>perform('record',async()=>{const current=await ensure();const value=await api<RM75Trajectory>(`/api/v1/trajectories/${current.trajectory_id}/waypoints`,{method:'POST'});applyDocument(value);await refreshLibrary();setNotice(`已记录 P${value.waypoints.length}：真实七轴与 Link7 TF`)})
  const removeSelected=()=>perform('delete-point',async()=>{if(!document||!selected)throw new Error('请先选择点位');const value=await api<RM75Trajectory>(`/api/v1/trajectories/${document.trajectory_id}/waypoints/${selected}`,{method:'DELETE'});applyDocument(value);setSelected('');setNotice('选中点已删除，旧轨迹已失效')})
  const move=(direction:'up'|'down')=>perform(direction,async()=>{if(!document||!selected)throw new Error('请先选择点位');const value=await api<RM75Trajectory>(`/api/v1/trajectories/${document.trajectory_id}/waypoints/${selected}/move`,{method:'POST',body:JSON.stringify({direction})});applyDocument(value);setNotice('点位顺序已更新，旧轨迹已失效')})
  const clear=()=>perform('clear',async()=>{if(!document)return;if(!window.confirm('清空当前路径的全部示教点？'))return;applyDocument(await api<RM75Trajectory>(`/api/v1/trajectories/${document.trajectory_id}/waypoints`,{method:'DELETE'}));setSelected('');setNotice('当前路径已清空')})
  const generate=()=>perform('generate',async()=>{const current=await ensure();const updated=await api<RM75Trajectory>(`/api/v1/trajectories/${current.trajectory_id}`,{method:'PUT',body:JSON.stringify({name,trajectory_config:{speed_scale:speed,acceleration_scale:acceleration}})});const value=await api<RM75Trajectory>(`/api/v1/trajectories/${updated.trajectory_id}/generate`,{method:'POST'});applyDocument(value);await refreshLibrary();setNotice(`已生成 ${value.trajectory.length} 个密集点，并通过限位/碰撞检查`)})
  const execute=()=>perform('execute',async()=>{if(!document)throw new Error('请先加载路径');await api(`/api/v1/trajectories/${document.trajectory_id}/execute`,{method:'POST'});setNotice('轨迹已进入既有七轴受控执行链')})
  const stop=()=>perform('stop',async()=>{await api('/api/v1/trajectories/execution/stop',{method:'POST'});setNotice('已请求停止，等待控制器确认')})
  const selectedIndex=document?.waypoints.findIndex(item=>item.waypoint_id===selected)??-1
  const parametersChanged=!!document&&(speed!==document.trajectory_config.speed_scale||acceleration!==document.trajectory_config.acceleration_scale)
  const ready=!parametersChanged&&!!document&&['READY','COMPLETED','CANCELLED'].includes(document.trajectory_status.state)
  const percent=Math.round((execution.progress||0)*100)
  return <div className="page-grid trajectory-page">
    <Panel title="RM75 路径示教" eyebrow="MANUAL WAYPOINT TEACHING" className="span-2">
      <div className="trajectory-title"><label><span>路径名称</span><input value={name} maxLength={120} onChange={event=>setName(event.target.value)}/></label><Metric label="已记录点数" value={document?.waypoints.length||0} accent/><Metric label="密集轨迹点" value={document?.trajectory.length||0}/></div>
      <div className="waypoint-table"><div className="waypoint-head"><span>点位</span><span>q1～q7 / rad</span><span>Link7 XYZ / m</span><span>ROS 时间</span></div>{document?.waypoints.length?document.waypoints.map(point=><button key={point.waypoint_id} className={selected===point.waypoint_id?'selected':''} onClick={()=>setSelected(point.waypoint_id)}><b>P{point.sequence}</b><code>{point.joint_positions.map(value=>value.toFixed(4)).join('  ')}</code><code>{(['x','y','z'] as const).map(key=>point.link7_pose.position[key].toFixed(4)).join('  ')}</code><time>{point.ros_timestamp.toFixed(3)}</time></button>):<div className="waypoint-empty"><Waypoints/>尚未记录点位；移动机械臂后点击“记录当前点”</div>}</div>
      <div className="trajectory-actions"><button className="primary" disabled={!sessionReady||!!busy} onClick={record}><Plus/>记录当前点</button><button disabled={!sessionReady||!!busy||!selected} onClick={removeSelected}><Trash2/>删除选中点</button><button disabled={!sessionReady||!!busy||selectedIndex<=0} onClick={()=>move('up')}><ArrowUp/>上移</button><button disabled={!sessionReady||!!busy||selectedIndex<0||selectedIndex===(document?.waypoints.length||0)-1} onClick={()=>move('down')}><ArrowDown/>下移</button><button className="danger-button" disabled={!sessionReady||!!busy||!document?.waypoints.length} onClick={clear}>清空路径</button></div>
    </Panel>
    <Panel title="轨迹参数" eyebrow="C2 QUINTIC · STRICT WAYPOINTS">
      <label className="trajectory-range"><span>速度比例 <b>{Math.round(speed*100)}%</b></span><input type="range" min="0.05" max="1" step="0.05" value={speed} onChange={event=>setSpeed(Number(event.target.value))}/></label>
      <label className="trajectory-range"><span>加速度比例 <b>{Math.round(acceleration*100)}%</b></span><input type="range" min="0.05" max="1" step="0.05" value={acceleration} onChange={event=>setAcceleration(Number(event.target.value))}/></label>
      <button className="primary full" disabled={!sessionReady||!!busy||(document?.waypoints.length||0)<2} onClick={generate}>{busy==='generate'?<RefreshCw className="spin"/>:<Waypoints/>}生成平滑轨迹</button>
      <div className={`trajectory-state ${document?.trajectory_status.state.toLowerCase()||'empty'}`}><b>{document?.trajectory_status.state||'EMPTY'}</b><span>{parametersChanged?'参数已修改，请重新生成后执行':document?.trajectory_status.message||'创建路径后开始示教'}</span><small>采样周期 {document?.trajectory_config.sample_period_s||.02} s · 分段五次 C2</small></div>
    </Panel>
    <Panel title="轨迹管理" eyebrow="ATOMIC JSON RESOURCE" className="span-2">
      <div className="trajectory-library"><select value={loadId} onChange={event=>setLoadId(event.target.value)}><option value="">选择已保存路径</option>{library.map(item=><option key={item.trajectory_id} value={item.trajectory_id}>{item.name} · {item.waypoint_count}点 · {item.state}</option>)}</select><button disabled={!!busy||!loadId} onClick={loadPath}><FolderOpen/>加载路径</button><button className="primary" disabled={!sessionReady||!!busy||!name.trim()} onClick={savePath}><Save/>保存路径</button><button className="danger-button" disabled={!sessionReady||!!busy||!document} onClick={deletePath}><Trash2/>删除路径</button></div>
      <p className="lock-note"><ShieldCheck size={16}/>稳定 ID：<b>{document?.trajectory_id||'创建后分配'}</b>。后端原子写入 JSON；示教点或参数变化后必须重新生成。</p>
    </Panel>
    <Panel title="轨迹执行" eyebrow="EXISTING GUARDED ACTION CHAIN">
      <div className="trajectory-execution"><Metric label="执行状态" value={execution.state} accent={execution.state==='EXECUTING'}/><Metric label="当前目标点" value={`${execution.current_waypoint} / ${execution.total_waypoints||document?.waypoints.length||0}`}/><Metric label="执行进度" value={percent} unit="%"/><Metric label="实际跟踪误差" value={execution.max_error_rad==null?'—':execution.max_error_rad.toFixed(4)} unit="rad"/></div><div className="trajectory-progress"><i style={{width:`${percent}%`}}/></div>
      <div className="button-row"><button className="primary" disabled={!sessionReady||!!busy||!ready||execution.state==='EXECUTING'} onClick={execute}><Play/>执行路径</button><button className="danger-button" disabled={!sessionReady||!!busy||execution.state!=='EXECUTING'} onClick={stop}><Square/>停止执行</button></div>
      {(execution.error||notice)&&<p className={`tool-notice ${execution.error?'critical':''}`}>{execution.error||notice}</p>}
    </Panel>
  </div>
}

type LimitRecoveryResult={ok:boolean;needs_recovery:boolean;recovered:boolean;exit_code:number;command:string;host:string;port:number;target:string;output:string}
function Diagnostics({state}:{state:StationState}) {
  const {sessionReady}=useStation()
  const [limitResult,setLimitResult]=useState<LimitRecoveryResult|null>(null)
  const [limitBusy,setLimitBusy]=useState<'query'|'recover'|''>('')
  const [confirmLimit,setConfirmLimit]=useState(false)
  const [limitTarget,setLimitTarget]=useState('192.168.1.18:8080')
  useEffect(()=>{api<{target?:string}>('/api/v1/limit-recovery/config').then(value=>{if(value.target)setLimitTarget(value.target)}).catch(()=>{})},[])
  const queryLimit=async()=>{setLimitBusy('query');try{setLimitResult(await api<LimitRecoveryResult>('/api/v1/limit-recovery'))}finally{setLimitBusy('')}}
  const recoverLimit=async()=>{setConfirmLimit(false);setLimitBusy('recover');try{setLimitResult(await api<LimitRecoveryResult>('/api/v1/limit-recovery',{method:'POST'}))}finally{setLimitBusy('')}}
  const commandPath='/home/zjh/project/RealMan/realman_limit_recovery.py'
  return <div className="page-grid">
    <Panel title="活动故障" eyebrow="LATCHED & LIVE" className="span-2"><div className="fault-table"><div className="table-head"><span>级别</span><span>代码</span><span>原因</span><span>状态</span><span>时间</span></div>{state.diagnostics.length?state.diagnostics.map((f,i)=><div className="table-row" key={`${f.code}-${i}`}><b className={f.level.toLowerCase()}>{f.level}</b><code>{f.code}</code><span>{f.reason}</span><span>{f.status}</span><time>{new Date(f.timestamp*1000).toLocaleTimeString()}</time></div>):<div className="all-clear"><ShieldCheck/>未发现活动故障</div>}</div></Panel>
    <Panel title="恢复控制" eyebrow="SAFE RECOVERY"><StateBadge state={state.operator_state}/><p>恢复按钮调用既有清错、ACK 和任务恢复流程，不会通过清除网页状态解除锁存。</p><button className="primary full" disabled={!sessionReady} onClick={()=>command('recover')}>执行受控恢复</button></Panel>
    <Panel title="机械臂软限位" eyebrow="0X1016 · LIMIT RECOVERY" className="span-2"><p className="muted">先执行只读查询。修复会清除 4118 错误，并让接近软限位的关节以 5% 速度退到安全裕量。当前目标控制器：<b>{limitResult?.target||limitTarget}</b></p><div className="limit-command"><code>{commandPath}{limitResult?.command.endsWith('--recover --yes')?' --recover --yes':''}</code></div><div className="button-row"><button disabled={!!limitBusy} onClick={queryLimit}>{limitBusy==='query'?'查询中…':'查询限位'}</button><button className="danger-button" disabled={!!limitBusy||!sessionReady||state.offline||state.simulation} onClick={()=>setConfirmLimit(true)}>{limitBusy==='recover'?'修复中…':'修复限位'}</button></div>{limitResult&&<><div className={`limit-summary ${limitResult.recovered||(!limitResult.needs_recovery&&limitResult.ok)?'ok':limitResult.needs_recovery?'warning':''}`}>{limitResult.recovered?'限位修复并复检成功':limitResult.needs_recovery?'检测到关节接近软限位，需要修复':'未检测到需修复的软限位'}</div><pre className="hardware-output limit-output">{limitResult.output}</pre></>}<p className="safety-note"><ShieldCheck size={16}/>修复前必须取得控制权，并确认机械臂周围净空、急停可用且所有人员远离运动范围。</p></Panel>
    <Panel title="事件时间线" eyebrow="RECENT EVENTS"><EventList state={state}/></Panel>
    {confirmLimit&&<ConfirmDialog title="确认修复机械臂软限位" detail={`${commandPath} --recover --yes\n目标控制器：${limitTarget}\n\n该命令会清除 4118 错误，并驱动接近软限位的关节低速运动。请确认现场净空、急停可用、所有人员均已离开机械臂运动范围。`} cancelLabel="取消，不执行修复" confirmLabel="确认现场安全并修复" onCancel={()=>setConfirmLimit(false)} onConfirm={recoverLimit}/>}
  </div>
}

function Parameters() {const [data,setData]=useState<Record<string,any>|null>(null);useEffect(()=>{api<Record<string,any>>('/api/v1/parameters').then(setData)},[]);return <div className="page-grid parameters"><Panel title="只读参数快照" eyebrow="YAML · READ ONLY" className="span-3"><p className="muted">网页不提供写参数控件，不会修改运行中的 ROS 安全阈值。</p>{data&&Object.entries(data.sources||{}).map(([name,src]:any)=><details key={name} open><summary>{name}<small>{src.path}</small></summary><pre>{JSON.stringify(src.values,null,2)}</pre></details>)}</Panel></div>}

function PoseTable({value}:{value:Dict}) {const flat=JSON.stringify(value);return <div className="pose-grid"><Metric label="位置 X / Y / Z" value={flat.length>2?'见实时状态':'—'} unit="m"/><Metric label="姿态 qx/qy/qz/qw" value={flat.length>2?'有效观测':'—'}/><pre>{JSON.stringify(value,null,2)}</pre></div>}
function EventList({state}:{state:StationState}) {return <div className="event-list">{state.events.slice(-12).reverse().map((e,i)=><div key={i}><time>{new Date(e.timestamp*1000).toLocaleTimeString()}</time><b className={e.level.toLowerCase()}>{e.level}</b><span>{e.message}</span></div>)}</div>}

export function App() {
  useConnection()
  const {view,setView,state,model,connected,sessionId,sessionReady,setSession,lastError,setError,loadSettings}=useStation()
  const [menu,setMenu]=useState(false)
  useEffect(()=>{void loadSettings()},[loadSettings])
  useEffect(()=>{
    const report=(event:PromiseRejectionEvent)=>{
      event.preventDefault()
      const reason=event.reason
      setError(reason instanceof Error?reason.message:String(reason||'操作未完成'))
    }
    window.addEventListener('unhandledrejection',report)
    return()=>window.removeEventListener('unhandledrejection',report)
  },[setError])
  const acquire=async()=>{try{if(state?.offline){throw new Error('OFFLINE模式禁止取得控制权')};const r=await api<{session_id:string}>('/api/v1/control/session',{method:'POST',body:JSON.stringify({client:navigator.userAgent})});setSession(r.session_id)}catch(e){setError(String(e))}}
  const release=async()=>{try{if(['JOGGING','STOPPING','LIMIT_REACHED'].includes(state?.cartesian_jog?.state||''))await api('/api/v1/cartesian/jog/stop',{method:'POST'});await api('/api/v1/control/session',{method:'DELETE'})}finally{setSession(null)}}
  const estop=()=>command('emergency_stop').catch(e=>setError(String(e)))
  const content=useMemo(()=>{if(!state)return <div className="loading"><RefreshCw className="spin"/>正在连接 RM75 Web Backend…</div>;if(view==='dashboard')return <Dashboard state={state}/>;if(view==='robot')return model?<RobotPage state={state} model={model}/>:<div className="loading">装载厂商模型…</div>;if(view==='cartesian')return <CartesianJogPage state={state}/>;if(view==='trajectory')return <TrajectoryPage state={state}/>;if(view==='force')return <ForcePage state={state}/>;if(view==='qr')return <VisionPage state={state}/>;if(view==='tools')return <ToolsPage state={state}/>;if(view==='robotctl')return <RobotControlPage state={state}/>;if(view==='sampling')return <SamplingPage state={state}/>;if(view==='diagnostics')return <Diagnostics state={state}/>;return <Parameters/>},[view,state,model])
  return <div className="app"><aside className={`sidebar ${menu?'open':''}`}><div className="brand"><div className="brand-mark"><Bot/></div><div><b>RM75</b><span>ALPINE SAMPLING</span></div></div><nav>{nav.map(([key,label,Icon])=><button key={key} className={view===key?'active':''} onClick={()=>{setView(key);setMenu(false)}}><Icon/><span>{label}</span></button>)}</nav><div className="sidebar-foot"><span>CONTROL PLATFORM</span><b>v1.0 · ROS Noetic</b></div></aside><main><header className="topbar"><button className="menu-button" onClick={()=>setMenu(!menu)}><Menu/></button><div className="breadcrumb"><span>高原寒区采样机器人</span><ChevronRight/><b>{nav.find(n=>n[0]===view)?.[1]}</b></div><div className="global-status"><span className={`connection ${connected?'on':''}`}><Radio/>{connected?'WebSocket 实时':'通信断开'}</span>{state?.offline&&<span className="offline">OFFLINE / 禁止控制</span>}{state?.simulation&&<span className="simulation">SIMULATION / 无物理输出</span>}<StateBadge state={state?.operator_state}/>{sessionId?<button className={`lease ${sessionReady?'ready':''}`} onClick={release}><Power/>{sessionReady?'控制权已就绪':'建立心跳…'}</button>:<button className="lease" onClick={acquire}><Unplug/>取得控制权</button>}<button className="estop" disabled={!sessionReady} onClick={estop}><OctagonX/>软件急停</button></div></header><div className="content">{lastError&&<div className="toast" onClick={()=>setError('')}><b>操作未完成</b><span>{lastError}</span><i>×</i></div>}{content}</div><footer><div><Activity/>刷新序列 <b>{state?.sequence??'—'}</b></div><div>数据时间 <b>{state?new Date(state.timestamp*1000).toLocaleTimeString():'—'}</b></div><div>控制会话 <b>{sessionReady?'独占 / 心跳有效':'只读'}</b></div></footer></main></div>
}
