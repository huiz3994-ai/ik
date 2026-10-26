import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { Activity, Bot, Camera, ChevronRight, CircleGauge, Eye, FileSliders, FlaskConical, Menu, OctagonX, PackageCheck, Power, Radio, RefreshCw, Save, Server, ShieldCheck, Siren, Unplug, Waves } from 'lucide-react'
import { CartesianGrid, Legend, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts'
import { JointPanel, Metric, Panel, RobotTwin, StateBadge, StatusPill } from './components'
import { api, command, useStation } from './store'
import type { Dict, RobotDescriptor, StationState, ToolPickConfig } from './types'

const nav = [
  ['dashboard','操作台',CircleGauge],['robot','机器人状态',Bot],['force','实时力监控',Waves],
  ['vision','视觉识别',Eye],['tools','五工具配置',PackageCheck],['sampling','采样设备',FlaskConical],['diagnostics','故障诊断',Siren],['parameters','系统参数',FileSliders]
] as const

function useConnection() {
  const {sessionId,setSessionReady,setState,setConnected,setModel,setError}=useStation()
  const retry=useRef(0)
  useEffect(()=>{ api<RobotDescriptor>('/api/v1/model').then(setModel).catch(e=>setError(String(e))) },[setModel,setError])
  useEffect(()=>{
    let socket:WebSocket|undefined, stopped=false, heartbeat:number|undefined, reconnect:number|undefined
    const connect=()=>{
      const scheme=location.protocol==='https:'?'wss':'ws'
      socket=new WebSocket(`${scheme}://${location.host}/api/v1/ws`)
      socket.onopen=()=>{retry.current=0;setConnected(true); if(sessionId) socket?.send(JSON.stringify({type:'heartbeat',session_id:sessionId}))}
      socket.onmessage=e=>{const m=JSON.parse(e.data);if(m.type==='state')setState(m.data);if(m.type==='heartbeat_ack')setSessionReady(true);if(m.type==='error')setError(m.message)}
      socket.onclose=()=>{setConnected(false);setSessionReady(false);if(!stopped){const delay=Math.min(5000,500*2**retry.current++);reconnect=window.setTimeout(connect,delay)}}
      socket.onerror=()=>socket?.close()
      heartbeat=window.setInterval(()=>{if(socket?.readyState===WebSocket.OPEN){socket.send(JSON.stringify(sessionId?{type:'heartbeat',session_id:sessionId}:{type:'ping'}))}},200)
    }
    connect(); return()=>{stopped=true;if(heartbeat)clearInterval(heartbeat);if(reconnect)clearTimeout(reconnect);socket?.close()}
  },[sessionId,setConnected,setError,setSessionReady,setState])
}

function Dashboard({state}:{state:StationState}) {
  const run=async(name:string,action:string)=>api(`/api/v1/system/${name}/${action}`,{method:'POST'})
  const systems=[['base','RM75 基础栈','驱动 · MoveIt · TF'],['camera','D435i RGB','424×240 · 6 Hz'],['qr','Mark / QR','定位与任务服务']]
  return <div className="page-grid dashboard-grid">
    <Panel title="系统态势" eyebrow="SYSTEM OVERVIEW" className="span-2"><div className="status-grid">
      <StatusPill label="ROS Master" ok={!!state.health.master}/><StatusPill label="RM75 控制器" ok={!!state.health.driver&&!!state.fresh.joints} detail="驱动节点在线且收到新鲜控制器反馈才视为已连接"/>
      <StatusPill label="七轴反馈" ok={!!state.fresh.joints}/><StatusPill label="MoveIt" ok={!!state.health.joint_control||!!state.subsystems.base?.running}/>
      <StatusPill label="D435i" ok={!!state.health.camera||!!state.fresh.image}/><StatusPill label="Geomagic Touch" ok={!!state.health.touch}/>
      <StatusPill label="六维力" ok={!!state.fresh.wrench}/><StatusPill label="Mark识别" ok={!!state.health.marker}/>
    </div></Panel>
    <Panel title="安全与控制" eyebrow="SAFETY"><div className="safety-hero"><StateBadge state={state.operator_state}/><Metric label="控制模式" value={state.control_launch_mode||state.control_mode||'IDLE'}/><Metric label="当前工具" value={state.selected_tool||'未选择'}/><Metric label="工具电压" value={state.tool_voltage??'—'} unit="V"/></div><div className="button-row power-buttons"><button onClick={()=>command('set_voltage',{voltage:24})}>工具 24V</button><button onClick={()=>command('set_voltage',{voltage:0})}>工具 0V</button></div></Panel>
    <Panel title="生命周期" eyebrow="SUPERVISED LAUNCH" className="span-2"><div className="lifecycle-list">{systems.map(([key,name,detail])=>{const s=state.subsystems[key];return <div className="lifecycle" key={key}><div className={`node-icon ${s?.running?'on':''}`}><Server size={19}/></div><div><b>{name}</b><span>{detail}</span></div><div className="lifecycle-state">{s?.running?'运行中':'已停止'}{s?.owned&&<small>本平台管理</small>}</div><button onClick={()=>run(key,s?.running?'stop':'start')}>{s?.running?'停止':'启动'}</button></div>})}</div></Panel>
    <Panel title="最近系统事件" eyebrow="EVENT STREAM"><EventList state={state}/></Panel>
  </div>
}

function RobotPage({state,model}:{state:StationState;model:RobotDescriptor}) {
  const {target,setTarget,sessionReady}=useStation()
  const actual=Array.from({length:7},(_,i)=>state.joints?.[`joint${i+1}`]||0)
  const exec=()=>api('/api/v1/joints/execute',{method:'POST',body:JSON.stringify({target})})
  return <div className="robot-layout"><div className="robot-stage"><RobotTwin descriptor={model} actual={actual} target={target} actualFresh={!!state.fresh.joints}/><div className="model-proof"><b>{model.model}</b><span>{model.link7_variant} · J7 Z 0.1612 m</span><small>厂商 xacro / STL 实时装载</small></div></div><aside><div className="aside-title"><div><span>JOINT CONTROL</span><h2>七轴离线目标</h2></div><StateBadge state={state.operator_state}/></div><JointPanel state={state} model={model} target={target} onTarget={setTarget} onExecute={()=>sessionReady?exec():Promise.reject(new Error('请先取得控制权并等待心跳就绪'))} onCancel={()=>command('cancel_joints')}/></aside></div>
}

function ForcePage({state}:{state:StationState}) {
  const [paused,setPaused]=useState(false), [samples,setSamples]=useState<Record<string,number>[]>([])
  useEffect(()=>{if(paused)return;const w=state.wrench||{};setSamples(v=>[...v.slice(-299),{t:Date.now(),Fx:w.Fx||0,Fy:w.Fy||0,Fz:w.Fz||0,setpoint:state.force_setpoint||0}])},[state.sequence,paused,state.force_setpoint,state.wrench])
  const w=state.wrench||{}, actual=state.filtered_force_z??state.force_z??w.Fz
  const zForceActive=state.control_launch_mode==='z_force_teleop'
  const hc=state.haptic_command, hapticNorm=hc?Math.hypot(hc.x,hc.y,hc.z):0
  return <div className="page-grid force-grid"><Panel title="六维力实时值" eyebrow="WRENCH · SENSOR FRAME" className="span-2"><div className="force-metrics">{['Fx','Fy','Fz','Tx','Ty','Tz'].map(k=><Metric key={k} label={k.replace('T','M')} value={Number.isFinite(w[k])?w[k].toFixed(3):'—'} unit={k.startsWith('T')?'N·m':'N'} accent={k==='Fz'}/>)}</div></Panel><Panel title="力控闭环" eyebrow="Z-ONLY HYBRID + HAPTIC"><div className="force-loop"><Metric label="目标下压力" value={state.force_setpoint?.toFixed(2)||'—'} unit="N"/><ChevronRight/><Metric label="实际下压力" value={Number.isFinite(actual)?Number(actual).toFixed(2):'—'} unit="N" accent/><ChevronRight/><Metric label="力误差" value={Number.isFinite(actual)&&Number.isFinite(state.force_setpoint)?(Number(state.force_setpoint)-Number(actual)).toFixed(2):'—'} unit="N"/></div><dl><dt>运动自由度</dt><dd>仅 Z；X/Y/姿态锁定</dd><dt>Touch力反馈</dt><dd>{zForceActive?(state.haptic_enabled?(state.haptic_force_fresh?`已开启 · ${hapticNorm.toFixed(3)} N`:'已开启 · 等待新鲜力'):'启动中'):'未开启'}</dd><dt>控制模式</dt><dd>{state.control_mode||'—'}</dd><dt>停止原因</dt><dd>{state.stop_reason||'无'}</dd><dt>反馈新鲜度</dt><dd>{state.fresh.wrench?`${((state.ages.wrench||0)*1000).toFixed(0)} ms`:'过期 / 不可用'}</dd></dl><div className="button-grid"><button className={zForceActive?'secondary':'primary'} onClick={()=>command(zForceActive?'stop_z_force_teleop':'start_z_force_teleop')}>{zForceActive?'停止 Z 力位遥操作':'启动 Z 力位 + 力反馈'}</button><button onClick={()=>command(state.control_launch_mode==='force_control'?'stop_sampling':'enter_force_control')}>{state.control_launch_mode==='force_control'?'停止独立力控':'独立垂直下压力控'}</button></div><p className="safety-note"><ShieldCheck size={16}/>灰键只控制 Z；接触后 Z 由力闭环接管。停止前先松开灰键，紧急情况使用顶部软件急停。</p></Panel><Panel title="Fz 滚动曲线" eyebrow="300 SAMPLES" className="span-2 chart-panel"><button className="chart-pause" onClick={()=>setPaused(!paused)}>{paused?'继续显示':'暂停显示'}</button><ResponsiveContainer width="100%" height={360}><LineChart data={samples}><CartesianGrid stroke="#17303d"/><XAxis dataKey="t" tickFormatter={v=>new Date(v).toLocaleTimeString()} stroke="#6c8694"/><YAxis unit=" N" stroke="#6c8694"/><Tooltip labelFormatter={v=>new Date(Number(v)).toLocaleTimeString()}/><Legend/><Line dataKey="Fz" stroke="#26d8cc" dot={false} isAnimationActive={false}/><Line dataKey="setpoint" stroke="#f3a451" dot={false} strokeDasharray="5 5" isAnimationActive={false}/></LineChart></ResponsiveContainer></Panel></div>
}

function VisionPage({state}:{state:StationState}) {
  const [tool,setTool]=useState(state.selected_tool||'Tool1'); const seq=(state as any).sequences?.image||0
  const select=()=>command('select_tool',{tool})
  return <div className="vision-layout"><Panel title="D435i RGB" eyebrow={`${state.image_topic||'/camera/color/image_raw'} · 424×240 @ 6Hz`} className="video-panel"><div className="video-frame">{state.fresh.image?<img src={`/api/v1/vision/frame.jpg?s=${seq}`} alt="D435i RGB live"/>:<div className="video-empty"><Camera size={52}/><b>等待 RGB 帧</b><span>当前链路仅声明 RGB-only，不冒充 RGB-D 验收</span></div>}<div className="video-overlay"><span>{state.health.camera?'CAMERA ONLINE':'CAMERA OFFLINE'}</span><span>{state.fresh.image?'LIVE':'NO SIGNAL'}</span></div></div></Panel><aside><Panel title="工具与标记" eyebrow="APRILTAG 16H5 · 22.5 mm"><div className="tool-picker">{[1,2,3,4,5].map(i=><button key={i} className={tool===`Tool${i}`?'active':''} onClick={()=>setTool(`Tool${i}`)}><b>Tool{i}</b><small>{state.five_target_status?'公共 ID 0':`ID ${i-1}`}</small></button>)}</div><button className="primary full" onClick={select}>选择 {tool}</button><div className="recognition"><Metric label="识别状态" value={String(state.vision_status?.ok??state.five_target_status?.ok??false)==='true'?'已识别':'未识别'}/><Metric label="稳定性" value={String(state.vision_status?.stable??state.five_target_status?.stable??false)==='true'?'稳定':'等待'}/></div><div className="button-grid"><button onClick={()=>command('detect_marker')}>识别标记</button><button onClick={()=>command('compute_targets')}>计算目标</button><button onClick={()=>command('preview_tool')}>预览目标</button><button onClick={()=>command('plan_tool')}>MoveIt规划</button></div><p className="lock-note"><ShieldCheck size={16}/>工具几何仍有 verified=false 项；识别成功不会自动解锁取放。</p></Panel><Panel title="六自由度位姿" eyebrow="BASE_LINK"><PoseTable value={(state.marker_0||state.vision_status||{}) as Dict}/></Panel></aside></div>
}

const emptyTool=(slot:number):ToolPickConfig=>({slot_id:slot,name:`Tool${slot}`,x:0,y:0,z:0,rx:0,ry:0,rz:0,lift_height:.2,position_unit:'m',angle_unit:'rad',pose_reference:'Link7',frame:'base_link'})

function ToolsPage({state}:{state:StationState}) {
  const {sessionReady}=useStation()
  const [tools,setTools]=useState<ToolPickConfig[]>(Array.from({length:5},(_,i)=>emptyTool(i+1)))
  const [slot,setSlot]=useState(1),[loading,setLoading]=useState(true),[busy,setBusy]=useState(false),[notice,setNotice]=useState('')
  const load=useCallback(()=>{setLoading(true);return api<{tools:ToolPickConfig[]}>('/api/v1/tools').then(r=>setTools(r.tools)).finally(()=>setLoading(false))},[])
  useEffect(()=>{load()},[load])
  const selected=tools.find(t=>t.slot_id===slot)||emptyTool(slot)
  const edit=(key:keyof ToolPickConfig,value:number)=>setTools(items=>items.map(item=>item.slot_id===slot?{...item,[key]:value}:item))
  const payload=()=>({x:selected.x,y:selected.y,z:selected.z,rx:selected.rx,ry:selected.ry,rz:selected.rz,lift_height:selected.lift_height})
  const save=async()=>{if(!sessionReady)throw new Error('请先取得控制权并等待心跳就绪');setBusy(true);try{const saved=await api<ToolPickConfig>(`/api/v1/tools/${slot}`,{method:'PUT',body:JSON.stringify(payload())});setTools(items=>items.map(item=>item.slot_id===slot?saved:item));setNotice(`Tool${slot} 配置已持久化`);return saved}finally{setBusy(false)}}
  const dryRun=async()=>{if(!sessionReady)throw new Error('请先取得控制权并等待心跳就绪');setBusy(true);try{await api<ToolPickConfig>(`/api/v1/tools/${slot}`,{method:'PUT',body:JSON.stringify(payload())});const plan=await api<any>(`/api/v1/tools/${slot}/dry-run`,{method:'POST'});setNotice(`Tool${slot} Dry Run 通过：下降 ${(Number(plan.descent_cartesian_fraction??1)*100).toFixed(1)}%，抬升 ${(Number(plan.lift_cartesian_fraction??1)*100).toFixed(1)}%；机械臂未运动`)}finally{setBusy(false);load()}}
  const execute=async()=>{if(!sessionReady)throw new Error('请先取得控制权并等待心跳就绪');setBusy(true);try{await api<ToolPickConfig>(`/api/v1/tools/${slot}`,{method:'PUT',body:JSON.stringify(payload())});const plan=await api<any>(`/api/v1/tools/${slot}/dry-run`,{method:'POST'});const c=plan.configuration||selected;const confirmed=window.confirm(`Tool${slot} Dry Run 已通过。\n\n目标 Link7（base_link）\nX ${Number(c.position_m?.[0]??c.x).toFixed(6)} m\nY ${Number(c.position_m?.[1]??c.y).toFixed(6)} m\nZ ${Number(c.position_m?.[2]??c.z).toFixed(6)} m\nLift ${Number(c.lift_height_m??c.lift_height).toFixed(4)} m\n\n将低速执行预抓取、严格Z下降、24V动作和严格Z抬升。是否确认执行？`);if(!confirmed){setNotice('用户取消：只完成 Dry Run，机械臂未运动');return}const result=await api<any>(`/api/v1/tools/${slot}/execute`,{method:'POST'});setNotice(result.grasp_confirmed?'执行完成并确认取物':'运动完成；24V ACK有效，但取物结果没有独立传感器确认')}finally{setBusy(false);load()}}
  const tcp=state.tcp||{}, status=state.configured_pick_status||{}
  const fields:[keyof ToolPickConfig,string][]=[['x','X'],['y','Y'],['z','Z'],['rx','Rx'],['ry','Ry'],['rz','Rz'],['lift_height','Lift Height']]
  const running=['PLANNING','RUNNING'].includes(status.status||'')
  return <div className="page-grid tool-config-page">
    <Panel title="五工具配置" eyebrow="ABSOLUTE LINK7 POSE · BASE_LINK" className="span-2">
      <div className="tool-tabs">{tools.map(tool=><button key={tool.slot_id} className={slot===tool.slot_id?'active':''} onClick={()=>setSlot(tool.slot_id)}>工具 {tool.slot_id}<small>{tool.updated_at?`已保存 ${new Date(tool.updated_at*1000).toLocaleString()}`:'项目默认值'}</small></button>)}</div>
      {loading?<div className="loading compact"><RefreshCw className="spin"/>读取配置…</div>:<div className="pose-form">{fields.map(([key,label])=><label key={key}><span>{label}</span><input type="number" step={key==='lift_height' ? .001 : .000001} value={Number(selected[key])} onChange={e=>edit(key,Number(e.target.value))}/><b>{key==='lift_height'||['x','y','z'].includes(key)?'m':'rad'}</b></label>)}</div>}
      <div className="tool-actions"><button disabled={busy||running||!sessionReady} onClick={()=>save()}><Save/>保存配置</button><button disabled={busy||running||!sessionReady} onClick={()=>dryRun()}><ShieldCheck/>仅 Dry Run</button><button className="primary" disabled={busy||running||!sessionReady} onClick={()=>execute()}><PackageCheck/>{busy?'检查中…':'确定并执行'}</button><button className="danger-button" disabled={!sessionReady||!running} onClick={()=>command('cancel_tool')}>终止执行</button></div>
      <p className="safety-note"><ShieldCheck size={16}/>保存使用原子替换，重启后仍保留。执行前必须先通过 MoveIt 可达性、限位、自碰撞和完整严格 Z 向路径 Dry Run；页面不会自动启动真机。</p>
      {notice&&<div className="tool-notice">{notice}</div>}
    </Panel>
    <Panel title="实时位姿" eyebrow="BASE_LINK → LINK7"><div className="live-pose">{['x','y','z','rx','ry','rz'].map(key=><Metric key={key} label={key.toUpperCase()} value={Number.isFinite(tcp[key])?Number(tcp[key]).toFixed(5):'—'} unit={['x','y','z'].includes(key)?'m':'rad'}/>)}</div><p className="lock-note"><ShieldCheck size={16}/>{state.fresh.tcp?'位姿反馈新鲜':'位姿反馈不可用或已过期，执行将被拒绝'}</p></Panel>
    <Panel title="当前执行步骤" eyebrow="FAIL-CLOSED SEQUENCE"><div className={`pick-state ${status.status==='FAILED'?'failed':status.status==='COMPLETED_UNVERIFIED'?'warning':''}`}><b>{status.status||'IDLE'}</b><strong>步骤 {status.step??0} / 6</strong><span>{status.step_name||'等待执行'}</span>{status.error&&<small>{status.error}</small>}</div><ol className="step-list"><li>运动到 Z+H 预抓取位姿</li><li>保持姿态，沿基坐标 Z 下降</li><li>调用现有 24V 末端动作</li><li>核验动作 ACK / 取物确认能力</li><li>保持姿态，沿基坐标 Z 抬升 H</li><li>保持当前位置等待</li></ol>{status.status==='COMPLETED_UNVERIFIED'&&<p className="unverified">运动成功；没有独立物体在位传感器，不能宣称“抓取成功”。</p>}</Panel>
    <Panel title="当前工具配置" eyebrow={`TOOL ${slot}`} className="span-2"><pre className="config-preview">{JSON.stringify({frame:selected.frame,pose_reference:selected.pose_reference,position_m:[selected.x,selected.y,selected.z],rpy_rad:[selected.rx,selected.ry,selected.rz],lift_height_m:selected.lift_height},null,2)}</pre></Panel>
  </div>
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
    <Panel title="DM-J4310 电机" eyebrow="CAN ID 0x01 · MIT MODE" className="span-2">
      <div className="motor-status"><StatusPill label="电机通信" ok={!!motor?.online}/><Metric label="位置" value={number(motor?.position,3)} unit="rad"/><Metric label="速度" value={number(motor?.velocity,3)} unit="rad/s"/><Metric label="力矩" value={number(motor?.torque,3)} unit="N·m"/></div>
      <pre className="hardware-output">{motor?.status||'尚未读取电机状态'}</pre>
    </Panel>
    <Panel title="电机受控动作" eyebrow="BOUNDED MOTION">
      <label className="hardware-field"><span>速度上限</span><input type="number" min="0.05" max="1" step="0.05" value={speed} onChange={e=>setSpeed(Math.min(1,Math.max(.05,Number(e.target.value)||.05)))}/><b>rad/s</b></label>
      <label className="hardware-field"><span>运行时长</span><input type="number" min="0.1" max="10" step="0.1" value={duration} onChange={e=>setDuration(Math.min(10,Math.max(.1,Number(e.target.value)||.1)))}/><b>s</b></label>
      <div className="motor-buttons"><button disabled={!motionReady||motor?.busy} onClick={()=>motorRun('forward')}>正转</button><button disabled={!motionReady||motor?.busy} onClick={()=>motorRun('reverse')}>反转</button><button className="primary" disabled={!motionReady||motor?.busy} onClick={()=>motorRun('dig')}>采样钻进</button><button className="danger-button" disabled={!controlsReady} onClick={()=>motorRun('stop')}>停止并失能</button><button disabled={!controlsReady||motor?.busy} onClick={()=>motorRun('status')}>读取状态</button></div>
      <p className="lock-note"><ShieldCheck size={16}/>动作限制为 ≤1.0 rad/s、≤10 s；电机程序仍会复核 CAN ID、模式与 PMAX/VMAX/TMAX。</p>
    </Panel>
    {!controlsReady&&<div className="hardware-lock span-3"><ShieldCheck/><div><b>当前仅可监视</b><span>{hw?.authorized?'请先取得控制权并等待心跳就绪。':'仅通过 --confirm-real-robot 启动时允许真实水泵和电机命令。'}</span></div></div>}
  </div>
}

function Diagnostics({state}:{state:StationState}) {return <div className="page-grid"><Panel title="活动故障" eyebrow="LATCHED & LIVE" className="span-2"><div className="fault-table"><div className="table-head"><span>级别</span><span>代码</span><span>原因</span><span>状态</span><span>时间</span></div>{state.diagnostics.length?state.diagnostics.map((f,i)=><div className="table-row" key={`${f.code}-${i}`}><b className={f.level.toLowerCase()}>{f.level}</b><code>{f.code}</code><span>{f.reason}</span><span>{f.status}</span><time>{new Date(f.timestamp*1000).toLocaleTimeString()}</time></div>):<div className="all-clear"><ShieldCheck/>未发现活动故障</div>}</div></Panel><Panel title="恢复控制" eyebrow="SAFE RECOVERY"><StateBadge state={state.operator_state}/><p>恢复按钮调用既有清错、ACK 和任务恢复流程，不会通过清除网页状态解除锁存。</p><button className="primary full" onClick={()=>command('recover')}>执行受控恢复</button></Panel><Panel title="事件时间线" eyebrow="RECENT EVENTS"><EventList state={state}/></Panel></div>}

function Parameters() {const [data,setData]=useState<Record<string,any>|null>(null);useEffect(()=>{api<Record<string,any>>('/api/v1/parameters').then(setData)},[]);return <div className="page-grid parameters"><Panel title="只读参数快照" eyebrow="YAML · READ ONLY" className="span-3"><p className="muted">网页不提供写参数控件，不会修改运行中的 ROS 安全阈值。</p>{data&&Object.entries(data.sources||{}).map(([name,src]:any)=><details key={name} open><summary>{name}<small>{src.path}</small></summary><pre>{JSON.stringify(src.values,null,2)}</pre></details>)}</Panel></div>}

function PoseTable({value}:{value:Dict}) {const flat=JSON.stringify(value);return <div className="pose-grid"><Metric label="位置 X / Y / Z" value={flat.length>2?'见实时状态':'—'} unit="m"/><Metric label="姿态 qx/qy/qz/qw" value={flat.length>2?'有效观测':'—'}/><pre>{JSON.stringify(value,null,2)}</pre></div>}
function EventList({state}:{state:StationState}) {return <div className="event-list">{state.events.slice(-12).reverse().map((e,i)=><div key={i}><time>{new Date(e.timestamp*1000).toLocaleTimeString()}</time><b className={e.level.toLowerCase()}>{e.level}</b><span>{e.message}</span></div>)}</div>}

export function App() {
  useConnection()
  const {view,setView,state,model,connected,sessionId,sessionReady,setSession,lastError,setError}=useStation()
  const [menu,setMenu]=useState(false)
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
  const release=async()=>{try{await api('/api/v1/control/session',{method:'DELETE'})}finally{setSession(null)}}
  const estop=()=>command('emergency_stop').catch(e=>setError(String(e)))
  const content=useMemo(()=>{if(!state)return <div className="loading"><RefreshCw className="spin"/>正在连接 RM75 Web Backend…</div>;if(view==='dashboard')return <Dashboard state={state}/>;if(view==='robot')return model?<RobotPage state={state} model={model}/>:<div className="loading">装载厂商模型…</div>;if(view==='force')return <ForcePage state={state}/>;if(view==='vision')return <VisionPage state={state}/>;if(view==='tools')return <ToolsPage state={state}/>;if(view==='sampling')return <SamplingPage state={state}/>;if(view==='diagnostics')return <Diagnostics state={state}/>;return <Parameters/>},[view,state,model])
  return <div className="app"><aside className={`sidebar ${menu?'open':''}`}><div className="brand"><div className="brand-mark"><Bot/></div><div><b>RM75</b><span>ALPINE SAMPLING</span></div></div><nav>{nav.map(([key,label,Icon])=><button key={key} className={view===key?'active':''} onClick={()=>{setView(key);setMenu(false)}}><Icon/><span>{label}</span></button>)}</nav><div className="sidebar-foot"><span>CONTROL PLATFORM</span><b>v1.0 · ROS Noetic</b></div></aside><main><header className="topbar"><button className="menu-button" onClick={()=>setMenu(!menu)}><Menu/></button><div className="breadcrumb"><span>高原寒区采样机器人</span><ChevronRight/><b>{nav.find(n=>n[0]===view)?.[1]}</b></div><div className="global-status"><span className={`connection ${connected?'on':''}`}><Radio/>{connected?'WebSocket 实时':'通信断开'}</span>{state?.offline&&<span className="offline">OFFLINE / 禁止控制</span>}{state?.simulation&&<span className="simulation">SIMULATION / 无物理输出</span>}<StateBadge state={state?.operator_state}/>{sessionId?<button className={`lease ${sessionReady?'ready':''}`} onClick={release}><Power/>{sessionReady?'控制权已就绪':'建立心跳…'}</button>:<button className="lease" onClick={acquire}><Unplug/>取得控制权</button>}<button className="estop" disabled={!sessionReady} onClick={estop}><OctagonX/>软件急停</button></div></header><div className="content">{lastError&&<div className="toast" onClick={()=>setError('')}><b>操作未完成</b><span>{lastError}</span><i>×</i></div>}{content}</div><footer><div><Activity/>刷新序列 <b>{state?.sequence??'—'}</b></div><div>数据时间 <b>{state?new Date(state.timestamp*1000).toLocaleTimeString():'—'}</b></div><div>控制会话 <b>{sessionReady?'独占 / 心跳有效':'只读'}</b></div></footer></main></div>
}
