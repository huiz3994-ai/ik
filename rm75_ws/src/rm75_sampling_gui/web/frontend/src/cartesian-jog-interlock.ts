export type JogDirection='x_plus'|'x_minus'|'y_plus'|'y_minus'|'z_plus'|'z_minus'|'rx_plus'|'rx_minus'|'ry_plus'|'ry_minus'|'rz_plus'|'rz_minus'

type Phase='start'|'stop'
type StateListener=(direction:JogDirection|null,stopping:boolean)=>void
type ErrorListener=(error:unknown,phase:Phase)=>void

/**
 * Serializes hold-to-jog start/stop requests.
 *
 * A release can reach the stop service while a slow start request is still
 * doing collision and IK checks.  When that happens, the completed start is
 * followed by another stop so motion cannot begin after the operator let go.
 */
export class CartesianJogInterlock {
  private activeDirection:JogDirection|null=null
  private generation=0
  private pendingStop:Promise<void>|null=null
  private readonly requestStart:(direction:JogDirection)=>Promise<unknown>
  private readonly requestStop:()=>Promise<unknown>
  private readonly onState:StateListener
  private readonly onError:ErrorListener

  constructor(
    requestStart:(direction:JogDirection)=>Promise<unknown>,
    requestStop:()=>Promise<unknown>,
    onState:StateListener,
    onError:ErrorListener,
  ) {
    this.requestStart=requestStart
    this.requestStop=requestStop
    this.onState=onState
    this.onError=onError
  }

  get direction(){return this.activeDirection}
  get stopping(){return this.pendingStop!==null}
  get busy(){return this.activeDirection!==null||this.pendingStop!==null}

  async start(direction:JogDirection):Promise<boolean>{
    if(this.busy)return false
    this.activeDirection=direction
    const generation=++this.generation
    this.publish()
    try{
      await this.requestStart(direction)
      if(this.activeDirection!==direction||generation!==this.generation){
        await this.stop(true)
      }
      return true
    }catch(error){
      if(generation===this.generation)this.activeDirection=null
      this.publish()
      this.onError(error,'start')
      return false
    }
  }

  async release():Promise<void>{
    if(!this.busy)return
    ++this.generation
    this.activeDirection=null
    this.publish()
    await this.stop(false)
  }

  reset():void{
    ++this.generation
    this.activeDirection=null
    this.publish()
  }

  private async stop(forceAfterPending:boolean):Promise<void>{
    const pending=this.pendingStop
    if(pending){
      await pending
      if(!forceAfterPending)return
    }
    const operation=(async()=>{
      try{await this.requestStop()}
      catch(error){this.onError(error,'stop')}
    })()
    this.pendingStop=operation
    this.publish()
    try{await operation}
    finally{
      if(this.pendingStop===operation)this.pendingStop=null
      this.publish()
    }
  }

  private publish():void{
    this.onState(this.activeDirection,this.pendingStop!==null)
  }
}
