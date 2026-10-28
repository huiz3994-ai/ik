import assert from 'node:assert/strict'
import test from 'node:test'
import {CartesianJogInterlock, type JogDirection} from '../src/cartesian-jog-interlock.ts'

function deferred(){
  let resolve!:()=>void
  const promise=new Promise<void>(done=>{resolve=done})
  return {promise,resolve}
}

async function nextTurn(){await new Promise(resolve=>setTimeout(resolve,0))}

test('release before start completion forces a stop after the late start',async()=>{
  const start=deferred(),stops=[deferred(),deferred()]
  let stopCalls=0
  const states:{direction:JogDirection|null;stopping:boolean}[]=[]
  const errors:unknown[]=[]
  const controller=new CartesianJogInterlock(
    ()=>start.promise,
    ()=>stops[stopCalls++].promise,
    (direction,stopping)=>states.push({direction,stopping}),
    error=>errors.push(error),
  )

  const started=controller.start('x_plus')
  const released=controller.release()
  assert.equal(stopCalls,1)

  // The first stop may have observed IDLE while start was still planning.
  stops[0].resolve()
  await released
  start.resolve()
  await nextTurn()
  assert.equal(stopCalls,2)

  stops[1].resolve()
  assert.equal(await started,true)
  assert.deepEqual(errors,[])
  assert.deepEqual(states.at(-1),{direction:null,stopping:false})
})

test('repeated releases share one in-flight stop request',async()=>{
  const stop=deferred()
  let stopCalls=0
  const controller=new CartesianJogInterlock(
    async()=>undefined,
    ()=>{++stopCalls;return stop.promise},
    ()=>undefined,
    error=>assert.fail(String(error)),
  )

  assert.equal(await controller.start('rz_minus'),true)
  const first=controller.release()
  const second=controller.release()
  assert.equal(stopCalls,1)
  assert.equal(await controller.start('x_plus'),false)
  stop.resolve()
  await Promise.all([first,second])
  assert.equal(controller.busy,false)
})
