/* Ejecuta trabajos del simulador en paralelo (un proceso por ronda) e imprime una linea de log cada vez que termina uno.
 *   const { runJobs, workerJob } = require('./par');
 *   if (workerJob()) { run(workerJob()).then(...) } else runJobs(__filename, jobs).then(...)
 * Cada ronda del simulador usa un solo nucleo, asi que con N nucleos libres el tiempo total baja casi N veces. */
'use strict';
const { fork } = require('node:child_process'), os = require('node:os');

function workerJob() {
  return process.argv[2] === '--job' ? JSON.parse(process.argv[3]) : null;
}
function workerDone(result) {
  process.send(result, () => process.exit(0));
}
function runJobs(script, jobs, opts = {}) {
  const log = opts.log || ((s) => console.error(s));
  /* Hay que dejar memoria: un proceso de ronda llega a ~100 MB, pero V8 puede pedir mucho mas al compilar el wasm y, con la memoria libre al limite, aborta con
   * 'Fatal process out of memory: Zone' (medido: 13 de 24 procesos con la memoria libre justa). Se limita a ~500 MB libres por proceso y se reintenta una ronda
   * cuyo proceso se cayo sin dar resultado. */
  const byMem = Math.max(1, Math.floor(os.freemem() / (500 * 1024 * 1024)));
  const max = Math.max(1, Math.min(jobs.length, (opts.workers || os.cpus().length - 2), byMem));
  const retries = opts.retries === undefined ? 2 : opts.retries;
  const t0 = Date.now();
  log(`[0s] ${jobs.length} trabajos en ${max} procesos en paralelo`);
  return new Promise((resolve) => {
    const out = new Array(jobs.length);
    let next = 0, active = 0, finished = 0;
    const queue = [], tries = new Array(jobs.length).fill(0);
    const launch = () => {
      while (active < max && next < jobs.length) {
        const index = queue.length ? queue.shift() : next++, job = jobs[index];
        active++;
        const child = fork(script, ['--job', JSON.stringify(job)], { stdio: ['ignore', 'inherit', 'inherit', 'ipc'] });
        let got = null;
        child.on('message', (m) => { got = m; });
        child.on('exit', (code) => {
          active--;
          if (got === null && code !== 0 && tries[index] < retries) {      /* proceso caido (p. ej. sin memoria): se repite */
            ++tries[index]; queue.push(index);
            log(`[${((Date.now() - t0) / 1000).toFixed(0)}s] reintento ${tries[index]} de ${JSON.stringify(job)} (codigo ${code})`);
            launch();
            return;
          }
          finished++;
          out[index] = { job, result: got, code };
          const note = got && got.log ? ' ' + got.log : '';
          log(`[${((Date.now() - t0) / 1000).toFixed(0)}s] ${finished}/${jobs.length} ${JSON.stringify(job)} ${code === 0 ? 'ok' : 'ERROR codigo ' + code}${note}`);
          if (finished === jobs.length) resolve(out); else launch();
        });
      }
    };
    launch();
  });
}
module.exports = { runJobs, workerJob, workerDone };
