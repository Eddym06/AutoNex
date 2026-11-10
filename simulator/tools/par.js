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
  const max = Math.max(1, Math.min(jobs.length, (opts.workers || os.cpus().length - 2)));
  const t0 = Date.now();
  log(`[0s] ${jobs.length} trabajos en ${max} procesos en paralelo`);
  return new Promise((resolve) => {
    const out = new Array(jobs.length);
    let next = 0, active = 0, finished = 0;
    const launch = () => {
      while (active < max && next < jobs.length) {
        const index = next++, job = jobs[index];
        active++;
        const child = fork(script, ['--job', JSON.stringify(job)], { stdio: ['ignore', 'inherit', 'inherit', 'ipc'] });
        let got = null;
        child.on('message', (m) => { got = m; });
        child.on('exit', (code) => {
          active--; finished++;
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
