// glass-lio course, lesson 7: H_t = sum n n^T over three planes, and the degeneracy gate.
(function () {
  'use strict';

  // Deterministic PRNG, so the explorer's point cloud does not change between renders.
  function mulberry32(seed) {
    return function () {
      seed |= 0;
      seed = (seed + 0x6d2b79f5) | 0;
      let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  // Eigen-decomposition of a symmetric 3x3 matrix by cyclic Jacobi rotations.
  // Returns eigenvalues ascending, with their unit eigenvectors.
  function symEig3(m) {
    const a = m.map((r) => r.slice());
    const v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for (let sweep = 0; sweep < 50; sweep++) {
      const off = a[0][1] ** 2 + a[0][2] ** 2 + a[1][2] ** 2;
      if (off < 1e-20) break;
      for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
        if (Math.abs(a[p][q]) < 1e-30) continue;
        const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < 3; k++) {
          const akp = a[k][p];
          const akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < 3; k++) {
          const apk = a[p][k];
          const aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 3; k++) {
          const vkp = v[k][p];
          const vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
    return [0, 1, 2]
      .map((i) => ({value: a[i][i], vector: [v[0][i], v[1][i], v[2][i]]}))
      .sort((x, y) => x.value - y.value);
  }

  // ---- Lesson 7: H_t = sum n n^T for points on three planes, and the degeneracy gate.
  function setupScatterExplorers() {
    document.querySelectorAll('.scatter-explorer').forEach((box, boxIndex) => {
      const gate = parseFloat(box.dataset.gate || '0.05');
      const init = (box.dataset.init || '100,0,100').split(',').map((s) => parseInt(s, 10) || 0);
      const MAX = 200;
      const NOISE = 0.04;   // normals are not perfectly axis-aligned in a real scan
      const planes = [
        {name: 'Wall A (normal ≈ x)', axis: [1, 0, 0]},
        {name: 'Wall B (normal ≈ y)', axis: [0, 1, 0]},
        {name: 'Floor (normal ≈ z)', axis: [0, 0, 1]},
      ];

      const rand = mulberry32(12345 + boxIndex);
      const gauss = () => Math.sqrt(-2 * Math.log(1 - rand())) * Math.cos(2 * Math.PI * rand());
      planes.forEach((pl) => {
        pl.normals = [];
        for (let i = 0; i < MAX; i++) {
          const n = pl.axis.map((c) => c + NOISE * gauss());
          const len = Math.hypot(n[0], n[1], n[2]);
          pl.normals.push(n.map((c) => c / len));
        }
      });

      box.innerHTML = '';
      const label = document.createElement('p');
      label.className = 'label';
      label.textContent = 'Try it · correspondences per plane';
      box.appendChild(label);

      const controls = document.createElement('div');
      controls.className = 'controls';
      box.appendChild(controls);
      planes.forEach((pl, i) => {
        const row = document.createElement('label');
        const name = document.createElement('span');
        name.className = 'name';
        name.textContent = pl.name;
        const input = document.createElement('input');
        input.type = 'range';
        input.min = '0';
        input.max = String(MAX);
        input.step = '10';
        input.value = String(Math.min(MAX, Math.max(0, init[i] || 0)));
        const out = document.createElement('output');
        row.append(name, input, out);
        controls.appendChild(row);
        pl.input = input;
        pl.output = out;
        input.addEventListener('input', update);
      });

      const bars = document.createElement('div');
      bars.className = 'bars';
      box.appendChild(bars);
      const barEls = ['λ₀', 'λ₁', 'λ₂'].map((sym) => {
        const row = document.createElement('div');
        row.className = 'bar';
        const name = document.createElement('span');
        name.textContent = sym;
        const track = document.createElement('span');
        track.className = 'track';
        const fill = document.createElement('span');
        fill.className = 'fill';
        track.appendChild(fill);
        const val = document.createElement('span');
        row.append(name, track, val);
        bars.appendChild(row);
        return {fill, val};
      });

      const ratioLine = document.createElement('p');
      ratioLine.className = 'verdict';
      ratioLine.setAttribute('aria-live', 'polite');
      box.appendChild(ratioLine);

      function update() {
        const H = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
        planes.forEach((pl) => {
          const count = parseInt(pl.input.value, 10);
          pl.output.textContent = String(count);
          for (let i = 0; i < count; i++) {
            const n = pl.normals[i];
            for (let r = 0; r < 3; r++) {
              for (let c = 0; c < 3; c++) H[r][c] += n[r] * n[c];
            }
          }
        });

        const trace = H[0][0] + H[1][1] + H[2][2];
        if (trace <= 0) {
          barEls.forEach((b) => {
            b.fill.style.width = '0%';
            b.val.textContent = '0.0';
          });
          ratioLine.className = 'verdict bad';
          ratioLine.textContent = 'No correspondences: nothing constrains the translation at all.';
          return;
        }

        const eig = symEig3(H);
        const top = Math.max(eig[2].value, 1e-12);
        eig.forEach((e, i) => {
          const val = Math.max(0, e.value);
          barEls[i].fill.style.width = `${(100 * val / top).toFixed(1)}%`;
          barEls[i].val.textContent = val.toFixed(1);
        });

        const ratio = Math.max(0, eig[0].value) / trace;
        const dir = eig[0].vector.map((c) => c.toFixed(2)).join(', ');
        if (ratio < gate) {
          ratioLine.className = 'verdict bad';
          ratioLine.textContent =
            `λ₀ / trace = ${ratio.toFixed(3)} < ${gate}: degenerate. ` +
            `Translation along ≈ [${dir}] is weakly constrained by these normals (rotation held fixed).`;
        } else {
          ratioLine.className = 'verdict ok';
          ratioLine.textContent =
            `λ₀ / trace = ${ratio.toFixed(3)} ≥ ${gate}: every pure translation direction passes this gate; rotation is held fixed. ` +
            `Weakest direction ≈ [${dir}].`;
        }
      }

      update();
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', setupScatterExplorers);
  } else {
    setupScatterExplorers();
  }
})();
