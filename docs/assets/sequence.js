// glass-lio course, lesson 9: rebuild the tight solve's steps in order. Loaded after course.js.
(function () {
  'use strict';

  function shuffle(items) {
    for (let i = items.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [items[i], items[j]] = [items[j], items[i]];
    }
    return items;
  }

  // ---- Sequence task: steps are shuffled, the reader taps them in order.
  function setupSequences() {
    document.querySelectorAll('.sequence-task').forEach((task) => {
      const list = task.querySelector('ol.steps');
      const explain = task.querySelector('.explain');
      if (!list) return;
      const items = Array.from(list.querySelectorAll('li[data-step]'));

      items.forEach((li) => {
        const btn = document.createElement('button');
        btn.type = 'button';
        while (li.firstChild) btn.appendChild(li.firstChild);
        li.appendChild(btn);
      });

      const hint = document.createElement('p');
      hint.className = 'hint';
      hint.setAttribute('aria-live', 'polite');
      task.insertBefore(hint, list);

      const reset = document.createElement('button');
      reset.type = 'button';
      reset.className = 'action secondary';
      reset.textContent = 'Shuffle and start over';
      list.after(reset);

      let next = 1;
      function start() {
        next = 1;
        shuffle(items).forEach((li) => {
          const btn = li.querySelector('button');
          btn.removeAttribute('data-pick');
          btn.classList.remove('correct', 'wrong');
          btn.disabled = false;
          list.appendChild(li);
        });
        if (explain) explain.hidden = true;
        hint.textContent = `Tap the steps in the order they happen (${items.length} steps).`;
      }

      items.forEach((li) => {
        const btn = li.querySelector('button');
        btn.addEventListener('click', () => {
          btn.dataset.pick = String(next);
          btn.disabled = true;
          next += 1;
          if (next <= items.length) return;
          let right = 0;
          items.forEach((it) => {
            const b = it.querySelector('button');
            const ok = b.dataset.pick === it.dataset.step;
            b.classList.add(ok ? 'correct' : 'wrong');
            if (ok) right += 1;
          });
          hint.textContent = right === items.length ?
            'All in order.' :
            `${right} of ${items.length} in the right place. The number shows your order; red ones are misplaced.`;
          if (explain) explain.hidden = false;
        });
      });

      reset.addEventListener('click', start);
      start();
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', setupSequences);
  } else {
    setupSequences();
  }
})();
