// glass-lio course: math rendering and the interactive pieces of each lesson.
// Everything hooks onto markup already in the pages. Lesson-specific widgets live in
// sequence.js (lesson 9) and normal-scatter.js (lesson 7).
(function () {
  'use strict';

  // ---- Math. The KaTeX scripts are `defer`red ahead of this one, so they have already run.
  function renderMath() {
    if (typeof window.renderMathInElement !== 'function') return;
    window.renderMathInElement(document.body, {
      delimiters: [
        {left: '$$', right: '$$', display: true},
        {left: '\\[', right: '\\]', display: true},
        {left: '\\(', right: '\\)', display: false},
      ],
      ignoredTags: ['script', 'noscript', 'style', 'textarea', 'pre', 'code'],
      throwOnError: false,
    });
  }

  // ---- Multiple-choice quizzes, with a per-page score.
  function setupQuizzes() {
    const quizzes = Array.from(document.querySelectorAll('.quiz[data-answer]'));
    const score = document.querySelector('.score');
    let answered = 0;
    let correct = 0;

    function updateScore() {
      if (!score) return;
      score.textContent = answered === 0 ? '' :
        `Quiz score: ${correct} / ${answered} answered correctly (${quizzes.length} on this page)`;
    }
    if (score) score.setAttribute('aria-live', 'polite');

    quizzes.forEach((quiz) => {
      const list = quiz.querySelector('ol.options');
      const items = Array.from(list.children);
      for (let i = items.length - 1; i > 0; i--) {
        const j = Math.floor(Math.random() * (i + 1));
        [items[i], items[j]] = [items[j], items[i]];
      }
      items.forEach((item) => list.appendChild(item));
      const buttons = Array.from(quiz.querySelectorAll('button.option'));
      buttons.forEach((btn) => {
        btn.addEventListener('click', () => {
          const right = btn.dataset.key === quiz.dataset.answer;
          buttons.forEach((b) => {
            b.disabled = true;
            if (b.dataset.key === quiz.dataset.answer) b.classList.add('correct');
          });
          if (!right) btn.classList.add('wrong');
          const explain = quiz.querySelector('.explain');
          if (explain) explain.hidden = false;
          answered += 1;
          if (right) correct += 1;
          updateScore();
        });
      });
    });
  }

  // ---- Free recall: write first, then reveal; checklist items tick on click.
  function setupRecall() {
    document.querySelectorAll('.recall').forEach((box) => {
      const btn = box.querySelector('button.reveal');
      const answer = box.querySelector('.answer');
      if (btn && answer) {
        btn.addEventListener('click', () => {
          answer.hidden = false;
          btn.hidden = true;
        });
      }
      box.querySelectorAll('ul.checklist li').forEach((li) => {
        li.setAttribute('role', 'checkbox');
        li.setAttribute('aria-checked', 'false');
        li.tabIndex = 0;
        const toggle = () => {
          const done = li.classList.toggle('done');
          li.setAttribute('aria-checked', String(done));
        };
        li.addEventListener('click', toggle);
        li.addEventListener('keydown', (e) => {
          if (e.key === ' ' || e.key === 'Enter') {
            e.preventDefault();
            toggle();
          }
        });
      });
    });
  }

  function init() {
    setupQuizzes();
    setupRecall();
    renderMath();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
