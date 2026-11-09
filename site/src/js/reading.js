// Keep navigation available without letting it dominate a narrow reading viewport.
const narrowScreen = window.matchMedia('(max-width: 820px)');
function arrangeNavigation() {
  document.querySelectorAll('[data-collapse-mobile]').forEach(panel => {
    panel.open = !narrowScreen.matches;
  });
}
arrangeNavigation();
narrowScreen.addEventListener('change', arrangeNavigation);

// Wide equations remain keyboard-scrollable instead of shrinking their notation.
function markWideEquations() {
  document.querySelectorAll('.lecture-body .katex-display').forEach(equation => {
    const wide = equation.scrollWidth > equation.clientWidth + 1;
    equation.classList.toggle('equation-scroll', wide);
    let hint = equation.nextElementSibling;
    if (!hint?.classList.contains('equation-hint')) hint = null;
    if (wide) {
      if (!hint) {
        hint = document.createElement('span');
        hint.className = 'equation-hint';
        hint.textContent = 'Scroll equation horizontally →';
        equation.insertAdjacentElement('afterend', hint);
      }
      equation.tabIndex = 0;
      equation.setAttribute('role', 'region');
      equation.setAttribute('aria-label', 'Equation: scroll horizontally to read all terms');
    } else {
      hint?.remove();
      equation.removeAttribute('tabindex');
      equation.removeAttribute('role');
      equation.removeAttribute('aria-label');
    }
  });
}
document.fonts.ready.then(markWideEquations);
window.addEventListener('resize', markWideEquations);
