(() => {
  const links = [...document.querySelectorAll(".toc a")];
  if (!links.length) return;

  const ids = links.map((a) => a.getAttribute("href").slice(1));
  const heads = ids.map((id) => document.getElementById(id)).filter(Boolean);

  const setActive = () => {
    let current = heads[0];
    const y = window.scrollY + 96;
    for (const h of heads) {
      if (h.offsetTop <= y) current = h;
    }
    links.forEach((a) => {
      a.classList.toggle("active", a.getAttribute("href") === `#${current.id}`);
    });
  };

  setActive();
  window.addEventListener("scroll", setActive, { passive: true });
})();
