import {test, expect} from '@playwright/test';

test('control basics provides a self-contained notebook without stored simulation media', async ({page}) => {
  await page.goto('/practices/p02-control-basics/');
  await expect(page.locator('h1')).toHaveText('2. Control basics with MuJoCo');
  const links=page.locator('.course-nav a[href*="/practices/p"]');
  const hrefs=await links.evaluateAll(items=>items.map(a=>a.getAttribute('href')));
  expect(hrefs.slice(0,3)).toEqual(['/practices/p01-simulation/','/practices/p02-control-basics/','/practices/p03-manipulator/']);
  await expect(page.locator('.katex-error')).toHaveCount(0);
  await expect(page.locator('video')).toHaveCount(0);
  await expect(page.locator('.practice-body img')).toHaveCount(5);
  await expect(page.locator('article')).toContainText('media.show_video');
  const response=await page.request.get('/practices/p02-control-basics/tutorial.ipynb');
  expect(response.ok()).toBeTruthy();
  const notebook=await response.json();
  const code=notebook.cells.filter(c=>c.cell_type==='code').map(c=>c.source.join('')).join('\n');
  expect(code).not.toContain('from forc');
  expect(code).toContain('def cart_pole(');
  expect(code).toContain('def linearize(');
  expect(notebook.cells.every(c=>!c.outputs?.length)).toBeTruthy();
  const pictures=await page.locator('.practice-body img').evaluateAll(async images=>{
    await Promise.all(images.map(img=>img.decode()));
    return images.map(img=>img.naturalWidth);
  });
  expect(pictures.every(width=>width>0)).toBeTruthy();
  await page.goto('/practices/p02-manipulator/');
  await expect(page).toHaveURL(/\/practices\/p03-manipulator\/$/);
});

test('control lesson fits a mobile viewport with readable scrollable equations', async ({page}) => {
  await page.setViewportSize({width:390,height:844});
  await page.goto('/practices/p02-control-basics/');
  await page.evaluate(()=>document.fonts.ready);
  const size=await page.evaluate(()=>({content:document.documentElement.scrollWidth,viewport:innerWidth}));
  expect(size.content).toBeLessThanOrEqual(size.viewport);
  await expect(page.locator('[data-collapse-mobile]')).not.toHaveAttribute('open','');
  const toc=page.locator('a[href="#7-cart-pole-stabilization"]');
  await toc.click();
  await expect(page.locator('[id="7-cart-pole-stabilization"]')).toBeInViewport();
});
