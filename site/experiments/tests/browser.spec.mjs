import { test, expect } from '@playwright/test';
import fs from 'node:fs/promises';

test('Go2 loads on demand, plots physics, compares runs and exports meaningful samples', async ({ page, context }, testInfo) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  const modelRequests = [];
  context.on('request', request => { if (request.url().includes('raw.githubusercontent.com')) modelRequests.push(request.url()); });
  await page.goto('experiments/go2/?lesson=control');
  await expect(page.locator('#lesson-title')).toHaveText('Go2 · joint feedback');
  await expect(page.locator('#controller-panel')).toBeVisible();
  expect(modelRequests).toHaveLength(0);
  await page.locator('#load').click();
  await expect(page.locator('#play')).toBeEnabled({ timeout: 90000 });
  await expect(page.locator('#model-stats')).toContainText('nq 19 · nv 18 · nu 12');
  await expect(page.locator('#joint option')).toHaveCount(12);
  await expect(page.locator('#target-joint')).toHaveValue('1');
  await expect(page.locator('#viewport canvas')).toBeVisible();
  await page.screenshot({ path: testInfo.outputPath('go2-desktop.png'), fullPage: true });
  await page.locator('#step').click();
  await expect(page.locator('#time')).toHaveText('0.002 s');
  await page.locator('#push').click();
  await page.locator('#play').click();
  await expect.poll(async () => Number((await page.locator('#time').textContent()).split(' ')[0])).toBeGreaterThan(1);
  await page.locator('#play').click();
  await expect(page.locator('#play')).toHaveText('Play');
  const paused = await page.locator('#time').textContent();
  await page.waitForTimeout(150);
  await expect(page.locator('#time')).toHaveText(paused);
  await expect(page.locator('.uplot')).toHaveCount(5);
  const csvDownload = page.waitForEvent('download'); await page.locator('#export').click();
  const csv = await fs.readFile(await (await csvDownload).path(), 'utf8');
  const [header, ...lines] = csv.trim().split('\n');
  expect(header).toContain('FL_thigh_joint_applied_torque_Nm');
  const samples = lines.map(line => line.split(',').map(Number));
  expect(samples.length).toBeGreaterThan(500);
  expect(samples[1][0] - samples[0][0]).toBeCloseTo(0.002, 9);
  expect(samples.some(row => row[2] > 0)).toBeTruthy();
  expect(samples.filter(row => row[4] === 30)).toHaveLength(50);
  expect(samples.every(row => row.every(Number.isFinite))).toBeTruthy();
  await page.locator('#save-run').click();
  await expect(page.locator('#comparison')).toContainText('Run A saved');
  await page.locator('#reset').click(); await expect(page.locator('#time')).toHaveText('0.000 s');
  await page.locator('#kp').fill('80'); await page.locator('#kp').dispatchEvent('input');
  await page.locator('#offset').fill('0.15'); await page.locator('#offset').dispatchEvent('input');
  await page.locator('#joint').selectOption('5');
  await expect(page.locator('#target-joint')).toHaveValue('1'); // Observation cannot change actuation.
  await page.locator('#step').click();
  const configDownload = page.waitForEvent('download'); await page.locator('#export-settings').click();
  const config = JSON.parse(await fs.readFile(await (await configDownload).path(), 'utf8'));
  expect(config.mujoco).toBe('4.0.0'); expect(config.dt).toBe(0.002);
  expect(config.previousRun).toBeTruthy(); expect(config.events.some(e => e.settings?.kp === 80)).toBeTruthy();
  await page.locator('#collision').check();
  await page.screenshot({ path: testInfo.outputPath('go2-collisions.png'), fullPage: true });
  await page.locator('#collision').uncheck();
  await page.locator('#plots').scrollIntoViewIfNeeded();
  await page.screenshot({ path: testInfo.outputPath('go2-plots.png') });
  for (const width of [820, 390, 320]) {
    await page.setViewportSize({ width, height: 900 });
    await expect.poll(() => page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBeTruthy();
    if (width === 390) { await page.evaluate(() => scrollTo(0, 0)); await page.screenshot({ path: testInfo.outputPath('go2-mobile.png'), fullPage: true }); }
  }
  // Same-origin CacheStorage persists between lecture modes; a second load needs no model network requests.
  const priorRequestCount = modelRequests.length;
  await page.goto('experiments/go2/?lesson=models');
  await expect(page.locator('#controller-panel')).toBeHidden();
  await page.locator('#load').click(); await expect(page.locator('#play')).toBeEnabled({ timeout: 30000 });
  expect(modelRequests.length).toBe(priorRequestCount);
  await page.locator('#step').click();
  const passiveDownload = page.waitForEvent('download'); await page.locator('#export').click();
  const passive = (await fs.readFile(await (await passiveDownload).path(), 'utf8')).trim().split('\n')[1].split(',');
  expect(Number(passive[7])).toBe(0); expect(Number(passive[8])).toBe(0); expect(passive[9]).toBe('');
  expect(errors).toEqual([]);
});

test('lecture embeds keep modeling and control separate and respect site routes', async ({ page }) => {
  for (const [lecture, mode] of [['01-modeling', 'models'], ['02-feedback', 'control']]) {
    await page.goto(`notes/${lecture}/`);
    const iframe = page.locator('iframe.lecture-experiment');
    await expect(iframe).toHaveAttribute('src', `../../experiments/go2/?lesson=${mode}`);
    await expect(iframe).toHaveAttribute('loading', 'lazy');
  }
});

test('a failed model download offers an explicit retry instead of broken controls', async ({ page, context }) => {
  await context.route('https://raw.githubusercontent.com/**', route => route.abort());
  await page.goto('experiments/go2/'); await page.locator('#load').click();
  await expect(page.locator('#status')).toContainText('reload the model to retry', { timeout: 15000 });
  await expect(page.locator('#load')).toBeEnabled();
  await expect(page.locator('#play')).toBeDisabled();
  await expect(page.locator('#export')).toBeDisabled();
});
