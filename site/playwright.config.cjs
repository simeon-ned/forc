const { defineConfig } = require('@playwright/test');
module.exports = defineConfig({
  testDir: './experiments/tests', testMatch: '**/*.spec.mjs', timeout: 120000, workers: 1,
  outputDir: './test-results', reporter: 'list',
  use: { baseURL: process.env.LAB_BASE_URL || 'http://localhost:8080', viewport: { width: 1440, height: 1000 },
    headless: true, launchOptions: { executablePath: process.env.CHROME_EXECUTABLE || undefined,
      args: ['--enable-unsafe-swiftshader', '--disable-dev-shm-usage'] },
  },
  webServer: process.env.LAB_BASE_URL ? undefined : { command: 'COURSE_PATH_PREFIX= npm run dev', url: 'http://localhost:8080/experiments/go2/',
    reuseExistingServer: !process.env.CI, timeout: 30000 },
});
