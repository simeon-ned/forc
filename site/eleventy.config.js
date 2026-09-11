const markdownIt = require("markdown-it");
const markdownItAnchor = require("markdown-it-anchor");

/** @param {import("@11ty/eleventy").UserConfig} eleventyConfig */
module.exports = function (eleventyConfig) {
  const curriculum = require("./lib/setup.cjs")(eleventyConfig);
  eleventyConfig.setServerPassthroughCopyBehavior("passthrough");

  // Project Pages: https://simeon-ned.github.io/courses/forc/
  // Local without prefix: COURSE_PATH_PREFIX= npm run dev
  const pathPrefix = curriculum.prefix;

  const md = markdownIt({
    html: true,
    linkify: true,
    typographer: true,
  }).use(markdownItAnchor, {
    permalink: markdownItAnchor.permalink.headerLink({
      safariReaderFix: true,
    }),
    level: [2, 3],
  });
  eleventyConfig.setLibrary("md", {render: curriculum.render});

  eleventyConfig.addPassthroughCopy({ "src/css": "css" });
  eleventyConfig.addPassthroughCopy({ demos: "demos" });
  eleventyConfig.addPassthroughCopy({ public: "." });

  eleventyConfig.addWatchTarget("src/css/");

  for (const pattern of [
    "**/node_modules/**",
    "**/_site/**",
    "../_legacy/**",
    "**/.git/**",
  ]) {
    eleventyConfig.watchIgnores.add(pattern);
  }

  eleventyConfig.addFilter("tocHeadings", (content) => {
    if (!content) return [];
    const headings = [];
    const re = /<h([23])[^>]*id="([^"]+)"[^>]*>(.*?)<\/h\1>/gi;
    let m;
    while ((m = re.exec(content)) !== null) {
      const text = m[3].replace(/<[^>]+>/g, "").trim();
      headings.push({ level: Number(m[1]), id: m[2], text });
    }
    return headings;
  });

  return {
    pathPrefix,
    dir: {
      input: "content",
      includes: "../src/_includes",
      data: "../src/_data",
      output: "_site",
    },
    htmlTemplateEngine: "njk",
    markdownTemplateEngine: "njk",
    templateFormats: ["njk", "md", "html"],
  };
};
