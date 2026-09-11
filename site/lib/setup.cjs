let c = require("./curriculum.cjs");
module.exports = function(config) {
  config.on("eleventy.before", () => {
    delete require.cache[require.resolve("./curriculum.cjs")];
    c = require("./curriculum.cjs");
  });
  config.addGlobalData("course", () => c.course);
  config.addGlobalData("lessons", () => c.modules);
  config.addGlobalData("coursePractices", () => c.practices);
  config.addGlobalData("assessments", () => c.assessments);
  config.addWatchTarget("../assessments");
  config.addGlobalData("references", () => Object.entries(c.refs).map(([id,r])=>({id,...r})));
  config.addFilter("courseMarkdown", source => c.render(source));
  config.setLibrary("md", {render: source => c.render(source)});
  config.addPassthroughCopy({"node_modules/reveal.js/dist":"vendor/reveal"});
  config.addPassthroughCopy({"node_modules/reveal.js/plugin/notes":"vendor/reveal/notes"});
  config.addPassthroughCopy({"node_modules/katex/dist":"vendor/katex"});
  config.addPassthroughCopy({"../CITATION.cff":"CITATION.cff"});
  config.addPassthroughCopy({"../course.yml":"downloads/course.yml"});
  config.addPassthroughCopy({"../assessments":"downloads/assessments"});
  for (const item of ["../modules","../practices","../course.yml","../references"]) config.addWatchTarget(item);
  return {prefix:c.prefix, render:source=>c.render(source)};
};
