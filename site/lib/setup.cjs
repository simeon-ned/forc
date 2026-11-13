const fs = require("node:fs");
const path = require("node:path");
let c = require("./curriculum.cjs");
module.exports = function(config) {
  config.on("eleventy.before", () => {
    delete require.cache[require.resolve("./curriculum.cjs")];
    c = require("./curriculum.cjs");
  });
  config.addGlobalData("course", () => c.course);
  config.addGlobalData("lessons", () => c.modules);
  config.addGlobalData("lectureRedirects", () => c.lectureRedirects);
  config.addGlobalData("coursePractices", () => c.practices);
  config.addGlobalData("coursePracticePages", () => c.practices.filter(p => !p.custom_page));
  config.addGlobalData("practiceRedirects", () => c.practiceRedirects);
  config.addGlobalData("assessments", () => c.assessments);
  config.addWatchTarget("../assessments");
  config.addGlobalData("references", () => Object.entries(c.refs).map(([id,r])=>({id,...r})));
  config.addFilter("courseMarkdown", source => c.render(source));
  config.setLibrary("md", {render: source => c.render(source)});
  config.addPassthroughCopy({"node_modules/reveal.js/dist":"vendor/reveal"});
  config.addPassthroughCopy({"node_modules/reveal.js/plugin/notes":"vendor/reveal/notes"});
  config.addPassthroughCopy({"node_modules/katex/dist":"vendor/katex"});
  config.addPassthroughCopy({"../CITATION.cff":"CITATION.cff"});
  for (const lesson of c.modules) {
    const images = '../modules/' + lesson.id + '/images';
    if (fs.existsSync(path.resolve(__dirname, '..', images))) {
      config.addPassthroughCopy({[images]:'assets/lectures/' + lesson.id});
    }
  }
  config.addPassthroughCopy({"../practices/p01-simulation/tutorial.ipynb":"practices/p01-simulation/tutorial.ipynb"});
  config.addPassthroughCopy({"../practices/p01-simulation/run.py":"practices/p01-simulation/run.py"});
  config.addPassthroughCopy({"../practices/p03-manipulator/*.ipynb":"practices/p03-manipulator"});
  config.addPassthroughCopy({"../practices/p02-control-basics/tutorial.ipynb":"practices/p02-control-basics/tutorial.ipynb"});
  // Preserve the former notebook download URL as well as the practice page.
  config.addPassthroughCopy({"../practices/p03-manipulator/control_and_linearization.ipynb":"practices/p02-manipulator/control_and_linearization.ipynb"});
  config.addPassthroughCopy({"../course.yml":"downloads/course.yml"});
  config.addPassthroughCopy({"../assessments":"downloads/assessments"});
  for (const item of ["../modules","../practices","../course.yml","../references"]) config.addWatchTarget(item);
  return {prefix:c.prefix, render:source=>c.render(source)};
};
