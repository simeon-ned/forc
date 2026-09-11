const fs = require("node:fs");
const path = require("node:path");
const yaml = require("js-yaml");
const MarkdownIt = require("markdown-it");
const anchor = require("markdown-it-anchor");
const texmath = require("markdown-it-texmath");
const katex = require("katex");
const root = path.resolve(__dirname, "../..");
const read = p => fs.readFileSync(path.join(root, p), "utf8");
const course = yaml.load(read("course.yml"));
const prefix = process.env.COURSE_PATH_PREFIX ?? process.env.FORC_PATH_PREFIX ?? course.path_prefix;
const refs = yaml.load(read("references/references.yml"));
const url = p => prefix.replace(/\/$/, "") + "/" + p.replace(/^\//, "");
const md = new MarkdownIt({html:true, linkify:true}).use(anchor).use(texmath, {engine:katex, delimiters:"dollars", katexOptions:{throwOnError:true}});
// Reveal interprets nested sections as vertical slides. Math must use div wrappers.
for (const name of ["math_block", "math_block_eqno"]) {
  const renderMath = md.renderer.rules[name];
  if (renderMath) md.renderer.rules[name] = (...args) => renderMath(...args)
    .replace(/<section(?=[ >])/g, "<div").replace(/<\/section>/g, "</div>");
}
function render(source) {
  return md.render(source.replace(/\[@([a-z0-9-]+)\]/g, (_, id) => {
    if (!refs[id]) throw new Error("Unknown citation: " + id);
    return "[" + refs[id].short + "](" + url("/references/#" + id) + ")";
  }));
}
const modules = fs.readdirSync(path.join(root,"modules"), {withFileTypes:true})
 .filter(e => e.isDirectory() && fs.existsSync(path.join(root,"modules",e.name,"module.yml")))
 .map(e => {
   const m = yaml.load(read("modules/" + e.name + "/module.yml"));
   if (m.id !== e.name) throw new Error("Module ID/directory mismatch: " + e.name);
   for (const id of m.references) if (!refs[id]) throw new Error("Unknown reference " + id);
   return {...m, notes:render(read("modules/"+e.name+"/notes.md")),
     slides:read("modules/"+e.name+"/slides.md").split(/\n---\s*\n/).map(render),
     reading:m.references.map(id => ({id,...refs[id]})),
     notes_url:"/notes/"+e.name+"/", slides_url:"/lectures/"+e.name+"/"};
 }).sort((a,b) => a.number-b.number);
const practices = fs.readdirSync(path.join(root,"practices"), {withFileTypes:true})
 .filter(e => e.isDirectory() && fs.existsSync(path.join(root,"practices",e.name,"practice.yml")))
 .map(e => ({...yaml.load(read("practices/"+e.name+"/practice.yml")),
   body:render(read("practices/"+e.name+"/README.md")), url:"/practices/"+e.name+"/"}));
const assessments = yaml.load(read("assessments/assessments.yml")).map(a => ({...a,body:render(read("assessments/"+a.id+".md"))}));
if (modules.length !== course.lectures) throw new Error("Lecture count differs from course.yml");
for (const m of modules) {
  if (!practices.some(p=>p.id===m.practice)) throw new Error("Missing practice for " + m.id);
  if (!m.outcomes.length || m.slides.length < 4) throw new Error("Incomplete module " + m.id);
}
module.exports = {course,prefix,refs,modules,practices,assessments,render};
