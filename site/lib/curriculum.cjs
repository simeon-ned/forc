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
const prefix = process.env.COURSE_PATH_PREFIX ?? course.path_prefix;
const refs = yaml.load(read("references/references.yml"));
const url = p => prefix.replace(/\/$/, "") + "/" + p.replace(/^\//, "");
const md = new MarkdownIt({html:true, linkify:true}).use(anchor).use(texmath, {engine:katex, delimiters:"dollars", katexOptions:{throwOnError:true}});
// Reveal interprets nested sections as vertical slides. Math must use div wrappers.
for (const name of ["math_block", "math_block_eqno"]) {
  const renderMath = md.renderer.rules[name];
  if (renderMath) md.renderer.rules[name] = (...args) => renderMath(...args)
    .replace(/<section(?=[ >])/g, "<div").replace(/<\/section>/g, "</div>");
}
// Keep Markdown links readable in the repository and resolve them to site routes.
function render(source) {
  return md.render(source.replace(/\[@([a-z0-9-]+)\]/g, (_, id) => {
    if (!refs[id]) throw new Error("Unknown citation: " + id);
    return "[" + refs[id].short + "](" + url("/references/#" + id) + ")";
  })).replace(/href="(?:\.\.\/)+modules\/([a-z0-9-]+)\/(notes|slides)\.md"/g,
    (_, id, kind) => 'href="' + url('/' + (kind === 'notes' ? 'notes' : 'lectures') + '/' + id + '/') + '"')
    .replace(/href="(?:\.\.\/)+practices\/([a-z0-9-]+)\/README\.md"/g,
      (_, id) => 'href="' + url('/practices/' + id + '/') + '"')
    .replace(/href="(?:\.\.\/)+site\/content\/notation\.md"/g,
      'href="' + url('/notation/') + '"')
    .replaceAll('href="' + course.site_url.replace(/\/$/, '') + '/', 'href="' + url('/'));
}
function renderModule(source, id) {
  return render(source)
    .replace(/(<h[23] id=")([^"]+)"/g,
      (_, opening, id) => opening + md.utils.escapeHtml(decodeURIComponent(id)) + '"')
    .replace(/(src|href)="images\/([^"<>]+)"/g,
    (_, attribute, file) => attribute + '="' + url('/assets/lectures/' + id + '/' + file) + '"');
}
const modules = fs.readdirSync(path.join(root,"modules"), {withFileTypes:true})
 .filter(e => e.isDirectory() && fs.existsSync(path.join(root,"modules",e.name,"module.yml")))
 .map(e => {
   const m = yaml.load(read("modules/" + e.name + "/module.yml"));
   if (m.id !== e.name) throw new Error("Module ID/directory mismatch: " + e.name);
   for (const id of m.references) if (!refs[id]) throw new Error("Unknown reference " + id);
   const notes = renderModule(read("modules/"+e.name+"/notes.md"), e.name);
   const sections = [];
   for (const [,level,id,heading] of notes.matchAll(/<h([23]) id="([^"]+)"[^>]*>(.*?)<\/h[23]>/g)) {
     const entry = {id:md.utils.unescapeAll(id), title:heading.replace(/<[^>]*>/g, ''), children:[]};
     if (level === '2') sections.push(entry);
     else if (sections.length) sections[sections.length - 1].children.push(entry);
   }
   const firstHeading = notes.indexOf('<h2');
   const introduction = firstHeading > 0 ? notes.slice(0, firstHeading) : '';
   const body = firstHeading > 0 ? notes.slice(firstHeading) : notes;
   return {...m, notes, introduction, body, sections,
     slides:read("modules/"+e.name+"/slides.md").split(/\n---\s*\n/).map(source => renderModule(source, e.name)),
     reading:m.references.map(id => ({id,...refs[id]})),
     notes_url:"/notes/"+e.name+"/", slides_url:"/lectures/"+e.name+"/"};
 }).sort((a,b) => a.number-b.number);
const lectureRedirects = modules.flatMap(m => (m.aliases || []).flatMap(alias => [
  {from:'/notes/' + alias + '/', to:m.notes_url, title:m.title},
  {from:'/lectures/' + alias + '/', to:m.slides_url, title:m.title}
]));
// Practice READMEs are also read directly in the repository. Map their shared
// setup links to the site route while keeping notebook downloads relative.
function renderPractice(source) {
  return render(source).replace(/href="\.\.\/README\.md(?:#([^"]+))?"/g, (_, fragment) => {
    const anchor = fragment === "rendering-and-robot-downloads" ? "#rendering-and-verification" : "";
    return 'href="' + url("/setup/") + anchor + '"';
  });
}
const practices = fs.readdirSync(path.join(root,"practices"), {withFileTypes:true})
 .filter(e => e.isDirectory() && fs.existsSync(path.join(root,"practices",e.name,"practice.yml")))
 .map(e => ({...yaml.load(read("practices/"+e.name+"/practice.yml")),
   body:renderPractice(read("practices/"+e.name+"/README.md")), url:"/practices/"+e.name+"/"}));
const assessments = yaml.load(read("assessments/assessments.yml")).map(a => ({...a,body:render(read("assessments/"+a.id+".md"))}));
if (modules.length !== course.lectures) throw new Error("Lecture count differs from course.yml");
for (const m of modules) {
  if (!practices.some(p=>p.id===m.practice)) throw new Error("Missing practice for " + m.id);
  if (!m.outcomes.length || m.slides.length < 4) throw new Error("Incomplete module " + m.id);
}
module.exports = {course,prefix,refs,modules,lectureRedirects,practices,assessments,render};
