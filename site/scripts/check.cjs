const fs=require("node:fs"),path=require("node:path");
const c=require("../lib/curriculum.cjs");
const root=path.resolve(__dirname,"../_site");
const pages=[
 "experiments/go2/index.html","index.html","notation/index.html","setup/index.html","syllabus/index.html",
 "sources/index.html","references/index.html","cite/index.html","assessments/index.html",
 ...c.modules.flatMap(m=>["notes/"+m.id+"/index.html","lectures/"+m.id+"/index.html"]),
 ...c.practiceRedirects.map(r=>r.from.slice(1)+"index.html"),
 ...c.lectureRedirects.map(r=>r.from.slice(1)+"index.html"),
 ...c.practices.map(p=>"practices/"+p.id+"/index.html"),
 ...c.assessments.map(a=>"assessments/"+a.id+"/index.html")
];
let errors=[];
if(c.practices.length!==8 || c.practices.some((p,i)=>Number(p.id.slice(1,3))!==i+1))errors.push("Practice numbering must run from 01 to 08");
if(c.practices[1].id!=="p02-control-basics" || c.practices[2].id!=="p03-manipulator")errors.push("Control basics must precede manipulator control");
if(c.modules.some((m,i)=>m.number!==i+1))errors.push("Lecture numbering must be consecutive and unique");
if(c.modules[0].id!=="01-modeling")errors.push("Modeling must be the first lecture");
for(const r of [...c.lectureRedirects, ...c.practiceRedirects]) {
 const file=path.join(root,r.from,"index.html");
 if(!fs.existsSync(file)||!fs.readFileSync(file,"utf8").includes('href="'+c.prefix+r.to+'"'))
  errors.push("Missing redirect: "+r.from);
}
for(const m of c.modules) {
 const deck=fs.readFileSync(path.join(root,"lectures",m.id,"index.html"),"utf8");
 if((deck.match(/<section(?: |>|\n)/g)||[]).length!==m.slides.length+2)
   errors.push("Unexpected nested slide sections: "+m.id);
}
const sample=fs.readFileSync(path.join(root,"index.html"),"utf8");
if(!sample.includes(c.course.title))errors.push("Title page missing course title");
if(!sample.includes(">Start lecture 01<"))errors.push("Title page missing first-lecture link");
const prefix=(sample.match(/href="([^"]*)\/css\/curriculum\.css"/)||[])[1]||"";
const redirect=path.join(root,"course/index.html");
if(!fs.existsSync(redirect)||!fs.readFileSync(redirect,"utf8").includes('href="'+(prefix||"")+'/'))
  errors.push("Missing /course/ redirect to the title page");
for(const file of pages) {
 const full=path.join(root,file);
 if(!fs.existsSync(full)){errors.push("Missing page: "+file);continue;}
 const html=fs.readFileSync(full,"utf8");
 if(!html.includes("<!doctype html>")&&!html.includes("<!DOCTYPE html>"))errors.push("Missing doctype: "+file);
 if(/katex-error|KaTeX parse error|ParseError|\[@[\w-]+\]|{{|{%/.test(html))errors.push("Unrendered content or math error: "+file);
 const base=new URL((prefix?prefix:"")+"/"+file,"https://course.invalid");
 for(const match of html.matchAll(/(?:href|src)="([^"]+)"/g)) {
  const href=match[1].replace(/&amp;/g,"&");
  if(/^(https?:|mailto:|data:|javascript:|\/\/)/.test(href))continue;
  const url=new URL(href,base);
  if(url.origin!==base.origin)continue;
  let local=decodeURIComponent(url.pathname);
  if(prefix && local.startsWith(prefix+"/"))local=local.slice(prefix.length);
  let target=path.join(root,local);
  if(fs.existsSync(target)&&fs.statSync(target).isDirectory())target=path.join(target,"index.html");
  if(!fs.existsSync(target)){errors.push(file+" -> missing "+href);continue;}
  if(url.hash && !url.hash.startsWith("#/") && target.endsWith(".html")){
   const id=decodeURIComponent(url.hash.slice(1));
   const body=fs.readFileSync(target,"utf8");
   if(!body.includes('id="'+id+'"') && !body.includes('id="'+url.hash.slice(1)+'"'))errors.push(file+" -> missing fragment "+href);
  }
 }
}
if(errors.length){console.error([...new Set(errors)].join("\n"));process.exit(1);}
console.log("Checked "+pages.length+" current pages, local assets, citations, and internal links. Prefix: "+(prefix||"/"));
