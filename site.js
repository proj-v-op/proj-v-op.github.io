/* Code viewer: <div class="codebox" data-src="code/file.py" data-lang="python"> */
document.querySelectorAll(".codebox[data-src]").forEach(async box=>{
  const src=box.dataset.src, lang=box.dataset.lang||"python";
  const repo=document.body.dataset.repo||"";            // set data-repo on <body> to link to GitHub
  box.innerHTML=`<div class="bar"><span>${src}</span><span>${repo?`<a href="${repo}/blob/main/${src}">view on GitHub</a>`:""}</span></div><pre><code class="language-${lang}">Loading…</code></pre>`;
  const code=box.querySelector("code");
  try{const r=await fetch(src);if(!r.ok)throw 0;code.textContent=await r.text();if(window.hljs)hljs.highlightElement(code);}
  catch(e){code.textContent="# "+src+" not added yet — drop the file into /code and refresh.";}
});
/* Lightbox for gallery images */
const lb=document.createElement("div");lb.className="lb";lb.innerHTML="<img>";document.body.appendChild(lb);
document.querySelectorAll(".gallery img").forEach(i=>i.addEventListener("click",()=>{lb.querySelector("img").src=i.src;lb.classList.add("on")}));
lb.addEventListener("click",()=>lb.classList.remove("on"));
document.addEventListener("keydown",e=>{if(e.key==="Escape")lb.classList.remove("on")});

/* ---------- CAD part catalog ----------
   Data: cad/parts.json.  Files for a part live in cad/parts/<id>/.
   - Catalog page:  <div id="part-grid"></div>
   - Part page:     projects/part.html?id=<id>
   - Link from any page:  <a data-part="<id>">text (optional)</a>  -> resolves to the part page */
(function(){
  const ROOT = new URL(".", document.currentScript ? document.currentScript.src : location.href);
  const url = p => new URL(p, ROOT).href;
  const esc = s => String(s==null?"":s).replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
  const partHref = id => url("projects/part.html?id="+encodeURIComponent(id));
  const load = () => fetch(url("cad/parts.json")).then(r=>r.json()).then(j=>j.parts||[]).catch(()=>[]);

  document.querySelectorAll("a[data-part]").forEach(a=>{
    a.href = partHref(a.dataset.part);
    if(!a.textContent.trim()) load().then(ps=>{const p=ps.find(x=>x.id===a.dataset.part); a.textContent = p?p.name:a.dataset.part;});
  });

  const grid = document.getElementById("part-grid");
  if(grid) load().then(ps=>{
    if(!ps.length){grid.innerHTML='<p class="note">No parts yet.</p>';return;}
    grid.innerHTML = ps.map(p=>`<a class="card" href="${partHref(p.id)}"><div class="thumb">${p.hero?`<img src="${url(p.hero)}" alt="${esc(p.name)}">`:"Renders coming"}</div><div class="body"><span class="status">${esc(p.status||"")}</span><h3>${esc(p.name)}</h3><p>${esc(p.summary)}</p>${(p.tags||[]).map(t=>`<span class="tag">${esc(t)}</span>`).join("")}</div></a>`).join("");
  });

  const box = document.getElementById("part-page");
  if(box) load().then(ps=>{
    const id = new URLSearchParams(location.search).get("id");
    const p = ps.find(x=>x.id===id);
    if(!p){box.innerHTML='<p class="note">Part not found. <a href="cad.html">Back to all parts</a></p>';return;}
    document.title = p.name+" — Owen Pritchard";
    const sec=(h,b)=>b?`<h2>${h}</h2>${b}`:"";
    const specs = Object.entries(p.specs||{});
    const fileIcon = f => /\.(sldprt|sldasm|slddrw)$/i.test(f)?"SolidWorks":/\.(step|stp)$/i.test(f)?"STEP":/\.stl$/i.test(f)?"STL":/\.pdf$/i.test(f)?"PDF":"File";
    box.innerHTML = `
      <p><a href="cad.html">← All CAD parts</a></p>
      <div class="page-head" style="padding-top:12px"><span class="status">${esc(p.status||"")}</span><h1>${esc(p.name)}</h1><p>${esc(p.summary)}</p>${(p.tags||[]).map(t=>`<span class="tag">${esc(t)}</span>`).join("")}</div>
      ${p.hero?`<figure><img src="${url(p.hero)}" alt="${esc(p.name)}"></figure>`:""}
      ${sec("Design intent", p.description?`<p>${esc(p.description)}</p>`:"")}
      ${sec("Specifications", specs.length?`<table>${specs.map(([k,v])=>`<tr><th>${esc(k)}</th><td>${esc(v)}</td></tr>`).join("")}</table>`:"")}
      ${sec("Renders and photos", (p.images||[]).length?`<div class="gallery">${p.images.map(i=>`<img src="${url(i.file||i)}" alt="${esc(i.caption||p.name)}">`).join("")}</div>`:"")}
      ${sec("Drawings", (p.drawings||[]).map(d=>`<h3>${esc(d.title||"Drawing")}</h3><div class="embed" style="aspect-ratio:4/3"><iframe src="${url(d.file)}" title="${esc(d.title||"Drawing")}" loading="lazy"></iframe></div><p><a href="${url(d.file)}">Open PDF</a></p>`).join(""))}
      ${sec("CAD and print files", (p.files||[]).length?`<table><tr><th>File</th><th>Type</th></tr>${p.files.map(f=>`<tr><td><a href="${url(f.file)}" download>${esc(f.label||f.file.split("/").pop())}</a></td><td>${fileIcon(f.file)}</td></tr>`).join("")}</table>`:"")}
      ${sec("Used in", (p.usedIn||[]).length?`<ul>${p.usedIn.map(u=>`<li><a href="${url(u.href)}">${esc(u.label)}</a></li>`).join("")}</ul>`:"")}
      ${!(p.images||[]).length&&!(p.drawings||[]).length&&!(p.files||[]).length?'<p class="note">Renders, drawings and files for this part are coming.</p>':""}`;
    box.querySelectorAll(".gallery img").forEach(i=>i.addEventListener("click",()=>{lb.querySelector("img").src=i.src;lb.classList.add("on")}));
  });
})();
