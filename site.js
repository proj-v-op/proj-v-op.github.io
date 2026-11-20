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
