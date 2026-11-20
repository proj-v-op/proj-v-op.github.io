# CAD parts

One folder per part under `cad/parts/<part-id>/`, plus one entry per part in `cad/parts.json`.
The catalog (`projects/cad.html`) and each part's product page (`projects/part.html?id=<part-id>`) are generated from that JSON.

## Folder layout for a part
```
cad/parts/<part-id>/
  renders/    PNG/JPG renders and photos
  drawings/   dimensioned drawings (PDF)
  solidworks/ .SLDPRT / .SLDASM / .SLDDRW
  print/      .STEP / .STL
```

## Entry in parts.json
```json
{
  "id": "part-id",
  "name": "Part name",
  "summary": "One line.",
  "status": "finished",
  "tags": ["SolidWorks", "3D printing"],
  "hero": "cad/parts/part-id/renders/hero.png",
  "description": "Design intent: what it does and why it looks the way it does.",
  "specs": { "Material": "PETG", "Mass": "12 g", "Process": "FDM" },
  "images":   [ { "file": "cad/parts/part-id/renders/iso.png", "caption": "Isometric" } ],
  "drawings": [ { "title": "Part drawing", "file": "cad/parts/part-id/drawings/part.pdf" } ],
  "files":    [ { "label": "Part (SolidWorks)", "file": "cad/parts/part-id/solidworks/part.SLDPRT" } ],
  "usedIn":   [ { "label": "5-inch build", "href": "projects/drone.html" } ]
}
```

## Linking from other pages
`<a data-part="part-id"></a>` becomes a link to that part's page (the link text defaults to the part name).
