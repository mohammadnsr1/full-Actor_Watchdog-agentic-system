# Reference views: fmb_assembly_2

Put pictures of the FINISHED assembly here (any number of views): `*.png`,
`*.jpg` or `*.jpeg`. Both agents get them: the executor's authoring agent via
`get_assembly_views`, the WatchDog via `assembly_reference`.

Optional `views.yaml` with a caption per file and notes about the set:

```yaml
notes: "Finished board, all four parts seated. Photos from the bench, lit from above."
top.png: "From above; the fixture's long edge runs left-right."
front_left.png: "From the front left, about 45 degrees down."
```

Large pictures are downscaled to 1024 px for the agents (originals stay here).
Never put placement poses or frame names with coordinates here: the executor
reads this folder. Placements are WatchDog-only (`config/placements/`).
