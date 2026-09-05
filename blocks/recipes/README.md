# Recipes — composed capabilities that need no new code

A recipe is a YAML subgraph definition: a named, loadable block whose implementation is a
composition of existing blocks with the derived values already worked out. Recipes exist
for the compositions a user would otherwise have to re-derive — a discriminator gain from
a rate and a deviation, a de-emphasis constant, a matched filter's construction — and the
scheduler's chain fusion makes the composed form cost what a hand-fused block would.

## The dialect

Each recipe is one file in the core's YAML-definitions form: a `definition_metadata`
header naming the `block_type`, and a single `SUBGRAPH` block whose `graph` lists the
interior blocks by registry id, their `connections`, and the `exported_ports` that become
the composite's own ports. `index.yaml` in this directory is the loader's catalog; a

## Reaching recipes from code

Nothing recipe-specific is required. A `gr::PluginLoader` given this directory among its
paths reads `index.yaml` and registers every listed recipe; `loader.instantiate(name)`
then returns the composite block exactly as it returns a compiled one, exported ports in
place. Code that creates blocks by registry name — a YAML chain loader, a palette, a
`Graph` builder — consumes recipes without knowing they are YAML.

## The gate

`test/qa_Recipes.cpp` loads this directory through the standard machinery, instantiates
every indexed recipe, and asserts its exported ports. A recipe that does not load and
instantiate does not merge.
