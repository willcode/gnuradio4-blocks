# Recipes — composed capabilities that need no new code

A recipe is a YAML subgraph definition: a named, loadable block whose implementation is a
composition of existing blocks with the derived values already worked out. Recipes exist
for the compositions a user would otherwise have to re-derive — a discriminator gain from
a rate and a deviation, a de-emphasis constant, a matched filter's construction — and the
scheduler's chain fusion makes the composed form cost what a hand-fused block would.

## What ships

| Recipe | What it composes | Required parameters |
|---|---|---|
| `gr::recipes::NbfmDemod` | discriminator and de-emphasis, general form | `sample_rate`, `deviation` |
| `gr::recipes::SampleClockOffset` | a resampling by `1 + ppm*1e-6`, tags re-originated | none |
| `gr::recipes::WbfmMonoDemod` | tuner, decimating channel filter, discriminator, audio resampler, de-emphasis | `sample_rate` |

A recipe with no required parameter instantiates bare; the rest name what they are missing.

## The dialect

Each recipe is one file in the core's YAML-definitions form: a `definition_metadata`
header naming the `block_type`, and a single `SUBGRAPH` block whose `graph` lists the
interior blocks by registry id, their `connections`, and the `exported_ports` that become
the composite's own ports. `index.yaml` in this directory is the loader's catalog; a
recipe ships by appearing there, and its `modified` stamp should be bumped with every
edit so the catalog keeps an honest account of each definition's age. A loader reads a
definition that is on disk at every load, so the edit itself reaches the generator and
the tests immediately.

A recipe may declare `exported_parameters` on its `SUBGRAPH`: named, typed values with a
`doc` and an optional `default`, which become the composite's own settings. One without a
default is required, and instantiating the recipe without it is refused by name. An
interior setting written as `=...` is bound to them — a numeric parameter through an
arithmetic expression, so a derivation is written where it can be read and re-derived
whenever a parameter changes; a string- or vector-typed parameter by substitution alone,
`=name` handing the value through as it stands, because there is no arithmetic to do on
one. A literal beginning with `=` is written `\=`.

The derivations themselves belong in the file's header comment: a recipe pins the numbers
*and* shows where they came from, so a reader can build a variant instead of guessing.

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
