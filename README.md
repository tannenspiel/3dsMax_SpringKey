# SpringKey

Sparse spring/elastic key baker for 3ds Max transform animation.

## What it does

`SpringKey.ms` opens a compact rollout that bakes an After Effects-style elastic
spring or bounce response onto selected Position, Rotation, and/or Scale tracks.

The script estimates incoming velocity at the selected source key, then adds a
damped sine response after that key. It creates only the spring keys it needs,
using a sparse key list instead of baking every frame.

SpringKey bakes transform controller values in local space, so linked objects
keep their animation relative to animated parents.

Internally the file also defines MAXScript scripted controller plugins:

- `SpringKey Position`
- `SpringKey Rotation`
- `SpringKey Rotation Axis`
- `SpringKey Scale`

The current rollout is focused on sparse baking, not on applying persistent live
controllers from the UI.

## Usage

1. Run `SpringKey.ms` in 3ds Max.
2. Select one or more animated objects.
3. Select at least two keys on the enabled transform tracks.
4. Choose which tracks to bake: Position, Rotation, and/or Scale.
5. Adjust the Elastic and Sparse Bake settings.
6. Click `Bake Spring/Bounce To Keys`.

SpringKey uses the selected key range as the source interval. The spring is baked
after the selected range, and existing baked keys after that range are cleared on
the enabled tracks before new keys are written. Selected source keys are not
rewritten; generated bake keys are kept strictly after the last selected key.

## Rollout controls

### Tracks

- `Position`: include the selected object's Position track in the bake.
- `Rotation`: include the selected object's Rotation track in the bake.
- `Scale`: include the selected object's Scale track in the bake. This is off by
  default.

### Elastic

- `Amplitude`: AE-style spring amplitude. Internally divided by `200`.
- `Frequency`: AE-style spring frequency. Internally divided by `30`.
- `Decay`: AE-style spring decay. Internally divided by `10`.
- `Velocity sample (frames)`: how far before the impact key SpringKey samples
  the original animation to estimate incoming velocity. The default is `0.1`
  frames.
- `Match incoming speed`: scales the spring response so it follows the incoming
  velocity more closely. Enabled by default.
- `Bounce`: uses a one-sided bounce response opposite the incoming motion
  instead of a two-sided spring oscillation. It uses the same Amplitude,
  Frequency, Decay, Velocity sample, and Match incoming speed settings. Generated
  bounce contact keys use linear tangents so contacts do not auto-smooth.

### Sparse Bake

- `Key step`: quantization step, in frames, for generated bake keys. The default
  is `1.0`.
- `Stop threshold`: response threshold used to end the spring tail once the
  damped motion becomes small enough. SpringKey fades the final tail section to
  a settle key with the same value as the last selected source key. Lower values
  keep more tail keys.
- `Bake Spring/Bounce To Keys`: bakes the spring or bounce result to keys on the
  selected nodes and enabled tracks.
- `Select at least two keys.`: reminder label. The bake requires a selected key
  range on the enabled tracks.

### Debug

- `Debug Selected`: prints diagnostic information for each selected node to the
  MAXScript listener. If logging is enabled, it also writes debug entries.

### Logging

- `Enable Log`: toggles writing diagnostic and bake information to
  `SpringKey_debug.log`.
- `Clear Log File`: recreates `SpringKey_debug.log` and writes a fresh header.
- `SpringKey_debug.log`: log file name. The log is written next to the loaded
  `SpringKey.ms` file.

## Notes

- Rotation baking currently expects an `Euler_XYZ` rotation controller when it
  needs to preserve spins greater than 360 degrees.
- Position and Scale are written to their local transform controllers, not to
  world-space object properties.
- `SpringKey_debug.log` is generated only when logging is enabled or cleared.
