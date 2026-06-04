# SpringKey

Sparse spring/elastic key generator for 3ds Max transform animation.

## What it does

`SpringKey.ms` opens a compact rollout that creates an After Effects-style
elastic spring or bounce response as keys on selected Position, Rotation, and/or
Scale tracks.

The script estimates incoming velocity at the selected source key, then adds a
damped sine response after that key. It creates only the spring keys it needs,
using a sparse key list instead of sampling every frame.

SpringKey writes transform controller values in local space, so linked objects
keep their animation relative to animated parents.

Internally the file also defines MAXScript scripted controller plugins:

- `SpringKey Position`
- `SpringKey Rotation`
- `SpringKey Rotation Axis`
- `SpringKey Scale`

The current rollout is focused on direct sparse key creation, not on applying
persistent live controllers from the UI.

It also includes a separate Anticipation tool for shaping the start of an
existing movement by creating an opposite-direction key between the selected
start key and the following key.

## Usage

1. Run `SpringKey.ms` in 3ds Max.
2. Select one or more animated objects.
3. Select the impact key on the enabled transform tracks. If several keys are
   selected, SpringKey uses the last selected key.
4. Choose which tracks to affect: Position, Rotation, and/or Scale.
5. Adjust the Elastic and Generated Keys settings.
6. Click `Create Spring/Bounce Keys`.

SpringKey uses the last selected key as the impact key. The spring or bounce keys
are created after that key, and existing generated keys after that key are
cleared on the enabled tracks before new keys are written. Selected source keys
are not rewritten; generated keys are kept strictly after the selected impact
key.

## Installation Package

The installer package is in `Out/SpringKey`:

- `SpringKey.mzp`: drag this file into the 3ds Max viewport, or run it through
  `Scripting -> Run Script`.
- `SpringKey.zip`: distributable archive containing `SpringKey.mzp` and
  `Install.txt`.

The installer copies `SpringKey.ms` to
`scripts/TannenspielScripts/SpringKey.ms`, copies the icon bitmaps to
`usericons`, and registers a `SpringKey` macro in the `TannenspielScripts`
category.

## Rollout controls

### Tracks

- `Position`: create keys on the selected object's Position track.
- `Rotation`: create keys on the selected object's Rotation track.
- `Scale`: create keys on the selected object's Scale track. This is off by
  default.
- `Allow negative scale`: lets generated Scale keys go below `0`. Disabled by
  default to avoid accidental mirrored/inverted scale. Affects both
  Spring/Bounce key creation and Anticipation.

### Elastic

- `Amplitude`: AE-style spring amplitude. Internally divided by `200`.
- `Frequency`: AE-style base elastic frequency. Internally divided by `30`.
  SpringKey then applies damping, so higher `Decay` lowers the effective spring
  frequency.
- `Decay`: AE-style spring decay. Internally divided by `10`. Higher values
  reduce amplitude sooner, shorten the tail, lower the effective spring
  frequency, and make Bounce intervals compress faster.
- `Velocity sample (frames)`: how far before the impact key SpringKey samples
  the original animation to estimate incoming velocity. The default is `0.1`
  frames.
- `Match incoming speed`: scales the spring response so it follows the incoming
  velocity more closely. Enabled by default. For spring keys, generated peak
  keys use Slow tangents. SpringKey does not create intermediate zero-crossing
  keys for spring tails; it keeps the curve clean and writes one final settle
  key at the impact value.
- `Bounce`: uses a one-sided bounce response opposite the incoming motion
  instead of a two-sided spring oscillation. It uses the same Amplitude,
  Frequency, Decay, Velocity sample, and Match incoming speed settings. Bounce
  contact/return intervals get shorter over time. For bounce keys, generated
  peak keys use Slow tangents and generated main keys use Fast tangents.
- `Auto spring impact tangent`: spring only. When enabled, the selected impact
  key is set to Auto tangents so the generated spring tail leaves the impact key
  smoothly. Disabled by default to preserve the original source key.

### Generated Keys

- `Key step`: quantization step, in frames, for generated keys. The default
  is `1.0`. This also snaps the Anticipation key timing.
- `Stop threshold`: response threshold used to end the spring tail once the
  damped motion becomes small enough. SpringKey fades the final tail section to
  a settle key with the same value as the selected impact key. Lower values
  keep more tail keys.
- `Create Spring/Bounce Keys`: creates the spring or bounce keys on the selected
  nodes and enabled tracks.
- `Select an impact key.`: reminder label. Key creation requires at least one
  selected key on the enabled tracks.

### Anticipation

- `Amount %`: how far the anticipation key moves opposite the main motion,
  measured as a percentage of the value difference between the start key and the
  following key. The default is `25`.
- `Timing %`: where the anticipation key is placed between the start key and the
  following key. The default is `35`, meaning 35 percent of the time after the
  start key. The final time is snapped by `Key step`; if snapping would land on
  the start key or following key, SpringKey keeps the new key inside that
  interval.
- `Create Anticipation Key`: creates an opposite-direction key at the start of
  an existing movement. Select the first key of the movement explicitly.
  SpringKey finds the next key on the same enabled track, computes the movement
  vector from the start key to that following key, and creates a new key between
  them in the opposite direction. The start key and the new anticipation key are
  set to Auto tangents.
- `Select a start key.`: reminder label. The Anticipation tool requires a
  selected start key with a following key on the enabled tracks. It never runs
  without an explicitly selected key.

### Debug

- `Debug Selected`: prints diagnostic information for each selected node to the
  MAXScript listener.

## Notes

- Rotation key creation currently expects an `Euler_XYZ` rotation controller when it
  needs to preserve spins greater than 360 degrees.
- Position and Scale are written to their local transform controllers, not to
  world-space object properties.
- Logging is currently disabled in the UI. The logging functions remain in the
  source as commented-out debug support.
