### Overview

<sup>Since: next release</sup>

`color-profile` sections color-manage an output with a display profile, similar to KDE Plasma's "Color profile" setting.
They are top-level sections, one per output, so they can live in their own included file:

```kdl
include optional=true "~/.config/niri/color-profiles.kdl"
```

```kdl
// ~/.config/niri/color-profiles.kdl
color-profile "eDP-1" {
    source "icc"
    icc "~/.local/share/icc/panel.icm"
    hdr-icc "~/.local/share/icc/panel-hdr.icm"
    sdr-color-intensity 0
}
```

The output name is matched like the name of an `output` section: a connector name, or the make, model and serial.

### `source`

Where the SDR display profile comes from:

- `"srgb"` (default): the display is treated as sRGB, and nothing is changed.
- `"icc"`: the ICC profile in `icc`. Matrix/TRC display profiles are supported, which covers the profiles display vendors and calibration tools produce.
- `"edid"`: a profile built from the primaries, white point and gamma in the display's EDID.

In SDR, content is assumed to be sRGB encoded with gamma 2.2, and is mapped colorimetrically into the display profile.
The mapping runs in the GPU's display color pipeline (`DEGAMMA_LUT` → `CTM` → `GAMMA_LUT`), so it applies to everything on the output, including the cursor and fullscreen direct scanout, without rendering cost.
It needs a driver that exposes those CRTC properties; amdgpu does.

Gamma control clients (night light) keep working: their ramp is applied after the profile's output curve.

### `sdr-color-intensity`

How far sRGB content is stretched toward the display's native gamut, in percent.
`0` shows sRGB colors accurately; `100` uses the full native gamut, which is how a wide-gamut panel looks without color management.
Values in between trade accuracy for more saturated colors, like Plasma's "SDR color intensity".

### `hdr-icc`

A profile used while the output is in HDR (see the `hdr` output option).
In HDR the signal is BT.2020 with the PQ transfer function and the display maps it itself, so the SDR mapping is not applied.
What is applied is the profile's vendor calibration: the regamma curves of a Windows `MHC2` tag, which HDR profiles from display vendors carry.
