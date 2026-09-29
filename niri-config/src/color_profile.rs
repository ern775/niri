//! Per-output color profile configuration.
//!
//! Kept separate from `output` so that it can live in its own included file (which a settings
//! GUI can rewrite), since `output` sections are not merged across includes.

use crate::utils::FloatOrInt;

/// Color management for one output.
///
/// ```kdl
/// color-profile "eDP-1" {
///     source "icc"
///     icc "~/.local/share/icc/panel.icm"
///     hdr-icc "~/.local/share/icc/panel-hdr.icm"
///     sdr-color-intensity 0
/// }
/// ```
#[derive(knuffel::Decode, Debug, Clone, PartialEq)]
pub struct ColorProfile {
    /// Output this applies to, matched like the name of an `output` section.
    #[knuffel(argument)]
    pub output: String,
    /// Where the SDR display profile comes from.
    #[knuffel(child, unwrap(argument, str), default)]
    pub source: ColorProfileSource,
    /// ICC profile describing the display in SDR mode, used with `source "icc"`.
    #[knuffel(child, unwrap(argument))]
    pub icc: Option<String>,
    /// ICC profile applied while the output is in HDR mode.
    #[knuffel(child, unwrap(argument))]
    pub hdr_icc: Option<String>,
    /// How far sRGB content is stretched toward the display's native gamut, in percent: 0 shows
    /// sRGB accurately, 100 uses the full native gamut (like no color management on a
    /// wide-gamut panel).
    #[knuffel(child, unwrap(argument))]
    pub sdr_color_intensity: Option<FloatOrInt<0, 100>>,
}

/// Source of an output's SDR display profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ColorProfileSource {
    /// Assume the display is sRGB: no color correction.
    #[default]
    Srgb,
    /// Use the ICC profile from `icc`.
    Icc,
    /// Build a profile from the primaries and gamma in the display's EDID.
    Edid,
}

impl std::str::FromStr for ColorProfileSource {
    type Err = miette::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "srgb" => Ok(Self::Srgb),
            "icc" => Ok(Self::Icc),
            "edid" => Ok(Self::Edid),
            _ => Err(miette::miette!(
                r#"invalid color profile source, can be "srgb", "icc" or "edid""#
            )),
        }
    }
}
