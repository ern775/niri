//! Display color profiles and the CRTC color pipeline built from them.
//!
//! A display profile describes how the panel turns device RGB into color: a linear
//! RGB -> XYZ matrix plus a per-channel tone response curve (TRC). Profiles come from ICC files
//! (matrix/TRC display profiles, the common kind) or from the EDID's primaries and gamma.
//!
//! In SDR, content is assumed to be sRGB (encoded with gamma 2.2, like KWin assumes) and mapped
//! colorimetrically into the display profile. This runs entirely in the CRTC color pipeline,
//! which is the same "efficiency" offload KWin uses:
//!
//! ```text
//! DEGAMMA_LUT (content EOTF) -> CTM (content RGB -> display RGB) -> GAMMA_LUT (inverse TRC)
//! ```
//!
//! so it applies to everything on the output, including the cursor plane and direct scanout,
//! at no rendering cost. `sdr_color_intensity` interpolates the content primaries from sRGB
//! toward the display's native primaries, like KWin's "SDR color intensity".
//!
//! In HDR the signal is PQ/BT.2020 and the sink maps it, so the SDR mapping is not applied;
//! an HDR profile's vendor calibration curves (the Windows `MHC2` regamma LUT, which HDR
//! profiles from display vendors carry) are applied in the gamma stage instead.

use std::path::Path;

use anyhow::{bail, ensure, Context};

pub type Mat3 = [[f64; 3]; 3];

/// CIE xy chromaticity.
pub type Xy = (f64, f64);

pub const D50: [f64; 3] = [0.9642, 1.0, 0.8249];
pub const D65_XY: Xy = (0.3127, 0.3290);
pub const SRGB_PRIMARIES: [Xy; 3] = [(0.640, 0.330), (0.300, 0.600), (0.150, 0.060)];

/// Gamma that SDR content is assumed to be encoded with.
pub const SDR_CONTENT_GAMMA: f64 = 2.2;

/// A per-channel tone response curve: device value (0..1) -> relative luminance (0..1).
#[derive(Debug, Clone, PartialEq)]
pub enum Trc {
    Gamma(f64),
    /// Uniformly sampled curve.
    Table(Vec<f64>),
    /// ICC `parametricCurveType`, normalized to the most general form (type 4):
    /// `Y = (a*X + b)^g + e` for `X >= d`, `Y = c*X + f` otherwise.
    Parametric {
        g: f64,
        a: f64,
        b: f64,
        c: f64,
        d: f64,
        e: f64,
        f: f64,
    },
}

impl Trc {
    pub fn eval(&self, x: f64) -> f64 {
        let x = x.clamp(0., 1.);
        let y = match self {
            Trc::Gamma(g) => x.powf(*g),
            Trc::Table(t) => sample(t, x),
            Trc::Parametric { g, a, b, c, d, e, f } => {
                if x >= *d {
                    let base = a * x + b;
                    if base <= 0. {
                        *e
                    } else {
                        base.powf(*g) + e
                    }
                } else {
                    c * x + f
                }
            }
        };
        y.clamp(0., 1.)
    }

    /// Device value producing relative luminance `y`. TRCs are non-decreasing, so this is a
    /// bisection; flat stretches resolve to their lowest input.
    pub fn inverse(&self, y: f64) -> f64 {
        let y = y.clamp(0., 1.);
        if let Trc::Gamma(g) = self {
            return y.powf(1. / g);
        }
        let (mut lo, mut hi) = (0f64, 1f64);
        for _ in 0..40 {
            let mid = (lo + hi) / 2.;
            if self.eval(mid) < y {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        hi
    }
}

/// Linear interpolation into a uniformly sampled table covering 0..1.
fn sample(t: &[f64], x: f64) -> f64 {
    match t.len() {
        0 => x,
        1 => t[0],
        n => {
            let pos = x.clamp(0., 1.) * (n - 1) as f64;
            let i = (pos.floor() as usize).min(n - 2);
            let frac = pos - i as f64;
            t[i] + (t[i + 1] - t[i]) * frac
        }
    }
}

/// A display profile: how the panel reproduces device RGB.
#[derive(Debug, Clone, PartialEq)]
pub struct DisplayProfile {
    pub description: String,
    /// Linear device RGB -> XYZ, relative to the display's own white (white maps to Y = 1).
    pub rgb_to_xyz: Mat3,
    /// White point chromaticity.
    pub white: Xy,
    pub trc: [Trc; 3],
    /// Vendor regamma calibration (`MHC2` tag), per channel, applied on the encoded signal.
    pub regamma: Option<[Vec<f64>; 3]>,
}

impl DisplayProfile {
    pub fn load_icc(path: &Path) -> anyhow::Result<Self> {
        let data = std::fs::read(path).with_context(|| format!("error reading {path:?}"))?;
        Self::parse_icc(&data).with_context(|| format!("error parsing ICC profile {path:?}"))
    }

    pub fn parse_icc(data: &[u8]) -> anyhow::Result<Self> {
        let icc = Icc::parse(data)?;
        ensure!(&data[12..16] == b"mntr", "not a display profile");
        ensure!(&data[16..20] == b"RGB ", "not an RGB profile");

        let description = icc.description().unwrap_or_default();

        let wtpt = icc.xyz("wtpt").unwrap_or(D50);
        // Colorants are stored adapted to the D50 PCS. ICC v4 (and some v2 profiles) record the
        // adaptation in `chad`; otherwise v2 profiles use Bradford from the media white.
        let unadapt = match icc.mat("chad") {
            Some(chad) => inv(chad)?,
            None => {
                let white = if icc.has("chad") { D50 } else { wtpt };
                inv(bradford(white, D50))?
            }
        };
        let colorants = ["rXYZ", "gXYZ", "bXYZ"].map(|t| icc.xyz(t));
        let [Some(r), Some(g), Some(b)] = colorants else {
            bail!("only matrix/TRC display profiles are supported (no rXYZ/gXYZ/bXYZ)");
        };
        let [r, g, b] = [r, g, b].map(|c| mul(&unadapt, c));
        let mut m = [
            [r[0], g[0], b[0]],
            [r[1], g[1], b[1]],
            [r[2], g[2], b[2]],
        ];
        // Normalize so that device white has Y = 1.
        let white_y = m[1][0] + m[1][1] + m[1][2];
        ensure!(white_y > 0., "degenerate colorants");
        for row in &mut m {
            for v in row {
                *v /= white_y;
            }
        }
        let white_xyz = mul(&m, [1., 1., 1.]);
        let white = xyz_to_xy(white_xyz);

        let trc = ["rTRC", "gTRC", "bTRC"].map(|t| icc.trc(t));
        let [Some(tr), Some(tg), Some(tb)] = trc else {
            bail!("missing tone response curves");
        };

        let regamma = icc.mhc2_regamma();

        Ok(Self {
            description,
            rgb_to_xyz: m,
            white,
            trc: [tr?, tg?, tb?],
            regamma,
        })
    }

    /// Builds a profile from EDID primaries and gamma. Returns `None` when the EDID has no
    /// usable chromaticity data.
    pub fn from_edid(edid: &[u8]) -> Option<Self> {
        if edid.len() < 128 || edid[0..8] != [0, 255, 255, 255, 255, 255, 255, 0] {
            return None;
        }
        let lo1 = edid[25];
        let lo2 = edid[26];
        let c = |hi: u8, lo: u8| f64::from((u16::from(hi) << 2) | u16::from(lo)) / 1024.;
        let red = (c(edid[27], lo1 >> 6 & 3), c(edid[28], lo1 >> 4 & 3));
        let green = (c(edid[29], lo1 >> 2 & 3), c(edid[30], lo1 & 3));
        let blue = (c(edid[31], lo2 >> 6 & 3), c(edid[32], lo2 >> 4 & 3));
        let white = (c(edid[33], lo2 >> 2 & 3), c(edid[34], lo2 & 3));
        let valid = |(x, y): Xy| x > 0. && y > 0. && x + y < 1.;
        if ![red, green, blue, white].into_iter().all(valid) {
            return None;
        }
        // Gamma byte: (gamma * 100) - 100; 0xFF means "defined elsewhere".
        let gamma = match edid[23] {
            0xff => SDR_CONTENT_GAMMA,
            g => (f64::from(g) + 100.) / 100.,
        };
        let rgb_to_xyz = rgb_to_xyz(&[red, green, blue], white).ok()?;
        Some(Self {
            description: String::from("EDID"),
            rgb_to_xyz,
            white,
            trc: [Trc::Gamma(gamma), Trc::Gamma(gamma), Trc::Gamma(gamma)],
            regamma: None,
        })
    }

    /// The display's native primaries.
    pub fn primaries(&self) -> [Xy; 3] {
        let m = &self.rgb_to_xyz;
        [0, 1, 2].map(|c| xyz_to_xy([m[0][c], m[1][c], m[2][c]]))
    }
}

/// The CRTC color pipeline for one output state, as normalized (0..1) values.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Pipeline {
    pub degamma: Option<Vec<[f64; 3]>>,
    pub ctm: Option<Mat3>,
    /// Per-channel output curve, applied before any gamma-control ramp.
    pub gamma: Option<Vec<[f64; 3]>>,
}

impl Pipeline {
    /// SDR: sRGB content (primaries interpolated toward the display's native primaries by
    /// `intensity`, 0..1) mapped colorimetrically into `display`.
    pub fn sdr(
        display: &DisplayProfile,
        intensity: f64,
        degamma_size: usize,
        gamma_size: usize,
    ) -> anyhow::Result<Self> {
        let content = content_primaries(display, intensity);
        let content_to_xyz = rgb_to_xyz(&content, display.white)?;
        let ctm = mat_mul(&inv(display.rgb_to_xyz)?, &content_to_xyz);

        let degamma = lut(degamma_size, |x| {
            let y = x.powf(SDR_CONTENT_GAMMA);
            [y, y, y]
        });
        let gamma = lut(gamma_size, |x| {
            [0, 1, 2].map(|c| {
                let enc = display.trc[c].inverse(x);
                apply_regamma(display.regamma.as_ref(), c, enc)
            })
        });

        Ok(Self {
            degamma: Some(degamma),
            ctm: Some(ctm),
            gamma: Some(gamma),
        })
    }

    /// HDR: only the profile's vendor regamma calibration, if it has one.
    pub fn hdr(display: &DisplayProfile, lut_size: usize) -> Self {
        let gamma = display.regamma.as_ref().map(|r| {
            lut(lut_size, |x| [0, 1, 2].map(|c| apply_regamma(Some(r), c, x)))
        });
        Self {
            degamma: None,
            ctm: None,
            gamma,
        }
    }

    pub fn is_identity(&self) -> bool {
        self.degamma.is_none() && self.ctm.is_none() && self.gamma.is_none()
    }
}

fn apply_regamma(regamma: Option<&[Vec<f64>; 3]>, channel: usize, x: f64) -> f64 {
    match regamma {
        Some(r) => sample(&r[channel], x).clamp(0., 1.),
        None => x,
    }
}

fn lut(size: usize, f: impl Fn(f64) -> [f64; 3]) -> Vec<[f64; 3]> {
    let n = size.max(2);
    (0..n).map(|i| f(i as f64 / (n - 1) as f64)).collect()
}

/// Content primaries: sRGB moved toward the display's primaries by `intensity` (0..1).
fn content_primaries(display: &DisplayProfile, intensity: f64) -> [Xy; 3] {
    let t = intensity.clamp(0., 1.);
    let native = display.primaries();
    [0, 1, 2].map(|i| {
        let (sx, sy) = SRGB_PRIMARIES[i];
        let (nx, ny) = native[i];
        (sx + (nx - sx) * t, sy + (ny - sy) * t)
    })
}

/// Composes a gamma-control ramp (u16, R then G then B, each `size` long) after `curve`
/// (normalized values), producing the final GAMMA_LUT contents.
pub fn compose_ramp(curve: Option<&[[f64; 3]]>, ramp: Option<&[u16]>, size: usize) -> Vec<[u16; 3]> {
    let ramp_channels = ramp.map(|r| {
        let n = r.len() / 3;
        [0, 1, 2].map(|c| r[c * n..(c + 1) * n].iter().map(|&v| f64::from(v) / 65535.).collect::<Vec<_>>())
    });
    let to_u16 = |v: f64| (v.clamp(0., 1.) * 65535.).round() as u16;
    (0..size)
        .map(|i| {
            let x = i as f64 / (size - 1).max(1) as f64;
            let enc = curve.map_or([x, x, x], |c| {
                let pos = x * (c.len() - 1) as f64;
                let j = (pos.floor() as usize).min(c.len().saturating_sub(2));
                let frac = pos - j as f64;
                [0, 1, 2].map(|ch| c[j][ch] + (c[(j + 1).min(c.len() - 1)][ch] - c[j][ch]) * frac)
            });
            [0, 1, 2].map(|ch| {
                let v = match &ramp_channels {
                    Some(r) => sample(&r[ch], enc[ch]),
                    None => enc[ch],
                };
                to_u16(v)
            })
        })
        .collect()
}

/// Encodes a matrix as a DRM `drm_color_ctm`: row-major S31.32 sign-magnitude.
pub fn ctm_to_drm(m: &Mat3) -> [u64; 9] {
    let enc = |v: f64| {
        let mag = ((v.abs() * 4294967296.).round() as u64) & !(1 << 63);
        if v < 0. {
            mag | (1 << 63)
        } else {
            mag
        }
    };
    [
        enc(m[0][0]),
        enc(m[0][1]),
        enc(m[0][2]),
        enc(m[1][0]),
        enc(m[1][1]),
        enc(m[1][2]),
        enc(m[2][0]),
        enc(m[2][1]),
        enc(m[2][2]),
    ]
}

pub fn to_u16_lut(lut: &[[f64; 3]]) -> Vec<[u16; 3]> {
    lut.iter()
        .map(|v| v.map(|c| (c.clamp(0., 1.) * 65535.).round() as u16))
        .collect()
}

// ---- color math ----

pub fn xyz_to_xy(v: [f64; 3]) -> Xy {
    let s = v[0] + v[1] + v[2];
    if s == 0. {
        return D65_XY;
    }
    (v[0] / s, v[1] / s)
}

fn xy_to_xyz((x, y): Xy) -> [f64; 3] {
    [x / y, 1., (1. - x - y) / y]
}

/// Linear RGB -> XYZ for the given primaries and white (white maps to Y = 1).
pub fn rgb_to_xyz(primaries: &[Xy; 3], white: Xy) -> anyhow::Result<Mat3> {
    let p = primaries.map(xy_to_xyz);
    let m = [
        [p[0][0], p[1][0], p[2][0]],
        [p[0][1], p[1][1], p[2][1]],
        [p[0][2], p[1][2], p[2][2]],
    ];
    let s = mul(&inv(m)?, xy_to_xyz(white));
    Ok([0, 1, 2].map(|r| [0, 1, 2].map(|c| m[r][c] * s[c])))
}

const BRADFORD: Mat3 = [
    [0.8951, 0.2664, -0.1614],
    [-0.7502, 1.7135, 0.0367],
    [0.0389, -0.0685, 1.0296],
];

/// Bradford chromatic adaptation from `src` to `dst` white (XYZ).
pub fn bradford(src: [f64; 3], dst: [f64; 3]) -> Mat3 {
    let s = mul(&BRADFORD, src);
    let d = mul(&BRADFORD, dst);
    let scale = [
        [d[0] / s[0], 0., 0.],
        [0., d[1] / s[1], 0.],
        [0., 0., d[2] / s[2]],
    ];
    // BRADFORD is invertible.
    let inv_b = inv(BRADFORD).unwrap();
    mat_mul(&inv_b, &mat_mul(&scale, &BRADFORD))
}

pub fn mul(m: &Mat3, v: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|r| m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2])
}

pub fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

pub fn inv(m: Mat3) -> anyhow::Result<Mat3> {
    let [[a, b, c], [d, e, f], [g, h, i]] = m;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    ensure!(det.abs() > 1e-12, "singular matrix");
    Ok([
        [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
        [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
        [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
    ])
}

// ---- ICC parsing ----

struct Icc<'a> {
    data: &'a [u8],
    tags: Vec<([u8; 4], usize, usize)>,
}

fn be_u16(d: &[u8], o: usize) -> Option<u16> {
    Some(u16::from_be_bytes(d.get(o..o + 2)?.try_into().ok()?))
}

fn be_u32(d: &[u8], o: usize) -> Option<u32> {
    Some(u32::from_be_bytes(d.get(o..o + 4)?.try_into().ok()?))
}

fn s15f16(d: &[u8], o: usize) -> Option<f64> {
    Some(f64::from(be_u32(d, o)? as i32) / 65536.)
}

impl<'a> Icc<'a> {
    fn parse(data: &'a [u8]) -> anyhow::Result<Self> {
        ensure!(data.len() >= 132, "file too small for an ICC profile");
        ensure!(&data[36..40] == b"acsp", "missing ICC signature");
        let count = be_u32(data, 128).context("truncated tag table")? as usize;
        ensure!(count < 1024, "unreasonable tag count");
        let mut tags = Vec::with_capacity(count);
        for i in 0..count {
            let base = 132 + 12 * i;
            let sig: [u8; 4] = data
                .get(base..base + 4)
                .context("truncated tag table")?
                .try_into()?;
            let offset = be_u32(data, base + 4).context("truncated tag table")? as usize;
            let size = be_u32(data, base + 8).context("truncated tag table")? as usize;
            ensure!(
                offset.checked_add(size).is_some_and(|end| end <= data.len()),
                "tag {:?} out of bounds",
                String::from_utf8_lossy(&sig)
            );
            tags.push((sig, offset, size));
        }
        Ok(Self { data, tags })
    }

    fn has(&self, sig: &str) -> bool {
        self.tags.iter().any(|(s, _, _)| s == sig.as_bytes())
    }

    fn tag(&self, sig: &str) -> Option<&'a [u8]> {
        let (_, off, size) = self.tags.iter().find(|(s, _, _)| s == sig.as_bytes())?;
        Some(&self.data[*off..*off + *size])
    }

    fn xyz(&self, sig: &str) -> Option<[f64; 3]> {
        let t = self.tag(sig)?;
        if &t[..4] != b"XYZ " {
            return None;
        }
        Some([s15f16(t, 8)?, s15f16(t, 12)?, s15f16(t, 16)?])
    }

    fn mat(&self, sig: &str) -> Option<Mat3> {
        let t = self.tag(sig)?;
        if &t[..4] != b"sf32" {
            return None;
        }
        let v = |i: usize| s15f16(t, 8 + 4 * i);
        Some([
            [v(0)?, v(1)?, v(2)?],
            [v(3)?, v(4)?, v(5)?],
            [v(6)?, v(7)?, v(8)?],
        ])
    }

    fn trc(&self, sig: &str) -> Option<anyhow::Result<Trc>> {
        let t = self.tag(sig)?;
        Some(parse_trc(t))
    }

    fn description(&self) -> Option<String> {
        let t = self.tag("desc")?;
        match &t[..4] {
            b"desc" => {
                let len = be_u32(t, 8)? as usize;
                let s = t.get(12..12 + len)?;
                Some(String::from_utf8_lossy(s).trim_end_matches('\0').to_owned())
            }
            b"mluc" => {
                let count = be_u32(t, 8)?;
                if count == 0 {
                    return None;
                }
                let len = be_u32(t, 20)? as usize;
                let off = be_u32(t, 24)? as usize;
                let raw = t.get(off..off + len)?;
                let units: Vec<u16> = raw
                    .chunks_exact(2)
                    .map(|c| u16::from_be_bytes([c[0], c[1]]))
                    .collect();
                Some(String::from_utf16_lossy(&units))
            }
            _ => None,
        }
    }

    /// Regamma LUTs from a Microsoft `MHC2` tag, if present and not an identity.
    fn mhc2_regamma(&self) -> Option<[Vec<f64>; 3]> {
        let t = self.tag("MHC2")?;
        if &t[..4] != b"MHC2" {
            return None;
        }
        let count = be_u32(t, 8)? as usize;
        if !(2..=4096).contains(&count) {
            return None;
        }
        let read_lut = |o: usize| -> Option<Vec<f64>> {
            let o = be_u32(t, o)? as usize;
            if t.get(o..o + 4)? != b"sf32" {
                return None;
            }
            (0..count).map(|i| s15f16(t, o + 8 + 4 * i)).collect()
        };
        let luts = [read_lut(24)?, read_lut(28)?, read_lut(32)?];
        let identity = luts.iter().all(|l| {
            l.iter()
                .enumerate()
                .all(|(i, &v)| (v - i as f64 / (count - 1) as f64).abs() < 1e-4)
        });
        (!identity).then_some(luts)
    }
}

fn parse_trc(t: &[u8]) -> anyhow::Result<Trc> {
    match &t[..4] {
        b"curv" => {
            let n = be_u32(t, 8).context("truncated curv")? as usize;
            match n {
                0 => Ok(Trc::Gamma(1.)),
                1 => Ok(Trc::Gamma(
                    f64::from(be_u16(t, 12).context("truncated curv")?) / 256.,
                )),
                _ => {
                    let table = (0..n)
                        .map(|i| be_u16(t, 12 + 2 * i).map(|v| f64::from(v) / 65535.))
                        .collect::<Option<Vec<_>>>()
                        .context("truncated curv")?;
                    Ok(Trc::Table(table))
                }
            }
        }
        b"para" => {
            let kind = be_u16(t, 8).context("truncated para")?;
            let p = |i: usize| s15f16(t, 12 + 4 * i).context("truncated para");
            let g = p(0)?;
            let trc = match kind {
                0 => Trc::Gamma(g),
                1 => {
                    let (a, b) = (p(1)?, p(2)?);
                    Trc::Parametric { g, a, b, c: 0., d: -b / a, e: 0., f: 0. }
                }
                2 => {
                    let (a, b, c) = (p(1)?, p(2)?, p(3)?);
                    Trc::Parametric { g, a, b, c: 0., d: -b / a, e: c, f: c }
                }
                3 => {
                    let (a, b, c, d) = (p(1)?, p(2)?, p(3)?, p(4)?);
                    Trc::Parametric { g, a, b, c, d, e: 0., f: 0. }
                }
                4 => {
                    let (a, b, c, d, e, f) = (p(1)?, p(2)?, p(3)?, p(4)?, p(5)?, p(6)?);
                    Trc::Parametric { g, a, b, c, d, e, f }
                }
                _ => bail!("unknown parametric curve type {kind}"),
            };
            Ok(trc)
        }
        other => bail!("unsupported TRC type {:?}", String::from_utf8_lossy(other)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    fn srgb_display() -> DisplayProfile {
        DisplayProfile {
            description: String::from("test sRGB"),
            rgb_to_xyz: rgb_to_xyz(&SRGB_PRIMARIES, D65_XY).unwrap(),
            white: D65_XY,
            trc: [Trc::Gamma(2.2), Trc::Gamma(2.2), Trc::Gamma(2.2)],
            regamma: None,
        }
    }

    #[test]
    fn srgb_matrix_matches_standard() {
        let m = rgb_to_xyz(&SRGB_PRIMARIES, D65_XY).unwrap();
        // IEC 61966-2-1 sRGB -> XYZ (D65).
        assert!(close(m[0][0], 0.4124, 1e-3));
        assert!(close(m[1][1], 0.7152, 1e-3));
        assert!(close(m[2][2], 0.9505, 1e-3));
    }

    #[test]
    fn srgb_display_is_identity() {
        let p = Pipeline::sdr(&srgb_display(), 0., 1024, 1024).unwrap();
        let ctm = p.ctm.unwrap();
        for r in 0..3 {
            for c in 0..3 {
                assert!(close(ctm[r][c], (r == c) as u8 as f64, 1e-9), "{ctm:?}");
            }
        }
        // Degamma is the content gamma; gamma inverts the display's gamma 2.2, so the two
        // cancel out end to end.
        let (d, g) = (p.degamma.unwrap(), p.gamma.unwrap());
        for i in [0, 100, 512, 1023] {
            let x = i as f64 / 1023.;
            assert!(close(d[i][0], x.powf(2.2), 1e-9));
            assert!(close(g[i][0].powf(2.2), x, 1e-6));
        }
    }

    #[test]
    fn full_intensity_is_identity() {
        let p3: [Xy; 3] = [(0.680, 0.320), (0.265, 0.690), (0.150, 0.060)];
        let display = DisplayProfile {
            rgb_to_xyz: rgb_to_xyz(&p3, D65_XY).unwrap(),
            ..srgb_display()
        };
        let ctm = Pipeline::sdr(&display, 1., 16, 16).unwrap().ctm.unwrap();
        for r in 0..3 {
            for c in 0..3 {
                assert!(close(ctm[r][c], (r == c) as u8 as f64, 1e-6), "{ctm:?}");
            }
        }
        // At 0, sRGB red on a P3 display needs less than full P3 red, and rows keep white.
        let ctm = Pipeline::sdr(&display, 0., 16, 16).unwrap().ctm.unwrap();
        assert!(ctm[0][0] < 0.9);
        for row in ctm {
            assert!(close(row.iter().sum(), 1., 1e-6));
        }
    }

    #[test]
    fn trc_inverse_roundtrips() {
        let table: Vec<f64> = (0..256).map(|i| (i as f64 / 255.).powf(2.4)).collect();
        for trc in [
            Trc::Gamma(2.2),
            Trc::Table(table),
            // sRGB as parametric type 3.
            Trc::Parametric { g: 2.4, a: 1. / 1.055, b: 0.055 / 1.055, c: 1. / 12.92, d: 0.04045, e: 0., f: 0. },
        ] {
            for y in [0.0, 0.001, 0.18, 0.5, 0.9, 1.0] {
                let x = trc.inverse(y);
                assert!(close(trc.eval(x), y, 1e-4), "{trc:?} y={y} x={x}");
            }
        }
    }

    #[test]
    fn ctm_encoding() {
        let m = [[1., -0.5, 0.], [0., 1., 0.], [0., 0., 1.]];
        let e = ctm_to_drm(&m);
        assert_eq!(e[0], 1 << 32);
        assert_eq!(e[1], (1 << 63) | (1 << 31));
        assert_eq!(e[2], 0);
    }

    #[test]
    fn edid_primaries() {
        // Base block header plus sRGB-ish chromaticity from a real sRGB panel EDID.
        let mut edid = [0u8; 128];
        edid[..8].copy_from_slice(&[0, 255, 255, 255, 255, 255, 255, 0]);
        edid[23] = 120; // gamma 2.2
        edid[25..35].copy_from_slice(&[0xEE, 0x91, 0xA3, 0x54, 0x4C, 0x99, 0x26, 0x0F, 0x50, 0x54]);
        let p = DisplayProfile::from_edid(&edid).unwrap();
        let prim = p.primaries();
        assert!(close(prim[0].0, 0.64, 0.01), "{prim:?}");
        assert!(close(prim[1].1, 0.60, 0.01), "{prim:?}");
        assert!(close(p.white.0, 0.3127, 0.005), "{:?}", p.white);
        assert_eq!(p.trc[0], Trc::Gamma(2.2));
    }

    #[test]
    fn compose_ramp_identity() {
        let out = compose_ramp(None, None, 256);
        assert_eq!(out[0], [0, 0, 0]);
        assert_eq!(out[255], [65535, 65535, 65535]);
        // A halving ramp after an identity curve.
        let ramp: Vec<u16> = (0..3).flat_map(|_| (0..256).map(|i| (i * 128) as u16)).collect();
        let out = compose_ramp(None, Some(&ramp), 256);
        assert!(out[255][0] > 32000 && out[255][0] < 33000);
    }
}
