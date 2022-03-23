const DEFAULT_SAMPLE_RATE: u32 = 48_000;
const MAX_DELAY_SAMPLES: usize = 1024 * 2; // Assumption of stereo!
const SAMPLES_PER_UPDATE: usize = 32;
const SPACING_DB: f32 = 5.0;

pub struct Compressor {
    num_channels: usize,
    meter_gain: f32,
    meter_release: f32,
    threshold: f32,
    knee: f32,
    linear_pre_gain: f32,
    linear_threshold: f32,
    slope: f32,
    attack_samples_inverse: f32,
    sat_release_samples_inverse: f32,
    wet: f32,
    dry: f32,
    k: f32,
    knee_db_offset: f32,
    linear_threshold_knee: f32,
    master_gain: f32,
    a: f32, // adaptive release polynomial coefficients
    b: f32,
    c: f32,
    d: f32,
    detector_average: f32,
    compressor_gain: f32,
    max_compressor_diff_db: f32,
    delay_buffer_size: usize,
    delay_write_pos: usize,
    delay_read_pos: usize,
    delay_buffer: Vec<f32>,
}

impl Compressor {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        num_channels: usize,
        sample_rate: u32,
        pre_gain: f32,
        threshold: f32,
        knee: f32,
        ratio: f32,
        attack: f32,
        release: f32,
    ) -> Self {
        Self::advanced_compressor(
            num_channels,
            sample_rate,
            pre_gain,
            threshold,
            knee,
            ratio,
            attack,
            release,
            0.006, // predelay
            0.090, // releasezone1
            0.160, // releasezone2
            0.420, // releasezone3
            0.980, // releasezone4
            0.000, // postgain
            1.000, // wet
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn advanced_compressor(
        num_channels: usize,
        sample_rate: u32,
        pre_gain: f32,
        threshold: f32,
        knee: f32,
        ratio: f32,
        attack: f32,
        release: f32,
        pre_delay: f32,
        release_zone_1: f32,
        release_zone_2: f32,
        release_zone_3: f32,
        release_zone_4: f32,
        post_gain: f32,
        wet: f32,
    ) -> Self {
        let mut delay_buffer_size = (sample_rate as f32 * pre_delay) as usize * num_channels;

        if delay_buffer_size < 1 {
            delay_buffer_size = 1;
        } else if delay_buffer_size > MAX_DELAY_SAMPLES {
            delay_buffer_size = MAX_DELAY_SAMPLES;
        }

        let linear_pre_gain = db_to_linear(pre_gain);
        let linear_threshold = db_to_linear(threshold);
        let slope = 1.0 / ratio;
        let attack_samples = sample_rate as f32 * attack;
        let attack_samples_inverse = 1.0 / attack_samples;
        let release_samples = sample_rate as f32 * release;
        let sat_release = 0.0025;
        let sat_release_samples_inverse = 1.0 / (sample_rate as f32 * sat_release);
        let dry = 1.0 - wet;

        let meter_gain = 1.0;
        let meter_falloff = 0.325; // Seconds
        let meter_release = 1.0 - f32::exp(-1.0 / (sample_rate as f32 * meter_falloff));
        let mut k = 0.5;
        let mut knee_db_offset = 0.0;
        let mut linear_threshold_knee = 0.0;

        if knee > 0.0 {
            let x_knee = db_to_linear(threshold + knee);
            let mut min_k = 0.1;
            let mut max_k = 10000.0;

            for _ in 0..15 {
                if knee_slope(x_knee, k, linear_threshold) < slope {
                    max_k = k;
                } else {
                    min_k = k;
                }

                k = (min_k * max_k).sqrt();
            }

            knee_db_offset = linear_to_db(knee_curve(x_knee, k, linear_threshold));
            linear_threshold_knee = db_to_linear(threshold + knee);
        }

        let full_level = compresor_curve(
            1.0,
            k,
            slope,
            linear_threshold,
            linear_threshold_knee,
            threshold,
            knee,
            knee_db_offset,
        );
        let master_gain = db_to_linear(post_gain) * (1.0 / full_level).powf(0.6);

        let y1 = release_samples * release_zone_1;
        let y2 = release_samples * release_zone_2;
        let y3 = release_samples * release_zone_3;
        let y4 = release_samples * release_zone_4;

        let a = (-y1 + 3.0 * y2 - 3.0 * y3 + y4) / 6.0;
        let b = y1 - 2.5 * y2 + 2.0 * y3 - 0.5 * y4;
        let c = (-11.0 * y1 + 18.0 * y2 - 9.0 * y3 + 2.0 * y4) / 6.0;
        let d = y1;

        Self {
            num_channels,
            meter_gain,
            meter_release,
            threshold,
            knee,
            linear_pre_gain,
            linear_threshold,
            slope,
            attack_samples_inverse,
            sat_release_samples_inverse,
            wet,
            dry,
            k,
            knee_db_offset,
            linear_threshold_knee,
            master_gain,
            a,
            b,
            c,
            d,
            detector_average: 0.0,
            compressor_gain: 1.0,
            max_compressor_diff_db: -1.0,
            delay_buffer_size,
            delay_write_pos: 0,
            delay_read_pos: if delay_buffer_size > 1 { 1 } else { 0 },
            delay_buffer: vec![0.0; MAX_DELAY_SAMPLES * num_channels],
        }
    }

    // It is assumed that `buffer` is an interleaved audio buffer.
    // It is modified in place.
    pub fn process(&mut self, buffer: &mut [f32]) {
        let chunks = buffer.len() / self.num_channels / SAMPLES_PER_UPDATE;
        let ang_90 = std::f32::consts::PI * 0.5;
        let ang_90_inv = 2.0 / std::f32::consts::PI;
        let mut sample_pos = 0;

        for _ in 0..chunks {
            self.detector_average = fix_float(self.detector_average, 1.0);
            let desired_gain = self.detector_average;
            let scaled_desired_gain = desired_gain.asin() * ang_90_inv;
            let mut compressor_diff_db = linear_to_db(self.compressor_gain / scaled_desired_gain);

            // Calculate envelope rate based on whether we're attacking or releasing
            let envelope_rate = if compressor_diff_db < 0.0 {
                compressor_diff_db = fix_float(compressor_diff_db, -1.0);
                self.max_compressor_diff_db = -1.0;

                let x = (compressor_diff_db.clamp(-12.0, 0.0) + 12.0) * 0.25;
                let release_samples = adaptive_release_curve(x, self.a, self.b, self.c, self.d);

                db_to_linear(SPACING_DB / release_samples)
            } else {
                compressor_diff_db = fix_float(compressor_diff_db, 1.0);

                if self.max_compressor_diff_db == -1.0
                    || self.max_compressor_diff_db < compressor_diff_db
                {
                    self.max_compressor_diff_db = compressor_diff_db;
                }

                let mut attenuate = self.max_compressor_diff_db;

                if attenuate < 0.5 {
                    attenuate = 0.5;
                }

                1.0 - (0.25 / attenuate).powf(self.attack_samples_inverse)
            };

            for _ in 0..SAMPLES_PER_UPDATE {
                let input_max = {
                    let mut input_max = 0.0f32;

                    for c in 0..self.num_channels {
                        let input_sample =
                            buffer[(sample_pos * self.num_channels) + c] * self.linear_pre_gain;

                        self.delay_buffer[(self.delay_write_pos * self.num_channels) + c] =
                            input_sample;

                        input_max = input_max.max(input_sample.abs());
                    }

                    input_max
                };

                let attenuation = if input_max < 0.0001 {
                    1.0
                } else {
                    let input_compressor = compresor_curve(
                        input_max,
                        self.k,
                        self.slope,
                        self.linear_threshold,
                        self.linear_threshold_knee,
                        self.threshold,
                        self.knee,
                        self.knee_db_offset,
                    );
                    input_compressor / input_max
                };

                let rate = if attenuation > self.detector_average {
                    let mut attenuation_db = -linear_to_db(attenuation);
                    if attenuation_db < 2.0 {
                        attenuation_db = 2.0;
                    }
                    let db_per_sample = attenuation_db * self.sat_release_samples_inverse;
                    db_to_linear(db_per_sample) - 1.0
                } else {
                    1.0
                };

                self.detector_average += (attenuation - self.detector_average) * rate;

                if self.detector_average > 1.0 {
                    self.detector_average = 1.0;
                }

                self.detector_average = fix_float(self.detector_average, 1.0);

                if envelope_rate < 1.0 {
                    self.compressor_gain +=
                        (scaled_desired_gain - self.compressor_gain) * envelope_rate;
                } else {
                    self.compressor_gain *= envelope_rate;

                    if self.compressor_gain > 1.0 {
                        self.compressor_gain = 1.0
                    }
                }

                let premix_gain = (ang_90 * self.compressor_gain).sin();
                let gain = self.dry + self.wet * self.master_gain * premix_gain;

                let premix_gain_db = linear_to_db(premix_gain);

                if premix_gain_db < self.meter_gain {
                    self.meter_gain = premix_gain;
                } else {
                    self.meter_gain += (premix_gain_db - self.meter_gain) * self.meter_release;
                }

                for c in 0..self.num_channels {
                    buffer[(sample_pos * self.num_channels) + c] =
                        self.delay_buffer[(self.delay_read_pos * self.num_channels) + c] * gain;
                }

                sample_pos += 1;
                self.delay_read_pos = (self.delay_read_pos + 1) % self.delay_buffer_size;
                self.delay_write_pos = (self.delay_write_pos + 1) % self.delay_buffer_size;
            }
        }
    }
}

impl Default for Compressor {
    fn default() -> Self {
        Self::advanced_compressor(
            2,
            DEFAULT_SAMPLE_RATE,
            0.000,   // pregain
            -24.000, // threshold
            30.000,  // knee
            12.000,  // ratio
            0.003,   // attack
            0.250,   // release
            0.006,   // predelay
            0.090,   // releasezone1
            0.160,   // releasezone2
            0.420,   // releasezone3
            0.980,   // releasezone4
            0.000,   // postgain
            1.000,   // wet
        )
    }
}

fn db_to_linear(db: f32) -> f32 {
    10.0_f32.powf(0.05 * db)
}

fn linear_to_db(linear: f32) -> f32 {
    20.0 * f32::log10(linear)
}

fn knee_slope(x: f32, k: f32, linear_threshold: f32) -> f32 {
    k * x / ((k * linear_threshold + 1.0) * f32::exp(k * (x - linear_threshold)) - 1.0)
}

fn knee_curve(x: f32, k: f32, linear_threshold: f32) -> f32 {
    linear_threshold + (1.0 - f32::exp(-k * (x - linear_threshold))) / k
}

fn fix_float(v: f32, def: f32) -> f32 {
    if v.is_nan() || v.is_infinite() {
        // TODO - Add a warning here to see where this actually happens.
        def
    } else {
        v
    }
}

fn adaptive_release_curve(x: f32, a: f32, b: f32, c: f32, d: f32) -> f32 {
    let x2 = x * x;
    a * x2 * x + b * x2 + c * x + d
}

#[allow(clippy::too_many_arguments)]
fn compresor_curve(
    x: f32,
    k: f32,
    slope: f32,
    linear_threshold: f32,
    linear_threshold_knee: f32,
    threshold: f32,
    knee: f32,
    knee_db_offset: f32,
) -> f32 {
    if x < linear_threshold {
        x
    } else if knee <= 0.0 {
        db_to_linear(threshold + slope * (linear_to_db(x) - threshold))
    } else if x < linear_threshold_knee {
        knee_curve(x, k, linear_threshold)
    } else {
        db_to_linear(knee_db_offset + slope * (linear_to_db(x) - threshold - knee))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let mut compressor = {
            let num_channels = 2;
            let pre_gain = 0.0;
            let threshold = -24.0;
            let knee = 30.0;
            let ratio = 12.0;
            let attack = 0.003; // Seconds
            let release = 0.25; // Seconds

            Compressor::new(
                num_channels,
                DEFAULT_SAMPLE_RATE,
                pre_gain,
                threshold,
                knee,
                ratio,
                attack,
                release,
            )
        };

        let mut buffer = [0.0; 960];

        compressor.process(&mut buffer);
    }
}
