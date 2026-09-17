use crate::constants::{EIGHT_BITS, FIVE_BITS, FOUR_BITS, ONE_BIT, SIX_BITS, SIXTEEN_BITS};
use crate::electromagnetic_emission::model::{
    ElectromagneticEmission, EmitterBeam, EmitterSystem, FundamentalParameter, SiteAppPair,
    TrackJam,
};
use crate::types::model::{CdisFloat, UVINT8};
use crate::writing::{BitBuffer, SerializeCdis, write_integer_bits};

impl SerializeCdis for ElectromagneticEmission {
    #[allow(clippy::let_and_return)]
    #[allow(clippy::cast_possible_truncation)]
    fn serialize(&self, buf: &mut BitBuffer, cursor: usize) -> usize {
        let cursor = write_integer_bits(buf, cursor, ONE_BIT, u8::from(self.full_update_flag));
        let cursor = write_integer_bits(buf, cursor, FIVE_BITS, self.fundamental_params.len());
        let cursor = write_integer_bits(buf, cursor, FIVE_BITS, self.beam_data.len());
        let cursor = write_integer_bits(buf, cursor, FIVE_BITS, self.site_app_pairs.len());

        let cursor = self.emitting_id.serialize(buf, cursor);
        let cursor = self.event_id.serialize(buf, cursor);

        let cursor =
            write_integer_bits(buf, cursor, ONE_BIT, u8::from(self.state_update_indicator));

        let cursor = UVINT8::from(self.emitter_systems.len() as u8).serialize(buf, cursor);

        let cursor = self
            .fundamental_params
            .iter()
            .fold(cursor, |cursor, param| param.serialize(buf, cursor));
        let cursor = self
            .beam_data
            .iter()
            .fold(cursor, |cursor, beam_data| beam_data.serialize(buf, cursor));
        let cursor = self
            .site_app_pairs
            .iter()
            .fold(cursor, |cursor, pair| pair.serialize(buf, cursor));

        let cursor = self
            .emitter_systems
            .iter()
            .fold(cursor, |cursor, system| system.serialize(buf, cursor));

        cursor
    }
}

impl SerializeCdis for FundamentalParameter {
    #[allow(clippy::let_and_return)]
    fn serialize(&self, buf: &mut BitBuffer, cursor: usize) -> usize {
        let cursor = self.frequency.serialize(buf, cursor);
        let cursor = self.frequency_range.serialize(buf, cursor);
        let cursor = write_integer_bits(buf, cursor, EIGHT_BITS, self.erp);
        let cursor = self.prf.serialize(buf, cursor);
        let cursor = self.pulse_width.serialize(buf, cursor);

        cursor
    }
}

impl SerializeCdis for SiteAppPair {
    #[allow(clippy::let_and_return)]
    fn serialize(&self, buf: &mut BitBuffer, cursor: usize) -> usize {
        let cursor = self.site.serialize(buf, cursor);
        let cursor = self.application.serialize(buf, cursor);

        cursor
    }
}

impl SerializeCdis for EmitterSystem {
    #[allow(clippy::let_and_return)]
    #[allow(clippy::cast_possible_truncation)]
    fn serialize(&self, buf: &mut BitBuffer, cursor: usize) -> usize {
        // Emitter System Details flag: true when both name and function are present.
        let cursor = write_integer_bits(
            buf,
            cursor,
            ONE_BIT,
            u8::from(self.name.is_some() && self.function.is_some()),
        );
        let cursor = write_integer_bits(
            buf,
            cursor,
            ONE_BIT,
            u8::from(self.location_with_respect_to_entity.is_some()),
        );

        let cursor = write_integer_bits(buf, cursor, FIVE_BITS, self.emitter_beams.len());

        let cursor = if self.name.is_some() && self.function.is_some() {
            let cursor = if let Some(name) = self.name {
                write_integer_bits(buf, cursor, SIXTEEN_BITS, u16::from(name))
            } else {
                cursor
            };
            let cursor = if let Some(function) = self.function {
                write_integer_bits(buf, cursor, EIGHT_BITS, u8::from(function))
            } else {
                cursor
            };
            cursor
        } else {
            cursor
        };
        let cursor = if let Some(location) = self.location_with_respect_to_entity {
            location.serialize(buf, cursor)
        } else {
            cursor
        };

        let cursor = self
            .emitter_beams
            .iter()
            .fold(cursor, |cursor, beam| beam.serialize(buf, cursor));

        cursor
    }
}

impl SerializeCdis for EmitterBeam {
    #[allow(clippy::let_and_return)]
    fn serialize(&self, buf: &mut BitBuffer, cursor: usize) -> usize {
        let cursor = write_integer_bits(
            buf,
            cursor,
            ONE_BIT,
            u8::from(self.fundamental_params_index.is_some()),
        );
        let cursor = write_integer_bits(
            buf,
            cursor,
            ONE_BIT,
            u8::from(self.beam_data_index.is_some()),
        );
        let cursor = write_integer_bits(
            buf,
            cursor,
            ONE_BIT,
            u8::from(self.jamming_technique_kind.is_some()),
        );
        let track_jam_flag = if let Some(track_jam) = self.track_jam.first() {
            track_jam.beam_number.is_some() & track_jam.emitter_number.is_some()
        } else {
            false
        };
        let cursor = write_integer_bits(buf, cursor, ONE_BIT, u8::from(track_jam_flag));

        let cursor = self.beam_id.serialize(buf, cursor);
        let cursor = write_integer_bits(buf, cursor, SIXTEEN_BITS, self.beam_parameter_index);

        let cursor = if let Some(index) = self.fundamental_params_index {
            write_integer_bits(buf, cursor, FIVE_BITS, index)
        } else {
            cursor
        };
        let cursor = if let Some(index) = self.beam_data_index {
            write_integer_bits(buf, cursor, FIVE_BITS, index)
        } else {
            cursor
        };
        let cursor = write_integer_bits(buf, cursor, FIVE_BITS, u8::from(self.beam_function));

        let cursor = write_integer_bits(buf, cursor, FOUR_BITS, self.track_jam.len());
        let cursor =
            write_integer_bits(buf, cursor, ONE_BIT, u8::from(self.high_density_track_jam));
        let cursor = write_integer_bits(buf, cursor, ONE_BIT, u8::from(self.beam_status));

        let cursor = if let Some(kind) = self.jamming_technique_kind {
            kind.serialize(buf, cursor)
        } else {
            cursor
        };
        let cursor = if let Some(category) = self.jamming_technique_category {
            category.serialize(buf, cursor)
        } else {
            cursor
        };
        let cursor = if let Some(subcategory) = self.jamming_technique_subcategory {
            subcategory.serialize(buf, cursor)
        } else {
            cursor
        };
        let cursor = if let Some(specific) = self.jamming_technique_specific {
            specific.serialize(buf, cursor)
        } else {
            cursor
        };

        let cursor = self
            .track_jam
            .iter()
            .fold(cursor, |cursor, track_jam| track_jam.serialize(buf, cursor));

        cursor
    }
}

impl SerializeCdis for TrackJam {
    #[allow(clippy::let_and_return)]
    fn serialize(&self, buf: &mut BitBuffer, cursor: usize) -> usize {
        let cursor = write_integer_bits(buf, cursor, SIX_BITS, self.site_app_pair_index);
        let cursor = self.entity_id.serialize(buf, cursor);

        let cursor = if let Some(number) = self.emitter_number {
            number.serialize(buf, cursor)
        } else {
            cursor
        };
        let cursor = if let Some(number) = self.beam_number {
            number.serialize(buf, cursor)
        } else {
            cursor
        };

        cursor
    }
}
