use crate::codec::Codec;
use crate::codec::{CodecOptions, CodecStateResult, CodecUpdateMode, DecoderState, EncoderState};
use crate::designator::model::{Designator, DesignatorUnits};
use crate::entity_state::codec::encode_dr_linear_acceleration;
use crate::records::codec::{
    decode_entity_coordinate_vector, decode_world_coordinates, encode_entity_coordinate_vector,
    encode_world_coordinates,
};
use crate::records::model::{EntityId, UnitsDekameters, UnitsMeters};
use crate::types::model::{UVINT16, UVINT32};
use crate::{BodyProperties, CdisBody};
use dis_rs::BodyRaw;
use dis_rs::enumerations::{DeadReckoningAlgorithm, DesignatorCode, DesignatorSystemName};
use dis_rs::model::{EntityId as DisEntityId, Location, PduBody};
use num_traits::FromPrimitive;
use std::time::Instant;

type Counterpart = dis_rs::designator::model::Designator;

pub(crate) fn encode_designator_body_and_update_state(
    dis_body: &Counterpart,
    state: &mut EncoderState,
    options: &CodecOptions,
) -> (CdisBody, CodecStateResult) {
    let state_for_id = state.designator.get(&dis_body.designating_entity_id);

    let (cdis_body, state_result) = Designator::encode(dis_body, state_for_id, options);

    if state_result == CodecStateResult::StateUpdateDesignator {
        state
            .designator
            .entry(dis_body.designating_entity_id)
            .and_modify(|designator| designator.heartbeat = Instant::now())
            .or_default();
    }

    (cdis_body.into_cdis_body(), state_result)
}

pub(crate) fn decode_designator_body_and_update_state(
    cdis_body: &Designator,
    state: &mut DecoderState,
    options: &CodecOptions,
) -> (PduBody, CodecStateResult) {
    let state_for_id = state
        .designator
        .get(&DisEntityId::from(&cdis_body.designating_entity_id));
    let (dis_body, state_result) = cdis_body.decode(state_for_id, options);

    if state_result == CodecStateResult::StateUpdateDesignator {
        state
            .designator
            .entry(DisEntityId::from(&cdis_body.designating_entity_id))
            .and_modify(|designator| {
                designator.heartbeat = Instant::now();
                designator.code_name = dis_body.system_name;
                designator.designator_code = dis_body.code;
                designator.designator_power = dis_body.power;
                designator.designator_wavelength = dis_body.wavelength;
                designator.designator_spot_location = dis_body.spot_location;
            })
            .or_insert(DecoderStateDesignator::new(&dis_body));
    }

    (dis_body.into_pdu_body(), state_result)
}

#[derive(Debug)]
pub struct EncoderStateDesignator {
    pub heartbeat: Instant,
}

impl Default for EncoderStateDesignator {
    fn default() -> Self {
        Self {
            heartbeat: Instant::now(),
        }
    }
}

#[derive(Debug)]
pub struct DecoderStateDesignator {
    pub heartbeat: Instant,
    pub code_name: DesignatorSystemName,
    pub designator_code: DesignatorCode,
    pub designator_power: f32,
    pub designator_wavelength: f32,
    pub designator_spot_location: Location,
}

impl DecoderStateDesignator {
    #[must_use]
    pub fn new(body: &Counterpart) -> Self {
        Self {
            heartbeat: Instant::now(),
            code_name: body.system_name,
            designator_code: body.code,
            designator_power: body.power,
            designator_wavelength: body.wavelength,
            designator_spot_location: body.spot_location,
        }
    }
}

impl Default for DecoderStateDesignator {
    fn default() -> Self {
        Self {
            heartbeat: Instant::now(),
            code_name: DesignatorSystemName::default(),
            designator_code: DesignatorCode::default(),
            designator_power: 0.0,
            designator_wavelength: 0.0,
            designator_spot_location: Location::default(),
        }
    }
}

impl Designator {
    fn encode(
        item: &Counterpart,
        state: Option<&EncoderStateDesignator>,
        options: &CodecOptions,
    ) -> (Self, CodecStateResult) {
        let designating_entity_id = EntityId::encode(&item.designating_entity_id);

        // When no designated entity is provided, the spot location in entity coordinates is also meaningless
        let (designated_entity_id, spot_wrt_designated_entity, location_wrt_entity_units) =
            if item.designated_entity_id != DisEntityId::default() {
                let designated_entity_id = EntityId::encode(&item.designated_entity_id);
                let (spot_wrt_designated_entity, location_wrt_entity_units) =
                    encode_entity_coordinate_vector(&item.spot_wrt_designated_entity);
                (
                    Some(designated_entity_id),
                    Some(spot_wrt_designated_entity),
                    location_wrt_entity_units,
                )
            } else {
                (None, None, UnitsMeters::default())
            };

        // When no DR algorithm is specified, the acceleration is not usable
        let (dr_algorithm, dr_entity_linear_acceleration) =
            if item.dead_reckoning_algorithm != DeadReckoningAlgorithm::Other {
                let dr_algorithm = Some(item.dead_reckoning_algorithm);
                let dr_entity_linear_acceleration = encode_dr_linear_acceleration(
                    item.dead_reckoning_algorithm,
                    &item.linear_acceleration,
                );
                (dr_algorithm, dr_entity_linear_acceleration)
            } else {
                (None, None)
            };

        let (
            full_update_flag,
            code_name,
            designator_code,
            designator_power,
            designator_wavelength,
            designator_spot_location,
            world_location_altitude_units,
            state_result,
        ) = if options.update_mode == CodecUpdateMode::PartialUpdate
            && state
                .is_some_and(|state| !evaluate_timeout_for_designator(&state.heartbeat, options))
        {
            // Do not update stateful fields when a full update is not required
            (
                false,
                None,
                None,
                None,
                None,
                None,
                UnitsDekameters::default(),
                CodecStateResult::StateUnaffected,
            )
        } else {
            // Full update mode, or heartbeat timeout has expired in partial update mode
            let wavelength = item.wavelength * 10f32; // microns (DIS) -> nanometers (C-DIS)
            let (spot_location, world_location_altitude_units) =
                encode_world_coordinates(&item.spot_location);
            (
                true,
                Some(item.system_name),
                Some(UVINT16::from(u16::from(item.code))),
                Some(UVINT32::from(u32::from_f32(item.power).unwrap_or_default())),
                Some(UVINT32::from(u32::from_f32(wavelength).unwrap_or_default())),
                Some(spot_location),
                world_location_altitude_units,
                CodecStateResult::StateUpdateDesignator,
            )
        };

        (
            Self {
                units: DesignatorUnits {
                    location_wrt_entity: location_wrt_entity_units,
                    world_location_altitude: world_location_altitude_units,
                },
                full_update_flag,
                designating_entity_id,
                code_name,
                designated_entity_id,
                designator_code,
                designator_power,
                designator_wavelength,
                spot_wrt_designated_entity,
                designator_spot_location,
                dr_algorithm,
                dr_entity_linear_acceleration,
            },
            state_result,
        )
    }

    fn decode(
        &self,
        state: Option<&DecoderStateDesignator>,
        _options: &CodecOptions,
    ) -> (Counterpart, CodecStateResult) {
        let state_result = if self.full_update_flag {
            CodecStateResult::StateUpdateDesignator
        } else {
            CodecStateResult::StateUnaffected
        };

        // If an optional field is missing and the PDU is a partial update, then attempt to retrieve
        // the field from the decoder state
        let fallback = if self.full_update_flag { None } else { state };
        let system_name = self
            .code_name
            .unwrap_or_else(|| fallback.map(|state| state.code_name).unwrap_or_default());
        let code = self.designator_code.map_or_else(
            || {
                fallback
                    .map(|state| state.designator_code)
                    .unwrap_or_default()
            },
            |code| DesignatorCode::from(code.value),
        );
        let power = self.designator_power.map_or_else(
            || {
                fallback
                    .map(|state| state.designator_power)
                    .unwrap_or_default()
            },
            |power| f32::from_u32(power.value).unwrap_or_default(),
        );
        let wavelength = self.designator_wavelength.map_or_else(
            || {
                fallback
                    .map(|state| state.designator_wavelength)
                    .unwrap_or_default()
            },
            |wavelength| f32::from_u32(wavelength.value).unwrap_or_default() / 10f32, // nanometers (C-DIS) -> microns (DIS)
        );
        let spot_location = self.designator_spot_location.map_or_else(
            || {
                fallback
                    .map(|state| state.designator_spot_location)
                    .unwrap_or_default()
            },
            |spot_location| {
                decode_world_coordinates(&spot_location, self.units.world_location_altitude)
            },
        );

        let designated_entity_id = self.designated_entity_id.unwrap_or_default().decode();
        let spot_wrt_designated_entity = decode_entity_coordinate_vector(
            &self.spot_wrt_designated_entity.unwrap_or_default(),
            self.units.location_wrt_entity,
        );
        let dr_algorithm = self.dr_algorithm.unwrap_or_default();
        let dr_entity_linear_acceleration = self
            .dr_entity_linear_acceleration
            .unwrap_or_default()
            .decode();

        (
            Counterpart::builder()
                .with_designating_entity_id(self.designating_entity_id.decode())
                .with_system_name(system_name)
                .with_designated_entity_id(designated_entity_id)
                .with_code(code)
                .with_power(power)
                .with_wavelength(wavelength)
                .with_spot_wrt_designated_entity(spot_wrt_designated_entity)
                .with_spot_location(spot_location)
                .with_dead_reckoning_algorithm(dr_algorithm)
                .with_linear_acceleration(dr_entity_linear_acceleration)
                .build(),
            state_result,
        )
    }
}

fn evaluate_timeout_for_designator(last_heartbeat: &Instant, options: &CodecOptions) -> bool {
    let elapsed = last_heartbeat.elapsed().as_secs_f32();
    elapsed
        > (options.federation_parameters.HBT_PDU_DESIGNATOR * options.hbt_cdis_full_update_mplier)
}
