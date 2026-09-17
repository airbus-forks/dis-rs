use crate::BodyProperties;
use crate::constants::{FOUR_BITS, ONE_BIT, SIXTEEN_BITS};
use crate::designator::model::Designator;
use crate::writing::{BitBuffer, SerializeCdis, serialize_when_present, write_integer_bits};
use dis_rs::enumerations::DesignatorSystemName;

impl SerializeCdis for Designator {
    #[allow(clippy::let_and_return)]
    fn serialize(&self, buf: &mut BitBuffer, cursor: usize) -> usize {
        let fields_present = self.fields_present_field();

        let cursor = write_integer_bits(buf, cursor, self.fields_present_length(), fields_present);
        let cursor = write_integer_bits(
            buf,
            cursor,
            ONE_BIT,
            u8::from(self.units.location_wrt_entity_units),
        );
        let cursor = write_integer_bits(
            buf,
            cursor,
            ONE_BIT,
            u8::from(self.units.world_location_altitude),
        );
        let cursor = write_integer_bits(buf, cursor, ONE_BIT, u8::from(self.full_update_flag));
        let cursor = self.designating_entity_id.serialize(buf, cursor);

        let cursor = serialize_when_present(&self.code_name, buf, cursor);
        let cursor = serialize_when_present(&self.designated_entity_id, buf, cursor);
        let cursor = serialize_when_present(&self.designator_code, buf, cursor);
        let cursor = serialize_when_present(&self.designator_power, buf, cursor);
        let cursor = serialize_when_present(&self.designator_wavelength, buf, cursor);
        let cursor = serialize_when_present(&self.spot_wrt_designated_entity, buf, cursor);
        let cursor = serialize_when_present(&self.designator_spot_location, buf, cursor);

        let cursor = if let Some(algo) = self.dr_algorithm {
            write_integer_bits(buf, cursor, FOUR_BITS, u8::from(algo))
        } else {
            cursor
        };
        let cursor = serialize_when_present(&self.dr_entity_linear_acceleration, buf, cursor);

        cursor
    }
}

impl SerializeCdis for DesignatorSystemName {
    #[allow(clippy::let_and_return)]
    fn serialize(&self, buf: &mut BitBuffer, cursor: usize) -> usize {
        let cursor = write_integer_bits(buf, cursor, SIXTEEN_BITS, u16::from(*self));

        cursor
    }
}
