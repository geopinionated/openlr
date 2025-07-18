use openlr::{LocationType, deserialize_base64_openlr, serialize_base64_openlr};

#[test]
fn openlr_binary_deserialization() {
    let openlr_codes = include_str!("data/lines.txt");

    for code in openlr_codes.split('\n') {
        let location = deserialize_base64_openlr(code).unwrap();
        assert_eq!(location.location_type(), LocationType::Line, "{code}");

        let encoded_code = serialize_base64_openlr(&location).unwrap();
        assert_eq!(encoded_code, code, "{code}");
    }
}
