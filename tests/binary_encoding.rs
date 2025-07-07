use openlr::{LocationType, decode_base64_openlr, encode_base64_openlr};

#[test]
fn openlr_binary_encoding() {
    let openlr_codes = include_str!("data/lines.txt");

    for code in openlr_codes.split('\n') {
        let location = decode_base64_openlr(code).unwrap();
        assert_eq!(location.location_type(), LocationType::Line, "{code}");

        let encoded_code = encode_base64_openlr(&location).unwrap();
        assert_eq!(encoded_code, code, "{code}");
    }
}
