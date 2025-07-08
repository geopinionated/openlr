# OpenLR Rust implementation

Binary (and Base64) ser/deserialization of OpenLR Location References (version 3).

[White Paper](https://download.tomtom.com/open/banners/openlr-whitepaper_v1.5.pdf)

[Reference Implementation (Java)](https://github.com/tomtom-international/openlr)


### Example

```rust
use openlr::{Coordinate, LocationReference, deserialize_base64_openlr, serialize_base64_openlr};

let location = LocationReference::GeoCoordinate(Coordinate {
    lon: 13.090918,
    lat: 52.466884,
});

let encoded: String = serialize_base64_openlr(&location).unwrap();
let decoded: LocationReference = deserialize_base64_openlr(&encoded).unwrap();
```
