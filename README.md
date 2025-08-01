# OpenLR Rust implementation

Binary (and Base64) ser/deserialization of OpenLR Location References (version 3) with basic support for line encoder and decoder capabilities.

[White Paper](https://download.tomtom.com/open/banners/openlr-whitepaper_v1.5.pdf)

[Reference Implementation (Java)](https://github.com/tomtom-international/openlr)


### Examples

#### Serialization

```rust
use openlr::{Coordinate, LocationReference, deserialize_base64_openlr, serialize_base64_openlr};

let location = LocationReference::GeoCoordinate(Coordinate {
    lon: 13.090918,
    lat: 52.466884,
});

let encoded: String = serialize_base64_openlr(&location).unwrap();
let decoded: LocationReference = deserialize_base64_openlr(&encoded).unwrap();
```

#### Decoding and Encoding

```rust,ignore
use openlr::{
    DecoderConfig, DirectedGraph, EncoderConfig, Location, decode_base64_openlr, encode_base64_openlr
};

struct RoadNetworkGraph;

type VertexId = i64;
type EdgeId = i64;

impl DirectedGraph for RoadNetworkGraph {
    type VertexId = VertexId;
    type EdgeId = EdgeId;

    // TODO: implement DirectedGraph methods
}

let graph = RoadNetworkGraph;

let location: Location<EdgeId> =
    decode_base64_openlr(&DecoderConfig::default(), &graph, "CwmShiVYczPJBgCs/y0zAQ==").unwrap();

let location: String =
    encode_base64_openlr(&EncoderConfig::default(), &graph, location).unwrap();
```
