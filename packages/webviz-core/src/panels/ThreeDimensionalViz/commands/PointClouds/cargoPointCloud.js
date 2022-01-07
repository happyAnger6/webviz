//
//  Copyright (c) 2022-present, zhangxiaoan@didi.global
//

import { Float32Reader, Int32Reader, Int64Reader } from "./readers";
import "webviz-core/src/types/lidar_pb";

const CARGO_FIELDS = [
  {name: 'x', count:1, offset:0, datatype: 7},
  {name: 'y', count:1, offset:4, datatype: 7},
  {name: 'z', count:1, offset:8, datatype: 7},
  {name: 'intensity', count:1, offset:12, datatype: 2},
  {name: 'ring', count:1, offset:14, datatype: 4},
];

const CARGO_POINT_STEP = 24;

export function toRosDrawData(byteData) {
  const pb_buffer = byteData.slice(12);
  const cargo_point_cloud = proto.cargo.lidar.PointCloud.deserializeBinary(pb_buffer);
  const timestamp = cargo_point_cloud.getHeader().getTimestampNs();
  const packed_scan_data = cargo_point_cloud.getScan().getData();
  const point_cloud_data = packed_scan_data.slice(8);
  const reader = new Int32Reader(0);
  const point_num = reader.read(packed_scan_data, 0);
  return {
    header: {
      frame_id: "cargo/lidar/main/left",
      seq: cargo_point_cloud.getHeader().getSequenceNum(),
      stamp: { sec: timestamp / 1000 * 1000 * 1000, nsec: timestamp % (1000 * 1000 * 1000)}
    },
    data: point_cloud_data,
    fields: CARGO_FIELDS,
    point_step: CARGO_POINT_STEP,
    is_bigendian: false,
    is_dense: true,
    width: point_num,
    height: 1,
    row_step: point_cloud_data.byteLength
  };
}
