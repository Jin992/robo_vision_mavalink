//
// Created by jin on 22.04.19.
//

#ifndef WEBSOCKET_TEST_BITRATE_H
#define WEBSOCKET_TEST_BITRATE_H

/**
 * Bitrate control structure
 */
typedef struct  bitrate_s {
    int32_t bitrate;            /// bitrate value
    int32_t min_bitrate;        /// minimal bitrate that codec can output
    int32_t max_bitrate;        /// maximal bitrate that codec can output
    int8_t  percent_substruct;  /// value in % that we subtract from bitrate when we need to decrease bitrate
    int8_t  persent_add;        /// value in % that we add to bitrate when we need to increase bitrate
    int32_t changes_period;     /// interval in milliseconds
    int32_t last_changes;       /// last bitrate change timestamp
    size_t  stability_cnt;      /// counter that count stable bandwidth transmission sequence
    bool    changes;            /// bitrate change flag
    std::mutex  bitrate_guard;  /// guard birate
}               bitrate_t;

#endif //WEBSOCKET_TEST_BITRATE_H
