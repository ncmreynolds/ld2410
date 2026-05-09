// Native host-side unit test for ld2410::parse_data_frame_().
// Feeds known-good frames (from the HLK-LD2410C V1.00 protocol document
// in docs/HLK-LD2410C_protocol.md) through the public read() API via a
// mock Stream, and asserts that the resulting field values match the
// protocol specification.
//
// Build & run:  bash tests/run.sh   (from the repo root)
//
// On the host ESP32 is not defined, so the FreeRTOS task path and the
// portMUX critical sections are compiled out. The test exercises only
// the parser, which is the surface this branch actually changed.

#include <Arduino.h>
#include <ld2410.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cassert>
#include <initializer_list>

// Mock UART. Two modes:
//   1. inject(...)            -- bytes are available IMMEDIATELY (used by
//                                data-frame tests that don't care about
//                                command/response ordering).
//   2. inject_response(...)   -- bytes are STAGED and released only when the
//                                test code observes the radar protocol
//                                postamble (0x04 0x03 0x02 0x01) being
//                                written to the UART. This models the real
//                                radar, which only emits an ACK after it
//                                has received a complete command.
class MockSerial : public Stream {
    std::vector<uint8_t> q_;          // bytes available for read()
    size_t pos_ = 0;
    std::vector<std::vector<uint8_t>> response_queue_;  // staged responses
    uint8_t last4_[4] = {0};
    int write_count_ = 0;
public:
    void inject(std::initializer_list<uint8_t> bytes) {
        q_.insert(q_.end(), bytes.begin(), bytes.end());
    }
    void inject(const std::vector<uint8_t>& bytes) {
        q_.insert(q_.end(), bytes.begin(), bytes.end());
    }
    void inject_response(const std::vector<uint8_t>& bytes) {
        response_queue_.push_back(bytes);
    }
    int available() override { return (int)(q_.size() - pos_); }
    int read() override {
        if (pos_ >= q_.size()) return -1;
        return q_[pos_++];
    }
    size_t write(uint8_t b) override {
        last4_[write_count_ % 4] = b;
        write_count_++;
        if (write_count_ >= 4) {
            // Recover the chronological order of the last 4 writes.
            uint8_t b0 = last4_[(write_count_ - 4) % 4];
            uint8_t b1 = last4_[(write_count_ - 3) % 4];
            uint8_t b2 = last4_[(write_count_ - 2) % 4];
            uint8_t b3 = last4_[(write_count_ - 1) % 4];
            if (b0 == 0x04 && b1 == 0x03 && b2 == 0x02 && b3 == 0x01) {
                if (!response_queue_.empty()) {
                    const auto& resp = response_queue_.front();
                    q_.insert(q_.end(), resp.begin(), resp.end());
                    response_queue_.erase(response_queue_.begin());
                }
            }
        }
        return 1;
    }
    using Print::write;  // pull in the size_t write(const uint8_t*, size_t) overload
    void clear() {
        q_.clear(); pos_ = 0;
        response_queue_.clear();
        write_count_ = 0;
    }
};

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
} while (0)

#define CHECK_EQ(a, b) do { \
    auto _a = (a); auto _b = (b); \
    if (!(_a == _b)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s == %s : got %lld vs %lld\n", \
                     __FILE__, __LINE__, #a, #b, (long long)_a, (long long)_b); \
        failures++; \
    } \
} while (0)

// Helper: pump the parser by calling read() until all queued bytes are drained.
static void drain(ld2410& r, MockSerial& s) {
    while (s.available() > 0) {
        r.read();
    }
}

// ---------------------------------------------------------------------------
// Test 1: basic frame from protocol §2.3.2 (Table 12 example).
//   F4 F3 F2 F1 | 0D 00 | 02 AA 02 51 00 00 00 00 3B 00 00 55 00 | F8 F7 F6 F5
// Decoded:
//   data_type=0x02 (basic), target_status=0x02,
//   moving distance=0x0051=81 cm, moving energy=0,
//   stationary distance=0, stationary energy=0x3B=59,
//   detection distance=0
// ---------------------------------------------------------------------------
#if !defined(LD2410_VARIANT_S)
static void test_basic_frame() {
    std::printf("test_basic_frame ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,             // header
        0x0D, 0x00,                         // intra-frame data length = 13
        0x02, 0xAA,                         // data_type basic, head 0xAA
        0x02,                               // target status
        0x51, 0x00,                         // moving distance LE = 81
        0x00,                               // moving energy
        0x00, 0x00,                         // stationary distance = 0
        0x3B,                               // stationary energy = 59
        0x00, 0x00,                         // detection distance = 0
        0x55, 0x00,                         // tail + calibration
        0xF8, 0xF7, 0xF6, 0xF5              // footer
    });
    drain(r, s);

    CHECK(r.presenceDetected());                     // target_status != 0
    CHECK(!r.movingTargetDetected());                // moving energy 0 -> not detected
    // Note: r.stationaryTargetDetected() is intentionally NOT asserted.
    // The doc's contrived basic-frame example has stationary_distance=0,
    // and the helper requires both distance > 0 and energy > 0. Parser is
    // correct; the doc example is just synthetic. We assert raw fields.
    CHECK_EQ((int)r.movingTargetDistance(), 81);
    CHECK_EQ((int)r.movingTargetEnergy(), 0);
    CHECK_EQ((int)r.stationaryTargetDistance(), 0);
    CHECK_EQ((int)r.stationaryTargetEnergy(), 59);
    CHECK_EQ((int)r.detectionDistance(), 0);
    CHECK(!r.engineeringRetrieved());                // no eng frame yet
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 2: engineering frame from protocol §2.3.2 (Table 14 example).
//   F4 F3 F2 F1 | 23 00 | 01 AA 03 1E 00 3C 00 00 39 00 00
//                          08 08
//                          3C 22 05 03 03 04 03 06 05
//                          00 00 39 10 13 06 06 08 04
//                          03 05    (retain bytes)
//                          55 00 |
//   F8 F7 F6 F5
// Decoded: target_status=0x03, moving=30 cm @ 60, stationary=0 @ 57,
//   detection=0; per-gate motion energies = [60, 34, 5, 3, 3, 4, 3, 6, 5];
//   per-gate stationary energies = [0, 0, 57, 16, 19, 6, 6, 8, 4].
// Pre-fix: the strict 0x02 check rejected this frame and the buffer-size
// guard (LD2410_MAX_FRAME_LENGTH=40) would have dropped it before parsing.
// ---------------------------------------------------------------------------
static void test_engineering_frame() {
    std::printf("test_engineering_frame ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x23, 0x00,                                // intra length = 35
        0x01, 0xAA,                                // engineering, head
        0x03,                                      // target status: both
        0x1E, 0x00,                                // moving dist = 30
        0x3C,                                      // moving energy = 60
        0x00, 0x00,                                // stationary dist = 0
        0x39,                                      // stationary energy = 57
        0x00, 0x00,                                // detection dist = 0
        0x08, 0x08,                                // max moving N=8, max stationary N=8
        0x3C, 0x22, 0x05, 0x03, 0x03, 0x04, 0x03, 0x06, 0x05,  // motion gate 0..8
        0x00, 0x00, 0x39, 0x10, 0x13, 0x06, 0x06, 0x08, 0x04,  // stationary gate 0..8
        0x03, 0x05,                                // retain (M=2)
        0x55, 0x00,                                // tail + cal at idx 39, 40
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);

    CHECK_EQ((int)r.movingTargetDistance(), 30);
    CHECK_EQ((int)r.movingTargetEnergy(), 60);
    CHECK_EQ((int)r.stationaryTargetDistance(), 0);
    CHECK_EQ((int)r.stationaryTargetEnergy(), 57);
    CHECK_EQ((int)r.detectionDistance(), 0);

    // Per-gate energies: this is the new surface that pre-fix did not exist.
    int expected_motion[9]     = {60, 34,  5,  3,  3,  4,  3,  6,  5};
    int expected_stationary[9] = { 0,  0, 57, 16, 19,  6,  6,  8,  4};
    for (uint8_t g = 0; g < 9; g++) {
        CHECK_EQ((int)r.movingEnergyAtGate(g), expected_motion[g]);
        CHECK_EQ((int)r.stationaryEnergyAtGate(g), expected_stationary[g]);
    }
    // Out-of-range gate index must return 0, not out-of-bounds-read.
    CHECK_EQ((int)r.movingEnergyAtGate(9), 0);
    CHECK_EQ((int)r.stationaryEnergyAtGate(255), 0);

    CHECK(r.engineeringRetrieved());
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 3: malformed frame (bad data_type) must be rejected.
// Same length as a basic frame but data_type = 0x99 (neither 0x01 nor 0x02).
// ---------------------------------------------------------------------------
static void test_invalid_data_type() {
    std::printf("test_invalid_data_type ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x0D, 0x00,
        0x99, 0xAA,                                // INVALID data_type
        0x02, 0x51, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00,
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);

    // Fields must remain at their initial zero values - no spurious update.
    CHECK_EQ((int)r.movingTargetDistance(), 0);
    CHECK_EQ((int)r.stationaryTargetEnergy(), 0);
    CHECK_EQ((int)r.detectionDistance(), 0);
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 4: bad trailer byte must be rejected (anchor on dynamic position).
// Use a basic frame but corrupt the 0x55 tail.
// ---------------------------------------------------------------------------
static void test_invalid_trailer() {
    std::printf("test_invalid_trailer ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x0D, 0x00,
        0x02, 0xAA,
        0x02, 0x51, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00,
        0x77, 0x00,                                // bogus tail (must be 0x55 0x00)
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);

    CHECK_EQ((int)r.movingTargetDistance(), 0);    // not updated
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 5: two consecutive frames - basic then engineering - should reuse
// the same library instance correctly and reflect the latest values.
// ---------------------------------------------------------------------------
static void test_sequential_frames() {
    std::printf("test_sequential_frames ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // First: a basic frame with moving distance 100 cm
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1, 0x0D, 0x00,
        0x02, 0xAA, 0x01, 0x64, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.movingTargetDistance(), 100);
    CHECK_EQ((int)r.movingTargetEnergy(), 50);
    CHECK(!r.engineeringRetrieved());

    // Then: engineering frame, moving distance 200, gate 5 motion energy 42
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1, 0x23, 0x00,
        0x01, 0xAA, 0x01,
        0xC8, 0x00,                                // moving dist = 200
        0x14,                                      // moving energy = 20
        0x00, 0x00, 0x00, 0x00, 0x00,              // stationary + detection
        0x08, 0x08,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00,  // gate 5 motion = 0x2A=42
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.movingTargetDistance(), 200);
    CHECK_EQ((int)r.movingTargetEnergy(), 20);
    CHECK_EQ((int)r.movingEnergyAtGate(5), 42);
    CHECK(r.engineeringRetrieved());
    std::printf("ok\n");
}

#endif // !LD2410_VARIANT_S

// ===========================================================================
// Command-side test helpers — used by both base/C and S suites, so they
// live OUTSIDE the variant guards. They synthesise ACK frames the way a
// real radar would send them, with the LE intra-length, the cmd-word ACK
// (op | 0x0100), and the canonical envelope.
// ===========================================================================

#if !defined(LD2410_VARIANT_S)
// Build a 0xA0 (firmware version) ACK frame with major=1, minor=7, bugfix=0x22091516.
// Only used by the base/C firmware test (S has its own FW layout, see step 9).
static std::vector<uint8_t> make_firmware_ack(uint8_t major, uint8_t minor,
                                              uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    return {
        0xFD, 0xFC, 0xFB, 0xFA,   // header
        0x0C, 0x00,               // intra-frame data length = 12
        0xA0, 0x01,               // command word ACK = 0xA0 | 0x0100 -> LE A0 01
        0x00, 0x00,               // ACK status: success
        0x01, 0x00,               // firmware type = 0x0001 LE
        minor, major,             // bytes 12, 13: minor then major (per parse_command_frame_)
        b0, b1, b2, b3,           // bugfix uint32 LE
        0x04, 0x03, 0x02, 0x01    // footer
    };
}
#endif

// Build a generic short ACK (4-byte payload, status=success) for opcode `op`.
// Used by enter_configuration_mode_'s ACK (0xFF) and most other commands.
static std::vector<uint8_t> make_short_ack(uint8_t op, uint16_t intra_len = 4) {
    std::vector<uint8_t> v = {
        0xFD, 0xFC, 0xFB, 0xFA,
        (uint8_t)(intra_len & 0xFF), (uint8_t)((intra_len >> 8) & 0xFF),
        op, 0x01,                // command word ACK
        0x00, 0x00               // status: success
    };
    // Pad with zero bytes if the caller asked for a longer intra_len so
    // length matches what the radar would actually send for some commands.
    for (uint16_t i = 4; i < intra_len; i++) v.push_back(0x00);
    v.push_back(0x04); v.push_back(0x03); v.push_back(0x02); v.push_back(0x01);
    return v;
}

#if !defined(LD2410_VARIANT_S)
// ===========================================================================
// Command-side tests (added by refactor/task-safe-commands).
// These exercise the wait_for_ack_ + cmd_seq_ machinery via the public
// request* API. Hardware is not available; we simulate the radar's ACK by
// pre-loading the MockSerial with the bytes the real radar would send.
//
// On the host autoReadTask is not active (no FreeRTOS available), so all
// commands take the polling branch of wait_for_ack_, which drains UART ->
// circular buffer -> read_frame_() -> parse_command_frame_() inline.
// ===========================================================================

// Test: requestFirmwareVersion full flow.
// Inject ACKs in the order the real radar would send: enter-config (0xFF),
// firmware-version (0xA0), leave-config (0xFE). All three must succeed and
// firmware_*_version fields must be populated.
static void test_command_ack_polling() {
    std::printf("test_command_ack_polling ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // enter_configuration_mode_ -> ACK 0xFF with 8-byte payload (status + version + buffer)
    // Stage three responses: each is released when the radar postamble is
    // written (i.e. immediately after the corresponding command bytes go out).
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_firmware_ack(1, 7, 0x16, 0x15, 0x09, 0x22));
    s.inject_response(make_short_ack(0xFE, 4));

    bool ok = r.requestFirmwareVersion();
    CHECK(ok);
    CHECK_EQ((int)r.firmware_major_version, 1);
    CHECK_EQ((int)r.firmware_minor_version, 7);
    CHECK_EQ((unsigned long)r.firmware_bugfix_version, 0x22091516UL);
    CHECK(!r.isAutoReadTaskRunning());
    std::printf("ok\n");
}

// Test: ACK with wrong opcode -> command must time out, returns false.
// Inject only enter-config ACK; the firmware command then receives no
// matching ACK and wait_for_ack_ should give up at radar_uart_command_timeout_.
static void test_command_ack_stale() {
    std::printf("test_command_ack_stale ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    // Only enter-config ACK; nothing for the actual command. wait_for_ack_
    // for 0xA0 will time out (radar_uart_command_timeout_ = 100 ms).
    s.inject_response(make_short_ack(0xFF, 8));

    // requestFirmwareVersion will: enter (ok), send 0xA0, then wait for 0xA0
    // ACK that never arrives. Should return false.
    bool ok = r.requestFirmwareVersion();
    CHECK(!ok);
    std::printf("ok\n");
}

// Test: two requestFirmwareVersion calls back-to-back. The cmd_seq_ counter
// must invalidate any leftover ACK state from call #1 so call #2 doesn't
// short-circuit on the previous response. We also vary the firmware bytes
// between the two calls to confirm the second value is what we read.
static void test_command_seq() {
    std::printf("test_command_seq ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // Call #1: major=1, minor=7
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_firmware_ack(1, 7, 0x16, 0x15, 0x09, 0x22));
    s.inject_response(make_short_ack(0xFE, 4));
    CHECK(r.requestFirmwareVersion());
    CHECK_EQ((int)r.firmware_major_version, 1);

    // Call #2: major=2, minor=10. Different bytes prove we are reading
    // the SECOND ACK not a residual from the first.
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_firmware_ack(2, 10, 0x01, 0x02, 0x03, 0x04));
    s.inject_response(make_short_ack(0xFE, 4));
    CHECK(r.requestFirmwareVersion());
    CHECK_EQ((int)r.firmware_major_version, 2);
    CHECK_EQ((int)r.firmware_minor_version, 10);
    CHECK_EQ((unsigned long)r.firmware_bugfix_version, 0x04030201UL);
    std::printf("ok\n");
}

// Test: setBaudRate(LD2410_BAUD_INDEX_256000) — opcode 0xA1.
// Inject ACKs in radar order (enter-config, set-baud, leave-config); all
// three must succeed. Closes regression vs v0.1.3 (upstream issue #39).
static void test_command_set_baud_rate() {
    std::printf("test_command_set_baud_rate ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0xA1, 4));
    s.inject_response(make_short_ack(0xFE, 4));

    bool ok = r.setBaudRate(LD2410_BAUD_INDEX_256000);
    CHECK(ok);
    std::printf("ok\n");
}

// Test: setBaudRate must report failure if the radar replies with a
// non-zero ACK status word. Same envelope as success but with
// status bytes set to 0xFF 0xFF.
static void test_command_set_baud_rate_failure() {
    std::printf("test_command_set_baud_rate_failure ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // Build a failure ACK: 0xA1 cmd-word, 0xFFFF status (instead of 0x0000).
    std::vector<uint8_t> bad_ack = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x04, 0x00,
        0xA1, 0x01,
        0xFF, 0xFF,                 // status: failure
        0x04, 0x03, 0x02, 0x01
    };
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(bad_ack);
    s.inject_response(make_short_ack(0xFE, 4));

    bool ok = r.setBaudRate(LD2410_BAUD_INDEX_57600);
    CHECK(!ok);
    std::printf("ok\n");
}

// Test: completely silent UART -> command must time out cleanly (no
// infinite loop, no field corruption).
static void test_command_no_ack() {
    std::printf("test_command_no_ack ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    // s is empty: no ACK at all.
    bool ok = r.requestFirmwareVersion();
    CHECK(!ok);
    // Fields must remain at their initial zero values.
    CHECK_EQ((int)r.firmware_major_version, 0);
    CHECK_EQ((int)r.firmware_minor_version, 0);
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 10: a lone 0xF4 followed by non-magic bytes must NOT commit to a
// frame. Pre-fix, the parser captured any 0xF4 as a frame start; if the
// next bytes happened to look like a length+payload, fields could be
// mis-populated. Post-fix, all four header bytes are validated.
// ---------------------------------------------------------------------------
static void test_lone_F4_not_a_header() {
    std::printf("test_lone_F4_not_a_header ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, false);
    s.inject({
        // Lone 0xF4 + three non-magic bytes, then a real frame.
        0xF4, 0x12, 0x34, 0x56,
        0xF4, 0xF3, 0xF2, 0xF1,
        0x0D, 0x00,
        0x02, 0xAA,
        0x02, 0x51, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00,
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.movingTargetDistance(), 81);
    CHECK_EQ((int)r.stationaryTargetEnergy(), 59);
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 11: when a partial header is broken by 0xF4 itself (e.g. radar emits
// F4 F3 F2 then a stray F4), the parser must reuse the F4 as a fresh
// position-0, not drop it. This exercises the resync branch where the
// offending byte is itself a candidate header.
// ---------------------------------------------------------------------------
static void test_resync_F4_at_wrong_position() {
    std::printf("test_resync_F4_at_wrong_position ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, false);
    s.inject({
        0xF4, 0xF3, 0xF2,                 // partial header, missing F1
        0xF4, 0xF3, 0xF2, 0xF1,           // real header (the F4 here is the resync)
        0x0D, 0x00,
        0x02, 0xAA,
        0x02, 0x51, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00,
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.movingTargetDistance(), 81);
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 12: the payload contains the data-frame footer pattern F8 F7 F6 F5
// in the middle. Pre-fix, the parser called check_frame_end_() after every
// byte from position 8 onwards and would terminate early when these four
// bytes appeared in payload, then reject the frame for length mismatch and
// lose alignment. Post-fix, the footer is only checked at the position
// dictated by the length field.
// ---------------------------------------------------------------------------
static void test_no_early_termination_on_payload_footer() {
    std::printf("test_no_early_termination_on_payload_footer ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x0D, 0x00,
        0x02, 0xAA,
        0x02,                             // target status
        0xF8, 0xF7,                       // moving distance LE = 0xF7F8 (intra bytes 3..4)
        0xF6,                             // moving energy = F6 (intra byte 5; getter clamps to 100)
        0xF5, 0x00,                       // stationary distance LE = 0x00F5
        0x00,                             // stationary energy
        0x42, 0x00,                       // detection distance = 0x0042 = 66 (sentinel)
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    // With the old parser, intra-bytes [F8 F7 F6 F5] at positions 9..12 of the
    // accumulated frame would trigger the per-byte footer check at position 13
    // and terminate the frame early; parse_data_frame_ would reject and the
    // detection_distance field (the last field, written only at full parse)
    // would stay at 0. The sentinel 66 above proves the new parser walked all
    // 23 bytes before validating the footer.
    drain(r, s);
    CHECK_EQ((int)r.movingTargetDistance(), 0xF7F8);
    CHECK_EQ((int)r.stationaryTargetDistance(), 0x00F5);
    CHECK_EQ((int)r.detectionDistance(), 66);
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// Test 13: a bogus length field rejects the frame and the parser resyncs in
// time to capture the next valid frame.
// ---------------------------------------------------------------------------
static void test_resync_after_bogus_length() {
    std::printf("test_resync_after_bogus_length ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0xFF, 0xFF,                       // intra length 65535 > LD2410_MAX_FRAME_LENGTH
        // Parser must drop and look for a fresh header in the bytes that follow.
        0xF4, 0xF3, 0xF2, 0xF1,
        0x0D, 0x00,
        0x02, 0xAA,
        0x02, 0x51, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00,
        0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.movingTargetDistance(), 81);
    std::printf("ok\n");
}

// ===========================================================================
// LD2410C-only command tests (Bluetooth, MAC address, distance resolution).
// Active when the build defines LD2410_HAS_* — i.e. when compiling with
// -DLD2410_VARIANT_C (gates set by ld2410_c.h). On default base builds
// these helpers do not exist, so they are excluded.
// ===========================================================================

#ifdef LD2410_HAS_BLUETOOTH
// 0xA4 setBluetooth(true) — standard 4-byte success ACK.
static void test_command_set_bluetooth_on() {
    std::printf("test_command_set_bluetooth_on ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0xA4, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    CHECK(r.setBluetooth(true));
    std::printf("ok\n");
}

// 0xA8 obtainBluetoothPermissions — the radar mirrors the ACK to UART
// only on some firmwares; mock pretends it does, so the host parses
// the 4-byte success envelope and returns true.
static void test_command_obtain_bluetooth_permissions() {
    std::printf("test_command_obtain_bluetooth_permissions ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0xA8, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    const uint8_t pwd[6] = {'H','i','L','i','n','k'};
    CHECK(r.obtainBluetoothPermissions(pwd));
    std::printf("ok\n");
}

// 0xA9 setBluetoothPassword — 4-byte success ACK on UART.
static void test_command_set_bluetooth_password() {
    std::printf("test_command_set_bluetooth_password ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0xA9, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    const uint8_t pwd[6] = {'H','i','L','i','n','k'};
    CHECK(r.setBluetoothPassword(pwd));
    std::printf("ok\n");
}
#endif

#ifdef LD2410_HAS_MAC_ADDRESS
// 0xA5 requestMACAddress — 10-byte intra ACK with 6-byte MAC at [10..15].
static void test_command_request_mac_address() {
    std::printf("test_command_request_mac_address ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    // MAC = 8F 27 2E B8 0F 65 (the example from HLK §2.2.13).
    std::vector<uint8_t> mac_ack = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x0A, 0x00,                                       // intra = 10
        0xA5, 0x01,                                       // cmd-word ACK
        0x00, 0x00,                                       // status: success
        0x8F, 0x27, 0x2E, 0xB8, 0x0F, 0x65,               // MAC bytes
        0x04, 0x03, 0x02, 0x01
    };
    s.inject_response(mac_ack);
    s.inject_response(make_short_ack(0xFE, 4));

    CHECK(r.requestMACAddress());
    CHECK_EQ((int)r.mac_address[0], 0x8F);
    CHECK_EQ((int)r.mac_address[1], 0x27);
    CHECK_EQ((int)r.mac_address[2], 0x2E);
    CHECK_EQ((int)r.mac_address[3], 0xB8);
    CHECK_EQ((int)r.mac_address[4], 0x0F);
    CHECK_EQ((int)r.mac_address[5], 0x65);
    std::printf("ok\n");
}
#endif

#ifdef LD2410_HAS_DISTANCE_RESOLUTION
// 0xAA setDistanceResolution(0.2 m) — standard 4-byte success ACK.
static void test_command_set_distance_resolution() {
    std::printf("test_command_set_distance_resolution ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0xAA, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    CHECK(r.setDistanceResolution(LD2410_DISTANCE_RESOLUTION_02M));
    std::printf("ok\n");
}

// 0xAB requestDistanceResolution — 6-byte intra ACK; index at [10..11] LE.
// HLK §2.2.17 example returns 01 00 → 0x0001 (= 0.2 m).
static void test_command_request_distance_resolution() {
    std::printf("test_command_request_distance_resolution ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    std::vector<uint8_t> get_ack = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x06, 0x00,                  // intra = 6
        0xAB, 0x01,                  // cmd-word ACK
        0x00, 0x00,                  // status: success
        0x01, 0x00,                  // index = 0x0001 (0.2 m)
        0x04, 0x03, 0x02, 0x01
    };
    s.inject_response(get_ack);
    s.inject_response(make_short_ack(0xFE, 4));

    CHECK(r.requestDistanceResolution());
    CHECK_EQ((int)r.distance_resolution, (int)LD2410_DISTANCE_RESOLUTION_02M);
    std::printf("ok\n");
}
#endif

#endif // !LD2410_VARIANT_S


#if defined(LD2410_VARIANT_S)
// ---------------------------------------------------------------------------
// S Test 1: standard data frame (HLK-LD2410S §2.1 Table 2-1).
//
// Layout (all offsets as full-frame indices after F4 F3 F2 F1):
//   [4-5]  intra-frame data length = 70 (LE: 0x46 0x00)
//   [6]    data type = LD2410_DATA_TYPE_STANDARD (0x01)
//   [7]    target state (0/1=no one, 2/3=present)
//   [8-9]  object distance (cm, LE)
//   [10-11] reserved bits
//   [12..43] motion energies for gates 0..15 (1 B each — first half of the
//            documented 64 B per-gate block; layout is unverified on HW)
//   [44..75] stationary energies for gates 0..15 + 16 B remainder unread
//   [76..79] frame end F8 F7 F6 F5
//
// The remaining 32 B of the 64 B per-gate block (offsets [60..75]) are
// padded with 0xFF to confirm parse_data_frame_ does NOT consume them.
// ---------------------------------------------------------------------------
static void test_s_standard_frame() {
    std::printf("test_s_standard_frame ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x46, 0x00,                                    // intra length = 70
        0x01,                                          // data type STANDARD
        0x02,                                          // target state = 2 (present)
        0xC8, 0x00,                                    // object distance = 200 cm
        0x00, 0x00,                                    // reserved bits
        // motion gates 0..15 (16 B)
        0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80,
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        // stationary gates 0..15 (16 B)
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
        // remaining 32 B of the 64 B documented block — must NOT be consumed
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);

    // Object distance is reported via detectionDistance() on S (single distance field).
    CHECK_EQ((int)r.detectionDistance(), 200);
    // base/C-only fields are zero on S.
    CHECK_EQ((int)r.movingTargetDistance(), 0);
    CHECK_EQ((int)r.movingTargetEnergy(), 0);
    CHECK_EQ((int)r.stationaryTargetDistance(), 0);
    CHECK_EQ((int)r.stationaryTargetEnergy(), 0);
    // Target state byte goes into target_type_, accessor exposed via presenceDetected().
    CHECK(r.presenceDetected());

    int expected_motion[16]     = {0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,
                                    0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
    int expected_stationary[16] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
                                    0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10};
    for (uint8_t g = 0; g < LD2410_GATE_COUNT; g++) {
        CHECK_EQ((int)r.movingEnergyAtGate(g),     expected_motion[g]);
        CHECK_EQ((int)r.stationaryEnergyAtGate(g), expected_stationary[g]);
    }
    // Out-of-range gate must return 0 (S has 16 gates, indexed 0..15).
    CHECK_EQ((int)r.movingEnergyAtGate(LD2410_GATE_COUNT), 0);
    CHECK_EQ((int)r.stationaryEnergyAtGate(255), 0);
    CHECK(r.engineeringRetrieved());

    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test 2: a frame with the wrong intra length (e.g. 35 = base/C eng size)
// must be rejected on -DLD2410_VARIANT_S.
// ---------------------------------------------------------------------------
static void test_s_rejects_basec_engineering_length() {
    std::printf("test_s_rejects_basec_engineering_length ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x23, 0x00,                                    // intra length = 35 (base/C engineering)
        0x01,                                          // data type STANDARD on S
        0xAA,                                          // 0xAA — base/C intra head; doesn't matter on S
        0x03, 0x1E, 0x00, 0x3C, 0x00, 0x00, 0x39, 0x00, 0x00,
        0x08, 0x08,
        0x3C, 0x22, 0x05, 0x03, 0x03, 0x04, 0x03, 0x06, 0x05,
        0x00, 0x00, 0x39, 0x10, 0x13, 0x06, 0x06, 0x08, 0x04,
        0x03, 0x05, 0x55, 0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    // Frame must be rejected — fields stay at their pre-frame defaults (0).
    CHECK_EQ((int)r.detectionDistance(), 0);
    CHECK(!r.engineeringRetrieved());
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test 3: auto-threshold progress frame (HLK-LD2410S §2.2.9, data type 0x03).
// Layout: F4 F3 F2 F1 | 03 00 (intra=3) | 03 (data type) | LO HI (progress LE)
//                     | F8 F7 F6 F5
// Total 13 bytes. The radar emits these continuously while the 0x09
// autoUpdateThreshold command is running.
// ---------------------------------------------------------------------------
static void test_s_auto_threshold_progress() {
    std::printf("test_s_auto_threshold_progress ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // Sanity: no progress before any frame is received.
    CHECK(!r.autoThresholdReceived());
    CHECK_EQ((int)r.autoThresholdProgress(), 0);

    // First progress frame: 5000 (= 50.00% in the radar's "progress×100"
    // encoding; 0x1388 LE = 0x88 0x13).
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x03, 0x00,                    // intra length = 3
        0x03,                          // data type AUTO_THRESHOLD
        0x88, 0x13,                    // progress = 0x1388 = 5000
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.autoThresholdProgress(), 5000);
    CHECK(r.autoThresholdReceived());

    // Second progress frame: 10000 (= 100.00% completed).
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x03, 0x00,
        0x03,
        0x10, 0x27,                    // 0x2710 = 10000
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.autoThresholdProgress(), 10000);

    // Standard frame interleaved must NOT touch the auto-threshold field.
    // Build a 70-byte intra standard frame with object distance = 123 cm.
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x46, 0x00,                                    // intra = 70
        0x01,                                          // STANDARD
        0x02,                                          // target state
        0x7B, 0x00,                                    // object distance = 123
        0x00, 0x00,
        // 64 bytes of per-gate energy (zeros + sentinel)
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.detectionDistance(), 123);
    CHECK_EQ((int)r.autoThresholdProgress(), 10000);   // last value, untouched
    CHECK(r.autoThresholdReceived());

    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test 4: a 0x03 frame with the wrong intra length must be rejected.
// ---------------------------------------------------------------------------
static void test_s_auto_threshold_rejects_wrong_length() {
    std::printf("test_s_auto_threshold_rejects_wrong_length ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x05, 0x00,                    // intra = 5 (WRONG; spec says 3)
        0x03,
        0xAA, 0xBB, 0xCC, 0xDD,        // 4 stray bytes
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK(!r.autoThresholdReceived());
    CHECK_EQ((int)r.autoThresholdProgress(), 0);
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test 5: minimal data frame (HLK-LD2410S §2.1 Table 2-1).
// Layout (5 bytes total): 6E | state (1B) | distance LE (2B) | 62
// Selected at runtime via setOutputMode(0x7A); the standard envelope
// (F4F3F2F1 / F8F7F6F5) does NOT apply.
// ---------------------------------------------------------------------------
static void test_s_minimal_frame() {
    std::printf("test_s_minimal_frame ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // Inject a present minimal frame: state=2, distance=200 cm.
    s.inject({ 0x6E, 0x02, 0xC8, 0x00, 0x62 });
    drain(r, s);
    CHECK(r.presenceDetected());
    CHECK_EQ((int)r.detectionDistance(), 200);
    CHECK_EQ((int)r.movingTargetDistance(), 0);   // base/C-only fields cleared
    CHECK_EQ((int)r.stationaryTargetDistance(), 0);

    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test 6: minimal frame with wrong tail must be rejected; the parser must
// resync and accept the next valid frame in the same stream.
// ---------------------------------------------------------------------------
static void test_s_minimal_rejects_wrong_tail() {
    std::printf("test_s_minimal_rejects_wrong_tail ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    s.inject({
        // bad: tail is 0x99 instead of 0x62 — frame discarded
        0x6E, 0x02, 0xFF, 0xFF, 0x99,
        // good: distance=42 cm, state=3 (present)
        0x6E, 0x03, 0x2A, 0x00, 0x62
    });
    drain(r, s);
    CHECK_EQ((int)r.detectionDistance(), 42);
    CHECK(r.presenceDetected());

    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test 7: minimal and standard frames can interleave. The parser must
// recognise both envelopes in the same stream without losing sync.
// ---------------------------------------------------------------------------
static void test_s_minimal_interleaved_with_standard() {
    std::printf("test_s_minimal_interleaved_with_standard ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);

    // Frame 1: minimal, distance=10, state=2.
    s.inject({ 0x6E, 0x02, 0x0A, 0x00, 0x62 });
    drain(r, s);
    CHECK_EQ((int)r.detectionDistance(), 10);

    // Frame 2: standard with distance=99, motion[0]=0x77, stationary[0]=0xCC.
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x46, 0x00,                                 // intra=70
        0x01, 0x02,                                 // STANDARD, state=2
        0x63, 0x00,                                 // dist=99
        0x00, 0x00,
        0x77, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,         // motion gates 0..15 (gate0=0x77)
        0xCC, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,         // stationary gates 0..15 (gate0=0xCC)
        // remaining 32 bytes of the documented 64 B per-gate block
        0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.detectionDistance(), 99);
    CHECK_EQ((int)r.movingEnergyAtGate(0), 0x77);
    CHECK_EQ((int)r.stationaryEnergyAtGate(0), 0xCC);

    // Frame 3: minimal again, distance=55. Standard-frame fields must
    // get the new state but the per-gate energies stay (minimal carries
    // no per-gate data — see parse_minimal_frame_ comment).
    s.inject({ 0x6E, 0x03, 0x37, 0x00, 0x62 });
    drain(r, s);
    CHECK_EQ((int)r.detectionDistance(), 55);
    CHECK_EQ((int)r.movingEnergyAtGate(0), 0x77);   // unchanged
    CHECK_EQ((int)r.stationaryEnergyAtGate(0), 0xCC); // unchanged

    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test 8: setOutputMode(true) — switch to standard frame envelope.
// 0x7A §2.2.1, send intra=8 (cmd-word + 6-byte payload), 4-byte ACK.
// ---------------------------------------------------------------------------
static void test_s_set_output_mode_standard() {
    std::printf("test_s_set_output_mode_standard ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0x7A, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    CHECK(r.setOutputMode(/*standard=*/true));
    std::printf("ok\n");
}

static void test_s_set_output_mode_minimal() {
    std::printf("test_s_set_output_mode_minimal ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0x7A, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    CHECK(r.setOutputMode(/*standard=*/false));
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test: writeGenericParameters — 0x70 ACK is standard 4-byte envelope.
// ---------------------------------------------------------------------------
static void test_s_write_generic_parameters() {
    std::printf("test_s_write_generic_parameters ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0x70, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    // Use the §2.2.7 example values: farthest=12, nearest=0, delay=40s,
    // status=5 (0.5 Hz), distance=5 (0.5 Hz), speed=5 (normal).
    CHECK(r.writeGenericParameters(12, 0, 40, 5, 5, 5));
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test: requestGenericParameters — 0x71 ACK is intra=28 with 6 LE 4-byte
// values; uses the §2.2.8 documented example payload.
// ---------------------------------------------------------------------------
static void test_s_request_generic_parameters() {
    std::printf("test_s_request_generic_parameters ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));

    std::vector<uint8_t> ack = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x1C, 0x00,                      // intra = 28
        0x71, 0x01,                      // cmd-word ACK
        0x00, 0x00,                      // status: success
        0x0C, 0x00, 0x00, 0x00,          // farthest = 12
        0x00, 0x00, 0x00, 0x00,          // nearest  = 0
        0x28, 0x00, 0x00, 0x00,          // unmanned delay = 40 s
        0x05, 0x00, 0x00, 0x00,          // status freq    = 5  (0.5 Hz)
        0x05, 0x00, 0x00, 0x00,          // distance freq  = 5  (0.5 Hz)
        0x05, 0x00, 0x00, 0x00,          // response speed = 5  (normal)
        0x04, 0x03, 0x02, 0x01
    };
    s.inject_response(ack);
    s.inject_response(make_short_ack(0xFE, 4));

    CHECK(r.requestGenericParameters());
    CHECK_EQ((int)r.detect_farthest_gate, 12);
    CHECK_EQ((int)r.detect_nearest_gate,   0);
    CHECK_EQ((int)r.unmanned_delay_s,     40);
    CHECK_EQ((int)r.status_report_freq,    5);
    CHECK_EQ((int)r.distance_report_freq,  5);
    CHECK_EQ((int)r.response_speed,        5);
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test: writeTriggerThresholds — 0x72 ACK is standard 4-byte envelope.
// ---------------------------------------------------------------------------
static void test_s_write_trigger_thresholds() {
    std::printf("test_s_write_trigger_thresholds ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0x72, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    // §2.2.10 example values
    const uint8_t trig[16] = {50,46,34,32,32,32,32,32, 50,46,34,32,32,32,32,32};
    CHECK(r.writeTriggerThresholds(trig));
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test: requestTriggerThresholds — 0x73 ACK is intra=68 with 16 LE 4B values.
// Uses the §2.2.11 documented payload.
// ---------------------------------------------------------------------------
static void test_s_request_trigger_thresholds() {
    std::printf("test_s_request_trigger_thresholds ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    std::vector<uint8_t> ack = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x44, 0x00,                  // intra = 68
        0x73, 0x01,                  // cmd-word ACK
        0x00, 0x00,                  // status: success
    };
    const uint8_t expected[16] = {0x32,0x2E,0x22,0x20,0x20,0x20,0x20,0x20,
                                  0x32,0x2E,0x22,0x20,0x20,0x20,0x20,0x20};
    for (int g = 0; g < 16; g++) {
        ack.push_back(expected[g]); ack.push_back(0x00);
        ack.push_back(0x00);        ack.push_back(0x00);
    }
    ack.push_back(0x04); ack.push_back(0x03); ack.push_back(0x02); ack.push_back(0x01);
    s.inject_response(ack);
    s.inject_response(make_short_ack(0xFE, 4));

    CHECK(r.requestTriggerThresholds());
    for (int g = 0; g < 16; g++) {
        CHECK_EQ((int)r.trigger_thresholds[g], (int)expected[g]);
    }
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test: writeHoldThresholds — 0x76 ACK is standard 4-byte envelope.
// ---------------------------------------------------------------------------
static void test_s_write_hold_thresholds() {
    std::printf("test_s_write_hold_thresholds ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0x76, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    const uint8_t hold[16] = {15,15,15,15,15,15,15,15, 9,9,9,9,9,9,9,9};
    CHECK(r.writeHoldThresholds(hold));
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test: requestHoldThresholds — 0x77 ACK intra=68; uses §2.2.13 example.
// ---------------------------------------------------------------------------
static void test_s_request_hold_thresholds() {
    std::printf("test_s_request_hold_thresholds ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    std::vector<uint8_t> ack = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x44, 0x00,
        0x77, 0x01,
        0x00, 0x00,
    };
    const uint8_t expected[16] = {0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,
                                  0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09};
    for (int g = 0; g < 16; g++) {
        ack.push_back(expected[g]); ack.push_back(0x00);
        ack.push_back(0x00);        ack.push_back(0x00);
    }
    ack.push_back(0x04); ack.push_back(0x03); ack.push_back(0x02); ack.push_back(0x01);
    s.inject_response(ack);
    s.inject_response(make_short_ack(0xFE, 4));

    CHECK(r.requestHoldThresholds());
    for (int g = 0; g < 16; g++) {
        CHECK_EQ((int)r.hold_thresholds[g], (int)expected[g]);
    }
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test: autoUpdateThreshold — firmwares that emit a 4-byte ACK on 0x09.
// ---------------------------------------------------------------------------
static void test_s_auto_update_threshold_with_ack() {
    std::printf("test_s_auto_update_threshold_with_ack ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    s.inject_response(make_short_ack(0x09, 4));
    s.inject_response(make_short_ack(0xFE, 4));
    CHECK(r.autoUpdateThreshold());           // defaults from ld2410_s.h
    std::printf("ok\n");
}

// ---------------------------------------------------------------------------
// S Test: autoUpdateThreshold — firmwares that do NOT ACK on 0x09 (per HLK).
// Method must return false (no ACK), but the call should complete cleanly
// without corrupting state. Also exercises that explicit factor/scan args
// compile through the default-arguments signature.
// ---------------------------------------------------------------------------
static void test_s_auto_update_threshold_no_ack() {
    std::printf("test_s_auto_update_threshold_no_ack ... ");
    ld2410 r;
    MockSerial s;
    r.begin(s, /*waitForRadar=*/false);
    s.inject_response(make_short_ack(0xFF, 8));
    // No 0x09 ACK injected; only enter/leave-config respond.
    s.inject_response(make_short_ack(0xFE, 4));
    CHECK(!r.autoUpdateThreshold(/*trigger=*/3, /*retention=*/2, /*scan_time_s=*/60));
    // Receiving a 0x03 progress frame after the call must still update
    // the field — verifies state is intact post-call.
    s.inject({
        0xF4, 0xF3, 0xF2, 0xF1,
        0x03, 0x00,
        0x03,
        0x88, 0x13,                       // 5000 = 50.00%
        0xF8, 0xF7, 0xF6, 0xF5
    });
    drain(r, s);
    CHECK_EQ((int)r.autoThresholdProgress(), 5000);
    std::printf("ok\n");
}
#endif   // LD2410_VARIANT_S

int main() {
#if !defined(LD2410_VARIANT_S)
    test_basic_frame();
    test_engineering_frame();
    test_invalid_data_type();
    test_invalid_trailer();
    test_sequential_frames();
    test_command_ack_polling();
    test_command_ack_stale();
    test_command_seq();
    test_command_no_ack();
    test_command_set_baud_rate();
    test_command_set_baud_rate_failure();
#ifdef LD2410_HAS_BLUETOOTH
    test_command_set_bluetooth_on();
    test_command_obtain_bluetooth_permissions();
    test_command_set_bluetooth_password();
#endif
#ifdef LD2410_HAS_MAC_ADDRESS
    test_command_request_mac_address();
#endif
#ifdef LD2410_HAS_DISTANCE_RESOLUTION
    test_command_set_distance_resolution();
    test_command_request_distance_resolution();
#endif
    test_lone_F4_not_a_header();
    test_resync_F4_at_wrong_position();
    test_no_early_termination_on_payload_footer();
    test_resync_after_bogus_length();
#else
    test_s_standard_frame();
    test_s_rejects_basec_engineering_length();
    test_s_auto_threshold_progress();
    test_s_auto_threshold_rejects_wrong_length();
    test_s_minimal_frame();
    test_s_minimal_rejects_wrong_tail();
    test_s_minimal_interleaved_with_standard();
    test_s_set_output_mode_standard();
    test_s_set_output_mode_minimal();
    test_s_write_generic_parameters();
    test_s_request_generic_parameters();
    test_s_write_trigger_thresholds();
    test_s_request_trigger_thresholds();
    test_s_write_hold_thresholds();
    test_s_request_hold_thresholds();
    test_s_auto_update_threshold_with_ack();
    test_s_auto_update_threshold_no_ack();
#endif

    if (failures == 0) {
        std::printf("\nALL TESTS PASS\n");
        return 0;
    }
    std::printf("\n%d FAILURE(S)\n", failures);
    return 1;
}
