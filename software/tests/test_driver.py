import pytest

from hexapod.driver import (SET_CMD, GET_CMD, IDX_RELAY, encode_set, encode_get,
                            decode_get_response, MockDriver)


def test_encode_set_bytes_exact():
    # 1500µs = 0b101_1101_1100 -> 低7位 0x5C, 高7位 0x0B
    pkt = encode_set(0, [1500])
    assert pkt == bytes([SET_CMD, 0, 1, 0x5C, 0x0B])


def test_encode_set_multi():
    pkt = encode_set(3, [1000, 2000])
    assert pkt[0] == SET_CMD and pkt[1] == 3 and pkt[2] == 2
    assert len(pkt) == 3 + 2 * 2
    assert (pkt[3] | (pkt[4] << 7)) == 1000
    assert (pkt[5] | (pkt[6] << 7)) == 2000


def test_encode_get():
    assert encode_get(18, 6) == bytes([GET_CMD, 18, 6])


def test_decode_get_response_roundtrip():
    vals = [1023, 0, 512]
    buf = bytes([GET_CMD, 18, 3]) + b"".join(
        bytes([v & 0x7F, (v >> 7) & 0x7F]) for v in vals)
    assert decode_get_response(buf, 18, 3) == vals


def test_decode_get_response_bad_header():
    with pytest.raises(IOError):
        decode_get_response(bytes([0x00, 18, 1, 0, 0]), 18, 1)


def test_mock_driver_state():
    d = MockDriver()
    d.set_all_pulses_us([1500] * 18)
    d.set_pulses_us(2, [1800])
    assert d.pulses[2] == 1800 and d.pulses[3] == 1500
    d.enable(True)
    assert d.enabled
    d.touch_raw[0] = 1000
    assert d.read_touch()[0] and not d.read_touch()[1]
