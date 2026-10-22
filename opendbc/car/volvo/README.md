# Volvo longitudinal control

Port of Leomonde's base `volvo_oplong` implementation:

- Sunnypilot: `031520683625fb2d518bffdeda430521ca78d690`
- Referenced opendbc: [`2572246bc32a4a903cc0cd5610ea0deeae15203c`](https://github.com/leomonde/opendbc/tree/2572246bc32a4a903cc0cd5610ea0deeae15203c)

The longitudinal decision, resume, handover and relay methods, and the entire
CAN builder module match that source. This replaces the earlier selective port.
Enable **Alpha Longitudinal Control** in the developer settings to use OP long;
`openpilotLongitudinalControl` now follows that setting as in the source.

## Behavior

- FSM3 acceleration, brake/coast/drive mode and traction are controlled together.
- FSM4 hydraulic-brake flags and the 14-bit target use the source's braking and
  non-braking lookup tables, including its camera-target blending.
- OP can ignore a camera hydraulic-brake request while it remains at or above
  -2.0 m/s². A request strictly below -2.0 m/s² latches camera-brake honoring until
  the camera releases its brake flag. OP can still request stronger braking.
  This is an acceleration threshold, not a decoded emergency/AEB classification.
- While ignoring a camera request, camera-bound 0x220 and 0x299 acknowledgements
  are fabricated. The 0x299 acknowledgement releases when the native camera's
  target reaches 13280; 0x220 remains until the camera releases its brake flag.
- When OP brakes without the camera requesting it, those two acknowledgement
  bits are masked, with the source's 100-control-frame tail. 0x133 and 0x323 are
  relayed unchanged with the default configuration.
- Behind a stock-recognized lead, the camera takes over the final stop below
  10 km/h, blended over one second. OP takes back control above 15 km/h or on
  target loss while moving. The source's stronger-OP-braking override is retained.
- Resume uses 15-button bursts, up to five attempts, on any increase in stock
  lead distance or a sustained 0.5-second OP resume request. FSM3 ACC_Check is
  asserted for 25 longitudinal transmissions after the first attempt.
- ECM engagement is state 3 of the two-bit ACC state. Camera standstill is gated
  by actual vehicle standstill. Reported actuator acceleration reflects the
  controller's selected acceleration, including stock-camera handover.
- Longitudinal delay and integral gain are both 0.5, as in the source.

The source's `OP_OWNS_ACC_AFTER_CAMERA_ABORT` code is present but remains disabled,
matching its default. No radar simulation, no-target stop, speed-floor spoofing,
or fake camera buttons are imported from `volvo_oplong_custom`.

## Integration differences

This is a stock-openpilot port, not a replacement with the complete Sunnypilot
application. CP_SP/CC_SP, Sunnypilot ICBM/custom-set-speed settings and its TSR UI
extension are not available here; their application-specific plumbing is omitted.
Existing local steering calibration, measured-angle inactive behavior, cluster
speed scaling and verified parking-brake decoding are retained. These are not
changes to the imported longitudinal arbitration or CAN encoding algorithms.

Panda's forwarding/TX policy is ported from the same source, with local steering
API/angle quantization adaptations. It permanently blocks native FSM0/FSM4 and
the four camera-bound feedback messages because the host relays every received
frame, including while OP long is inactive. FSM1/FSM3 native forwarding is blocked
only while controls are allowed and gas is clear. FSM3 requests are range-checked
(-4 to +2 m/s²), and FSM0 cannot assert engagement without ECM engagement.

The previous selective port's 100 ms relay lease, immutable-byte checks and
"never weaken camera braking" restriction are not part of this source policy and
have been removed. This version does not claim those protections or independent
Panda enforcement of the -2.0 m/s² arbitration threshold; that arbitration is in
the host controller. Matching rebuilt Panda firmware is required. Nothing is
flashed automatically by this source change.

## Validation

Run from the opendbc repository:

```sh
python -m pytest -q opendbc/car/volvo/tests opendbc/safety/tests/test_volvo.py
python -m pytest -q opendbc/car/tests/test_car_interfaces.py -k VOLVO
ruff check opendbc/car/volvo opendbc/safety/tests/test_volvo.py
```

Coverage includes threshold boundaries (-1.96, -2.0, -2.04), brake-honor latching,
fabricated acknowledgement release, unchanged feedback bytes, stop handover,
resume burst shape, native batch cadence, disabled relays, ECM engagement,
forwarding policy and a 601-point command sweep through compiled Panda safety.
Eight inherited safety tests are skipped by the existing test framework.

Source comparison verified all eight longitudinal/resume/relay methods and the
complete CAN builder module are AST-identical to the pinned source. Tests do not
establish physical braking performance or vehicle acceptance. No road test or
firmware deployment has been performed.
