# Installed sensor axes and lever arm

All directions below are viewed from the top. The physical `Body` frame is a
mechanical drawing frame and is not the payload frame used by attitude and
guidance.

## Frames

- Physical Body: +X up the drawing, +Y right, +Z into the drawing; right-handed.
- Payload: +X along the parafoil front, +15 degrees clockwise/right of physical Body +X;
  +Y 90 degrees right of payload +X; +Z into the drawing; right-handed.
- BMI270: +X 45 degrees left of physical Body +X; +Y a further 90 degrees left;
  +Z out of the drawing; right-handed.
- IIS2MDC: +X 45 degrees right of physical Body +X; +Y a further 90 degrees
  right; +Z out of the drawing; the labelled frame is left-handed.

The attitude module always consumes the payload frame: +X parafoil front,
+Y right and +Z down/into the drawing.

## Required processing order

```text
BMI raw -> BMI-native calibration -> BMI-to-payload mapping
IIS2MDC raw -> offset/matrix calibration in IIS2MDC axes -> mag-to-payload mapping
```

Do not rotate the magnetometer before applying the supplied calibration. Its
offset and 3x3 mapping were identified in the native IIS2MDC coordinates.

`dcs_sensor_transform` implements both mappings. The BMI mapping applies to
both gyro and accelerometer vectors. Flight integration should call
`dcs_attitude_update_installed`; it performs these mappings and then invokes the
payload-frame Fusion update. `dcs_attitude_update` remains the lower-level API
for data already expressed in payload axes.

## BMI lever arm

The BMI position is `(7.157, 40.885) mm` in physical-Body X/Y coordinates,
interpreting that frame's origin as the cylinder axis. In payload coordinates:

```text
r_BMI = (+17.4949, +37.6395, 0) mm
|r_BMI| = 41.5067 mm
```

For rigid rotation about physical Body/payload +Z, the implemented transfer is:

```text
a_sensor = a_origin + alpha x r + omega x (omega x r)
a_origin = a_sensor - alpha x r - omega x (omega x r)
```

The equation applies equally to accelerometer specific force because gravity is
effectively the same at the two points. Call
`dcs_bmi_accel_to_body_origin_payload` with payload/Body `omega_z` and `alpha_z`.
Since BMI +Z points out of the drawing, `omega_z = -calibrated_bmi_gyro_z`.
Estimate alpha from timestamped gyro samples only if its noise is acceptable;
passing zero deliberately omits tangential correction.

This correction is valid only for the assumed rigid cylinder-axis rotation. A
parafoil and suspended CanSat form a flexible coupled system, so there is no
known fixed combined rotation point. Do not reinterpret the physical-Body
origin as that point without measurement or system identification.

## Initial heading consequence

Attitude initialization uses the payload +X direction, not physical Body +X.
If physical Body +X points true North, the initial payload heading is +15
degrees. A configured zero heading requires payload/parafoil +X
itself to point true North.
