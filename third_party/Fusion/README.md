# Fusion AHRS subset

Unmodified x-io Technologies Fusion files from revision
`a8d7224f36a0ec82345ef49a3db50e65f8d3bab8`:

https://github.com/xioTechnologies/Fusion/tree/a8d7224f36a0ec82345ef49a3db50e65f8d3bab8

The MIT license is retained in LICENSE.md. UPSTREAM.json records SHA-256 hashes
of all downloaded upstream files. No calibration or separate bias estimator is
used. Local application behavior lives in ../../dcs_attitude.c.

Verify the downloaded bytes offline:

```sh
python tools/vendor_fusion.py --verify
```

The command above runs from the repository root. Running the script without
`--verify` downloads this same fixed revision again and needs network access.

Compile dcs_attitude.c and Fusion/FusionAhrs.c with
`FUSION_USE_NORMAL_SQRT` defined for both. This selects upstream's supported
standard square-root option without modifying its sources. The supplied host
test script applies that definition consistently. FusionConvention.c is also
included for users of its convention-name helper.
