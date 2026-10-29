# Archived NEO-6M GNSS driver

Snapshot before migration to Quectel L89HA-S90. These files are unused by all
current CMake targets. GNSS.c/GNSS.h contain the old u-blox UBX configuration,
GGA/VTG parser, optional GSV and NAV-VELNED/TIMEUTC profiles. GNSS.md documents
that old hardware only. Test sources are kept here for reference.

Do not compile this GNSS.c together with Core/Src/drivers/GNSS.c: both own the
same UART callbacks and API names. Restoring this driver requires restoring its
matching header and explicit build selection. Do not send its UBX commands to
the Quectel module.
