# Parent visual review

The real-source startup image shows a blue real-hardware badge, Disconnected status, a blank port selection, zero reply counters and disabled arm controls. The demo-source image shows its amber no-hardware badge, Disconnected status and disabled arm controls. Both use the U2D2 adapter profile. The dark position-test image clearly identifies simulation and shows measured and requested position curves with live telemetry. No visual discrepancy was found. These images do not validate the new core on hardware.

The first sandbox baseline failed the existing ownership guard because sandbox /tmp has UID65534. Host /tmp has UID0 and mode1777. The audited fake/PTY-only baseline passed with host filesystem access; production ownership checks were not relaxed.
