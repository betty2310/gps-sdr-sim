we research about topic GPS spoofing detection, which target on dataset mix of authentic and spoofed.
we use gps-sdr-sim as the tool to create this dataset mix.
the target is inject non-visible satellites into real gps sigal from real sky, and want the u-blox receiver track both, not only track,
but also "use" satellites from both group to solve PVT (mean the `svUsed` in u-blox nav message is marked).
