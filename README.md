# perf_counters
gputop like AMD perf counters tool.

You can build it with a standard meson flow and then run it while passing the counter
names as arguments. It will print various counters over an 1-second interval.

Currently only tested on a vega 64 and RX 5700 series.

The counters are a mix, I still need to separate them out. The CU ones are Vega only and the VRAM/L2 counters are Navi only.



