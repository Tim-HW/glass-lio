# Third-party components

glasslio itself is [MIT](LICENSE). This file lists everything in, or used by, this
repository that is **not** ours, and what it is licensed under.

## Bundled in this repository

| Component | Where | Licence | Notes |
|---|---|---|---|
| **Sophus** | [`glass_core/include/sophus/`](glass_core/include/sophus/) | MIT | Lie group primitives (SO(3), SE(3), `Exp`/`Log`). Vendored as headers. © Hauke Strasdat, Steven Lovegrove. See [`glass_core/include/sophus/LICENSE`](glass_core/include/sophus/LICENSE). Upstream: <https://github.com/strasdat/Sophus> |

That is the only third-party *code* in the tree. Everything else under `src/`,
`include/glasslio/`, `test/`, `doc/`, `config/`, `scripts/` and `docker/` is ours and is
MIT.

## Fetched at runtime, not bundled

| Component | Licence | Notes |
|---|---|---|
| **Test dataset** — *Driving SLAM Test with Livox MID360* | **CC-BY-4.0** | Not in the repo (1.4 GB); [`scripts/download_bag.sh`](scripts/download_bag.sh) fetches it from Zenodo. **Attribution is required if you redistribute or publish results from it.** |

> Koide, K. (2025). *Driving SLAM Test with Livox MID360* [Data set]. Zenodo.
> <https://doi.org/10.5281/zenodo.14841855>

## Build dependencies (linked, not bundled)

These are ordinary dependencies — installed by `rosdep`/apt, not copied into this repo. They
do not constrain glasslio's licence, but you inherit their terms in anything you *ship*:

| Component | Licence |
|---|---|
| ROS 2 (Jazzy) — `rclcpp`, `sensor_msgs`, `tf2_*`, `pcl_conversions` | Apache-2.0 |
| Eigen 3 | MPL-2.0 (the parts used here) |
| PCL (common / io / filters) | BSD-3-Clause |

## If you fork or vendor this

Two obligations, and only two:

1. **Keep [`LICENSE`](LICENSE)** — MIT's single condition is that the copyright notice
   travels with the code.
2. **Keep [`glass_core/include/sophus/LICENSE`](glass_core/include/sophus/LICENSE)** if you keep the Sophus
   headers. The same condition applies to it, and it is not ours to waive.

If you use the test bag in a paper or a demo, cite Koide (2025) above — that one is a
licence term, not a courtesy.
