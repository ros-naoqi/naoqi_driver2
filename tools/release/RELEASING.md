# Releasing the naoqi / qi ROS 2 stack

This runbook (re)releases the qi-related packages onto the active ROS 2 distros
via [bloom]. It covers six upstream repos and their `*-release` counterparts.

| Upstream repo | Package(s) | Devel branch | Release repo |
|---|---|---|---|
| `ros-naoqi/libqi` | `naoqi_libqi` | `ros2` | `libqi-release` |
| `ros-naoqi/libqicore` | `naoqi_libqicore` | `ros2` | `libqicore-release` |
| `ros-naoqi/naoqi_bridge_msgs2` | `naoqi_bridge_msgs` | `main` | `naoqi_bridge_msgs2-release` |
| `ros-naoqi/naoqi_driver2` | `naoqi_driver` | `main` | `naoqi_driver2-release` |
| `ros-naoqi/nao_meshes2` | `nao_meshes` | `main` | `nao_meshes-release` |
| `ros-naoqi/pepper_meshes2` | `pepper_meshes` | `main` | `pepper_meshes2-release` |

**Active target distros:** Humble, Jazzy, Kilted, Rolling. (Iron, Foxy,
Galactic and ROS 1 Noetic are EOL — rosdistro will not accept new releases.)

**Platforms:** Humble → Ubuntu 22.04 (jammy); Jazzy/Kilted → Ubuntu 24.04
(noble); Rolling → **Ubuntu 26.04 (resolute)** since the 2026-04 sync (it left
24.04 behind). The buildfarm is the source of truth for Rolling — local CI on a
stale Ubuntu will show false failures.

> **Note — `geometry_msgs/Pose2D` removed in Rolling.** `naoqi_bridge_msgs`
> used to reference `geometry_msgs/Pose2D` in `CmdPoseService.srv`; that type
> was removed in Rolling. As of `naoqi_bridge_msgs` 2.1.1 it ships its own
> field-compatible `naoqi_bridge_msgs/Pose2D` instead, so it builds on all
> distros. The two upstream commits are preserved as patches under
> `tools/release/patches/` in case the bridge_msgs checkout needs rebuilding.

## Prerequisites

* A GitHub **bloom token** (PAT) belonging to a `ros-naoqi` member with write
  access to the upstream + `*-release` repos, able to fork/PR `ros/rosdistro`.
* Network egress to `github.com`, `api.github.com` and `raw.githubusercontent.com`
  (`api.github.com` is required for bloom to open the rosdistro pull requests).
* Tooling: `pip install "empy==3.3.4" bloom catkin_pkg rosdistro rosdep`.

```bash
git config --global user.name  "Your Name"
git config --global user.email "you@example.com"
# Let git authenticate pushes with the bloom token:
git config --global url."https://x-access-token:${BLOOM_TOKEN}@github.com/".insteadOf "https://github.com/"
```

## Step 1 — bump the upstream packages (only where source moved past last tag)

For each repo with commits beyond its newest tag, on its devel branch:

```bash
catkin_generate_changelog -y            # populate the Forthcoming section
# review CHANGELOG.rst, then:
python3 tools/release/finalize_release.py . <new_version>
git push origin <devel_branch>
git push origin <new_version>           # the tag
```

Last computed round (June 2026):

| Package | Old → New | Notes |
|---|---|---|
| `naoqi_libqicore` | 3.0.0 → **3.0.1** | ships "Support for Jazzy" |
| `naoqi_bridge_msgs` | 2.1.0 → **2.1.1** | "Support for Jazzy" + CI + local `Pose2D` (Rolling fix) |
| `naoqi_driver` | 2.1.1 → **2.1.2** | ships Jazzy cv_bridge fix + more |
| `nao_meshes` | 2.1.1 → **2.1.2** | "Remove java dependency" |
| `naoqi_libqi` | 3.0.3 (no bump) | source == latest tag |
| `pepper_meshes` | 3.0.0 (no bump) | source == latest tag |

## Step 2 — ensure bloom tracks exist for every target distro

In each `*-release` checkout (tracks land on its `master` branch):

```bash
git clone https://github.com/ros-naoqi/<repo>-release.git && cd <repo>-release
python3 tools/release/setup_tracks.py --devel <ros2|main> --distros jazzy kilted rolling humble
git commit -am "Add/refresh bloom tracks"
git push origin master
```

The generated tracks use `version: :{auto}` + `release_tag: :{version}`, so the
release below is non-interactive and always matches the upstream tag.

## Step 3 — release each package onto each distro

```bash
for d in humble jazzy kilted rolling; do
  bloom-release --rosdistro $d --track $d <package_name>
done
```

`<package_name>` is the rosdistro key (`naoqi_libqi`, `naoqi_libqicore`,
`naoqi_bridge_msgs`, `naoqi_driver`, `nao_meshes`, `pepper_meshes`). bloom finds
the `*-release` repo from existing rosdistro entries, imports the upstream tag,
generates the platform debians/rpms, pushes to the release repo, and opens a
`ros/rosdistro` PR. Distro coverage for the current round:

* `naoqi_libqi` → humble, kilted, rolling (jazzy already at 3.0.3)
* `naoqi_libqicore`, `naoqi_bridge_msgs`, `naoqi_driver`, `nao_meshes` → humble, jazzy, kilted, rolling
* `pepper_meshes` → jazzy, kilted, rolling (humble already current)

## Step 4 — merge the rosdistro PR(s)

Each release opens (or updates) a PR on `ros/rosdistro`. Once CI there is green
and it's merged, the buildfarm builds the binaries; the next sync publishes them
to `packages.ros.org`. Watch status at <https://build.ros2.org>.

[bloom]: https://github.com/ros-infrastructure/bloom
