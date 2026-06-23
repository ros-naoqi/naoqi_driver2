# Releasing the qi stack into ROS 2 Rolling (ros2-gbp)

Rolling is a special case: **release repositories for Rolling must live in the
[`ros2-gbp`](https://github.com/ros2-gbp) GitHub organization**, not in
`ros-naoqi`. Humble/Jazzy/Kilted were released from `ros-naoqi/<repo>-release`
and merged fine; only Rolling enforces `ros2-gbp`.

This was surfaced on rosdistro PRs **#52234–#52237** (libqi, naoqi_bridge_msgs2,
nao_meshes, pepper_meshes → rolling), which were `CHANGES_REQUESTED` by the
automated reviewer and maintainer `ahcorde`:

> ❌ Release repositories for rolling must be hosted in the ros2-gbp organization
> — This URL should start with `https://github.com/ros2-gbp/`

## Packages needing a `ros2-gbp` release repo

| rosdistro key | current (ros-naoqi) | target (ros2-gbp) | wave |
|---|---|---|---|
| `naoqi_libqi` | libqi-release | ros2-gbp/libqi-release | 1 |
| `naoqi_bridge_msgs2` | naoqi_bridge_msgs2-release | ros2-gbp/naoqi_bridge_msgs2-release | 1 |
| `nao_meshes` | nao_meshes-release | ros2-gbp/nao_meshes-release | 1 |
| `pepper_meshes` | pepper_meshes2-release | ros2-gbp/pepper_meshes2-release | 1 |
| `naoqi_libqicore` | libqicore-release | ros2-gbp/libqicore-release | 2 |
| `naoqi_driver` | naoqi_driver2-release | ros2-gbp/naoqi_driver2-release | 2 |

## Step 1 — Provision the `ros2-gbp` release repos (maintainer + ROS 2 infra)

These cannot be created with a normal PAT; they are provisioned by the ROS 2
infrastructure team. Reply on one of the rolling PRs (#52234–#52237) asking
`ahcorde` / the ROS 2 infra team for the `ros2-gbp` onboarding, and pick one:

- **(a) New empty `ros2-gbp/<repo>-release` repos** + push access for the
  maintainer. Recommended — leaves the `ros-naoqi` repos (used by the merged
  Humble/Jazzy/Kilted entries) untouched.
- **(b) Transfer the existing `ros-naoqi/<repo>-release` repos into `ros2-gbp`.**
  GitHub leaves redirects, so the already-merged stable-distro entries keep
  resolving. More future-proof (all distros end up on ros2-gbp) but moves the
  stable-distro release repos too.

## Step 2 — Re-release Rolling against `ros2-gbp`

Once the `ros2-gbp/<repo>-release` repo exists and you can push to it, for each
Rolling package (deps first: libqi → libqicore → driver; the leaves any time):

```bash
export PATH="$PWD/bloom-venv/bin:$PATH"; export HOME=/root   # see RELEASING.md
bloom-release -y --no-web \
  --override-release-repository-url https://github.com/ros2-gbp/<repo>-release.git \
  -r rolling -t rolling <rosdistro_key>
```

Notes:
- `--override-release-repository-url` makes bloom both push the Rolling
  release branches/tags into the `ros2-gbp` repo **and** write that URL into the
  rosdistro entry (which is exactly what the reviewer asked for).
- A new repo is empty, so bloom regenerates everything; no migration of the old
  `ros-naoqi` release content is needed.
- New-distro entries prompt for source/doc/status — drive with the pexpect
  helper from `RELEASING.md` (accept the `:{auto}` track defaults, status
  `maintained`).
- If route (b) was used, also update the Rolling track's `release_repo_url` /
  push URL to `ros2-gbp` instead of using the override.

## Step 3 — Supersede the old Rolling PRs

The new `bloom-release` run opens fresh rosdistro PRs whose `release.url` points
at `ros2-gbp`. Close the superseded #52234–#52237, link the new PRs, and
re-request review from `ahcorde`; queue once green (`@mergifyio queue`).

## Step 4 — Wave-2 Rolling

`naoqi_libqicore` and `naoqi_driver` can only be released to Rolling once their
dependencies (`naoqi_libqi`, and for the driver also `naoqi_libqicore` +
`naoqi_bridge_msgs`) are **merged** into Rolling's rosdistro. Release them the
same way (Step 2) after the Wave-1 Rolling PRs land.
