#!/usr/bin/env python3
"""Add/refresh deterministic bloom tracks in a *-release repo checkout.

Copies an existing track (preferring ``humble``) as a template and creates one
track per requested distro, configured so ``bloom-release`` runs unattended:
the version is read from the upstream package.xml (``:{auto}``) and the upstream
git tag is taken to equal that version (``:{version}``).

Run from inside a *-release repo working copy (where tracks.yaml lives):
    setup_tracks.py --devel <branch> --distros jazzy kilted rolling
"""
import argparse, copy, yaml

ap = argparse.ArgumentParser()
ap.add_argument('--devel', required=True, help='upstream devel branch (e.g. ros2, main)')
ap.add_argument('--distros', required=True, nargs='+')
ap.add_argument('--file', default='tracks.yaml')
args = ap.parse_args()

data = yaml.safe_load(open(args.file))
tracks = data['tracks']
template = tracks.get('humble') or next(iter(tracks.values()))

for d in args.distros:
    t = copy.deepcopy(template)
    t['ros_distro'] = d
    t['devel_branch'] = args.devel
    t['version'] = ':{auto}'
    t['release_tag'] = ':{version}'
    t['name'] = 'upstream'
    t['patches'] = None
    t['release_repo_url'] = None
    t['release_inc'] = '1'
    for k in ('last_version', 'last_release'):
        t.pop(k, None)
    tracks[d] = t

data['tracks'] = tracks
yaml.dump(data, open(args.file, 'w'), default_flow_style=False, sort_keys=True)
print(f"tracks set for {args.distros} (devel={args.devel})")
