#!/usr/bin/env python3
"""Finalize a release of a single-package ROS repo, the catkin way, but
without needing a TTY (unlike ``catkin_prepare_release``).

It renames the ``Forthcoming`` section of CHANGELOG.rst to the new version
with today's date, bumps ``<version>`` in every package.xml, commits as
"<version>", and creates a matching git tag.

Run ``catkin_generate_changelog -y`` first to populate the Forthcoming section.

Usage:
    finalize_release.py <repo_dir> <new_version> [YYYY-MM-DD]
"""
import sys, re, os, subprocess, datetime

repo_dir = sys.argv[1]
new_ver = sys.argv[2]
date = sys.argv[3] if len(sys.argv) > 3 else datetime.date.today().isoformat()
os.chdir(repo_dir)

# Bump <version> in every package.xml (mirrors catkin_prepare_release).
changed = []
for root, _, files in os.walk('.'):
    if '.git' in root:
        continue
    if 'package.xml' in files:
        p = os.path.join(root, 'package.xml')
        s = open(p).read()
        s2 = re.sub(r'(<version>)[^<]+(</version>)', r'\g<1>' + new_ver + r'\g<2>', s, count=1)
        if s2 != s:
            open(p, 'w').write(s2)
            changed.append(p)

# Promote the Forthcoming changelog section to the released version.
header = f"{new_ver} ({date})"
cl = open('CHANGELOG.rst').read()
cl2 = re.sub(r'Forthcoming\n-+\n', header + "\n" + "-" * len(header) + "\n", cl, count=1)
if cl2 == cl:
    sys.exit("ERROR: no 'Forthcoming' section found; run catkin_generate_changelog first")
open('CHANGELOG.rst', 'w').write(cl2)

subprocess.check_call(['git', 'add', 'CHANGELOG.rst', *changed])
subprocess.check_call(['git', 'commit', '-q', '-m', new_ver])
subprocess.check_call(['git', 'tag', new_ver])
print(f"finalized {changed} -> {new_ver}; tagged {new_ver}")
