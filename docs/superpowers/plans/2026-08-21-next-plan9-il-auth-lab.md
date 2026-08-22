# NeXT Lab Plan 9 IL Authentication Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provision and run a distinct authenticated Plan 9 lab profile that proves wrong-password rejection, correct-password root mounting, IL/566 and IL/17008 traffic, and visible guest completion without storing secrets in evidence.

**Architecture:** Preserve the existing TCP argv builder and add a strict IL builder using paths to a native key DB and QEMU Secret file. A bounded metadata-only PCAP parser identifies IL tuples. The baseline runner performs two clean, ordered VM runs and promotes evidence only when rejection and success gates both pass.

**Tech Stack:** Python 3 standard library, unittest, QEMU CLI/QMP, classic PCAP, GTK visible-run workflow.

---

### Task 1: Add a secret-safe authenticated argv builder

**Files:** Modify `nextcube_lab/plan9_netboot.py`, `tests/test_plan9_netboot.py`.

- [ ] Add failing tests proving existing `build_plan9_argv()` remains byte-identical and a new authenticated builder emits `transport=il,il-port=17008,auth-port=566,auth-id=p9fs,auth-domain=nextlab`, `keydb=PATH`, and `key-secret=plan9-master` plus `-object secret,id=plan9-master,file=PATH`.
- [ ] Add failing path tests for non-regular files, symlinks, group/world access, owner mismatch, missing paths, identical paths, and files under the result bundle.
- [ ] Implement validation using opened file descriptors plus `fstat`; require regular owner-only files and retain no file content. The builder accepts paths only and never accepts a key/password value.
- [ ] Assert serialized argv, reprs, exceptions, and test fixtures contain no secret payload.
- [ ] Run `python3 -B -m unittest tests.test_plan9_netboot`; commit with `git commit -m "plan9: build secret-safe authenticated IL launches"`.

### Task 2: Parse bounded IL metadata from PCAP

**Files:** Create `nextcube_lab/plan9_pcap.py`, `tests/test_plan9_pcap.py`.

- [ ] Add failing tests for classic little/big-endian PCAP, Ethernet/IPv4 options, protocol 40, IL header checksum/length, ports 566/17008, fragmentation rejection after capture reassembly is unavailable, truncated records, oversized capture lengths, and non-IL isolation.
- [ ] Implement a streaming parser with explicit per-packet and total-packet limits. Return timestamp, IPv4 source/destination, IL type, source/destination port, IL length, and checksum-valid only; never return payload bytes.
- [ ] Add helpers `saw_il_port(events, 566)` and `saw_il_port(events, 17008)` that accept either source or destination direction.
- [ ] Run `python3 -B -m unittest tests.test_plan9_pcap`; commit with `git commit -m "plan9: inspect IL metadata in packet captures"`.

### Task 3: Add the paired authenticated baseline workflow

**Files:** Modify `nextcube_lab/baseline/model.py`, `nextcube_lab/baseline/runner.py`, `nextcube_lab/baseline_cli.py`, `tests/test_baseline_runner.py`, `tests/test_plan9_netboot.py`.

- [ ] Add failing tests for an explicit authenticated Plan 9 profile requiring a parent pair ID, ordered `wrong-password` then clean `correct-password` runs, GTK/QMP gates, and refusal to reuse a VM/disk/run directory.
- [ ] Define the negative gate as both an observed authentication failure/password reprompt and absence of root-mount/desktop success markers before controlled shutdown. A timeout or crash alone is not rejection proof.
- [ ] Define the positive gate as root mounted plus terminal/desktop reached. Require PCAP metadata for both ports across the pair and port 17008 in the successful run.
- [ ] Keep passwords interactive and outside runner arguments/config. Sanitize `command.json`, summaries, transcripts, manifests, exception text, and environment captures; exclude key DB and Secret paths from bundle file copying.
- [ ] Run baseline/model/CLI tests; commit with `git commit -m "baseline: add paired Plan 9 authentication profile"`.

### Task 4: Document provisioning and launch

**Files:** Modify `README.md`; modify the authenticated Plan 9 example configuration to contain path-only examples.

- [ ] Document `qemu-plan9-keydb create`, chmod/ownership expectations, the QEMU Secret file boundary, the paired wrong/correct workflow, TCP compatibility, and the private SLiRP-only threat boundary.
- [ ] Add the exact archive root `https://ftp.osuosl.org/pub/plan9/history/` and the commands used to stage the Second Edition tree.
- [ ] State explicitly that passwords and all seven-byte keys must not enter shell argv, Git, QMP, logs, screenshots, transcripts, result manifests, or packet payload summaries.
- [ ] Run README command/executable/path checks from tests and `git diff --check`.
- [ ] Commit with `git commit -m "docs: add authenticated Plan 9 lab workflow"`.

### Task 5: Execute regression and visible acceptance

**Files:** Add only normal non-overwriting result metadata under the repository's established `results/plan9/` layout; never add key files.

- [ ] Run `python3 -B -m unittest discover -s tests`; expected: zero failures.
- [ ] Run the existing unauthenticated TCP/564 profile and prove its root/visible gates still pass.
- [ ] Provision fresh local-only `tor` and `p9fs` keys. Verify both files are 0600 and ignored/outside the result tree.
- [ ] Launch the wrong-password VM, enter an incorrect password interactively, observe the explicit negative gate, then shut down via authenticated QMP.
- [ ] Launch a fresh correct-password VM, enter tor's password interactively, observe root mount and terminal/desktop gates, then shut down cleanly.
- [ ] Parse captures and verify valid protocol-40 metadata for IL/566 and IL/17008. Store metadata only.
- [ ] Run a final recursive secret scan across the candidate bundle before promotion. Reject promotion on any key/password/ticket match or copied secret path.
- [ ] Invoke `superpowers:verification-before-completion`, record exact test/run IDs and commands, and commit only the accepted secret-free metadata if the repository's existing policy tracks it.
